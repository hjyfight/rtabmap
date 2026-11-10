"""Concatenation fusion for EMSANet's dual RGB-D encoder.

EMSANet fuses the two encoder branches after every stage with an (optionally
SE-weighted) element-wise sum.  This module keeps that interface but replaces
the sum with channel concatenation followed by a 1x1 Conv-BN-Act projection:

    fused = Act(BN(Conv1x1(cat([w_rgb(x_rgb), w_depth(x_depth)], channel))))

so the network learns a per-channel linear mixing of both modalities instead
of a fixed 1:1 sum.  The third-party sources under ``.tools/`` are untouched;
``use_concat_fusion`` swaps the fusion modules of an already built encoder.

The learned Conv-BN-Act variant needs fine-tuning. The separate fixed variant
below preserves the loaded SE modules and implements [I I] on concatenated
channels without new parameters, normalization, or activation. It is an
equivalent implementation of the original sum, not a new learned fusion.
"""
from typing import Type

import torch
from torch import nn

from nicr_mt_scene_analysis.model.activation import get_activation_class
from nicr_mt_scene_analysis.model.encoder_fusion import (
    EncoderRGBDFusionWeightedAdd, _apply_NCHW_operation,
)
from nicr_mt_scene_analysis.model.initialization import he_initialization
from nicr_mt_scene_analysis.model.normalization import get_normalization_class
from nicr_mt_scene_analysis.model.utils import ConvNormAct
from nicr_mt_scene_analysis.utils import partial_class


KNOWN_CONCAT_FUSIONS = (
    'concat', 'concat-uni-rgb', 'concat-uni-depth',
    'se-concat', 'se-concat-uni-rgb', 'se-concat-uni-depth',
)
# Channel axis of the encoder features for each supported memory layout.
_CHANNEL_AXIS = {'nchw': 1, 'nhwc': 3}


class EncoderRGBDFusionConcat(EncoderRGBDFusionWeightedAdd):
    """RGB-D encoder fusion: optional SE weighting, concat, 1x1 projection."""

    def __init__(
        self,
        n_channels_in: int,
        destinations,
        use_se_weighting: bool,
        input_memory_layout: str,
        normalization: Type[nn.Module] = get_normalization_class(),
        activation: Type[nn.Module] = get_activation_class(),
        **kwargs
    ) -> None:
        if input_memory_layout not in _CHANNEL_AXIS:
            raise ValueError(f'Unknown input_memory_layout: {input_memory_layout}')
        super().__init__(n_channels_in=n_channels_in, destinations=destinations,
                         use_se_weighting=use_se_weighting,
                         input_memory_layout=input_memory_layout,
                         activation=activation, **kwargs)
        # Input channels are ordered [rgb | depth]; output matches each branch
        # so the next backbone stage and the decoder skips stay unchanged.
        self.projection = ConvNormAct(2 * n_channels_in, n_channels_in, kernel_size=1,
                                      normalization=normalization, activation=activation)

    def _weighted(self, x, name):
        if not self._use_se_weighting:
            return x
        return _apply_NCHW_operation(x, getattr(self, f'weighting_{name}'),
                                     self._input_memory_layout)

    def forward(self, x):
        x_rgb, x_depth = x['rgb'], x['depth']
        if x_rgb.shape != x_depth.shape:
            # Same-scale fusion: both encoders must emit identical feature maps.
            raise ValueError(f'RGB/depth feature shapes differ: '
                             f'{tuple(x_rgb.shape)} vs {tuple(x_depth.shape)}')
        stacked = torch.cat((self._weighted(x_rgb, 'rgb'), self._weighted(x_depth, 'depth')),
                            dim=_CHANNEL_AXIS[self._input_memory_layout])
        fused = _apply_NCHW_operation(stacked, self.projection, self._input_memory_layout)
        return {'rgb': fused if 'rgb' in self._destinations else x_rgb,
                'depth': fused if 'depth' in self._destinations else x_depth}


def get_concat_fusion_class(name: str = 'se-concat-uni-rgb', **kwargs):
    """Mirror of EMSANet's get_encoder_fusion_class naming for concat variants."""
    name = name.lower()
    if name not in KNOWN_CONCAT_FUSIONS:
        raise ValueError(f"Unknown concat encoder fusion: '{name}'")
    kwargs['use_se_weighting'] = name.startswith('se-')
    if name.endswith('uni-rgb'):
        kwargs['destinations'] = ('rgb',)
    elif name.endswith('uni-depth'):
        kwargs['destinations'] = ('depth',)
    else:
        kwargs['destinations'] = ('rgb', 'depth')
    return partial_class(EncoderRGBDFusionConcat, **kwargs)


def use_concat_fusion(encoder, name: str = 'se-concat-uni-rgb',
                      normalization: str = 'batchnorm', activation: str = 'relu',
                      he_init: bool = True):
    """Replace every stage fusion of a FusedRGBDEncoder with concat fusion.

    Pass the same normalization/activation as the EMSANet args
    (``--encoder-normalization``, ``--activation``).  Returns the encoder.
    """
    rgb, depth = getattr(encoder, 'backbone_rgb', None), getattr(encoder, 'backbone_depth', None)
    if rgb is None or depth is None or not hasattr(encoder, 'fusions'):
        raise ValueError('Concat fusion needs a dual-backbone FusedRGBDEncoder')
    fusion = get_concat_fusion_class(name)
    modules = nn.ModuleList(
        fusion(n_channels_in=n, input_memory_layout=layout,
               normalization=get_normalization_class(normalization),
               activation=get_activation_class(activation))
        for n, layout in zip(rgb.stages_n_channels, rgb.stages_memory_layout))
    if he_init:
        # EMSANet He-initializes its encoder fusions by default (--he-init).
        for index, module in enumerate(modules):
            he_initialization(module, name_hint=f'encoder.fusions.{index}', debug=False)
    reference = next(encoder.parameters())
    # New modules default to train mode; follow the encoder (e.g. eval BN stats).
    encoder.fusions = modules.to(device=reference.device, dtype=reference.dtype).train(encoder.training)
    return encoder


class FixedConcatProjection(nn.Module):
    """Parameter-free sparse linear projection [I I]: 2C -> C.

    Evaluate only the two nonzero entries of each matrix row. This avoids a
    dense convolution's zero products/reduction order and preserves the original
    pairwise addition, including negative values. The input is the *actual*
    concatenated tensor; no bypass of the concatenation is used.
    """

    def __init__(self, n_channels, input_memory_layout):
        super().__init__()
        if n_channels <= 0 or input_memory_layout not in _CHANNEL_AXIS:
            raise ValueError('Invalid fixed concat projection channels/layout')
        self.n_channels = n_channels
        self.channel_axis = _CHANNEL_AXIS[input_memory_layout]

    def forward(self, stacked):
        if stacked.ndim != 4 or stacked.shape[self.channel_axis] != 2 * self.n_channels:
            raise ValueError('Fixed concat projection requires a 4D tensor with 2C channels')
        rgb = stacked.narrow(self.channel_axis, 0, self.n_channels)
        depth = stacked.narrow(self.channel_axis, self.n_channels, self.n_channels)
        return rgb + depth


class EncoderRGBDFusionFixedConcat(nn.Module):
    """Reuse an existing weighted-add layer's SE weights and output routing."""

    def __init__(self, source):
        super().__init__()
        if not isinstance(source, EncoderRGBDFusionWeightedAdd) or hasattr(source, 'projection'):
            raise ValueError('Fixed concat conversion requires an original weighted-add layer')
        self._n_channels_in = source._n_channels_in
        self._input_memory_layout = source._input_memory_layout
        self._use_se_weighting = source._use_se_weighting
        self._destinations = source._destinations
        if self._use_se_weighting:
            # Same module objects, parameters, device, dtype, and state_dict keys.
            self.weighting_rgb = source.weighting_rgb
            self.weighting_depth = source.weighting_depth
        self.projection = FixedConcatProjection(self._n_channels_in, self._input_memory_layout)
        self.projection.train(source.training)
        self.training = source.training

    def forward(self, x):
        rgb, depth = x['rgb'], x['depth']
        axis = _CHANNEL_AXIS[self._input_memory_layout]
        if rgb.ndim != 4 or rgb.shape != depth.shape or rgb.shape[axis] != self._n_channels_in:
            raise ValueError('Fixed concat fusion requires matching RGB/depth feature scales')
        if self._use_se_weighting:
            rgb_weighted = _apply_NCHW_operation(rgb, self.weighting_rgb, self._input_memory_layout)
            depth_weighted = _apply_NCHW_operation(depth, self.weighting_depth, self._input_memory_layout)
        else:
            rgb_weighted, depth_weighted = rgb, depth
        stacked = torch.cat((rgb_weighted, depth_weighted), dim=axis)
        fused = self.projection(stacked)
        return {'rgb': fused if 'rgb' in self._destinations else rgb,
                'depth': fused if 'depth' in self._destinations else depth}


def use_fixed_concat_fusion(encoder):
    """Convert loaded add fusion in place, preserving every checkpoint tensor.

    Call before attaching timing hooks. Converting twice is a no-op; learned
    concat layers and incompatible encoders are rejected before any mutation.
    """
    if (getattr(encoder, 'backbone_rgb', None) is None or
            getattr(encoder, 'backbone_depth', None) is None or
            not hasattr(encoder, 'fusions') or not len(encoder.fusions)):
        raise ValueError('Fixed concat fusion needs a dual-backbone FusedRGBDEncoder')
    if all(isinstance(f, EncoderRGBDFusionFixedConcat) for f in encoder.fusions):
        return encoder
    replacements = [EncoderRGBDFusionFixedConcat(f) for f in encoder.fusions]
    modules = nn.ModuleList(replacements)
    modules.training = encoder.fusions.training
    encoder.fusions = modules
    return encoder
