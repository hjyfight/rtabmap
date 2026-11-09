"""Concatenation fusion for EMSANet's dual RGB-D encoder.

EMSANet fuses the two encoder branches after every stage with an (optionally
SE-weighted) element-wise sum.  This module keeps that interface but replaces
the sum with channel concatenation followed by a 1x1 Conv-BN-Act projection:

    fused = Act(BN(Conv1x1(cat([w_rgb(x_rgb), w_depth(x_depth)], channel))))

so the network learns a per-channel linear mixing of both modalities instead
of a fixed 1:1 sum.  The third-party sources under ``.tools/`` are untouched;
``use_concat_fusion`` swaps the fusion modules of an already built encoder.

The pretrained NYUv2 checkpoint was trained with add fusion.  All its weights
still fit a concat model except the new projection layers, so a concat model
must be fine-tuned before it is used for mapping.
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
