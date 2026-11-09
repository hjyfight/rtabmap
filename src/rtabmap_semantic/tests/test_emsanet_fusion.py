"""Correctness tests for the concat (cat + 1x1 conv) EMSANet encoder fusion.

Needs the EMSANet runtime:
  .venv-emsanet/bin/python -m unittest discover -s src/rtabmap_semantic/tests -p 'test_emsanet_fusion.py' -v
"""
import os
from pathlib import Path
import unittest

try:
    import torch
    import torch.nn.functional as F
    from nicr_mt_scene_analysis.model.encoder_fusion import EncoderRGBDFusionWeightedAdd
    from rtabmap_semantic.emsanet_fusion import (
        EncoderRGBDFusionConcat, KNOWN_CONCAT_FUSIONS, get_concat_fusion_class, use_concat_fusion,
    )
except ImportError:  # system python3 runs the torch-free test suite
    torch = None

PROJECT = Path(__file__).resolve().parents[3]
WEIGHTS = PROJECT / 'models/emsanet/nyuv2_r34_NBt1D_pre.pth'
N = 32  # SE reduction is 16, so channels must be >= 16


def make(name, layout='nchw', n=N):
    return get_concat_fusion_class(name)(n_channels_in=n, input_memory_layout=layout)


def tensors(value):
    """Flatten every tensor in EMSANet's nested outputs."""
    if torch.is_tensor(value):
        yield value
    elif isinstance(value, dict):
        for item in value.values():
            yield from tensors(item)
    elif isinstance(value, (list, tuple)):
        for item in value:
            yield from tensors(item)


@unittest.skipIf(torch is None, 'EMSANet runtime (torch) is not available')
class ConcatFusionModuleTests(unittest.TestCase):
    def setUp(self):
        torch.manual_seed(0)
        self.rgb = torch.randn(2, N, 6, 8)
        self.depth = torch.randn(2, N, 6, 8)

    def test_is_subclass_with_doubled_input_projection(self):
        module = make('se-concat-uni-rgb')
        self.assertIsInstance(module, EncoderRGBDFusionWeightedAdd)
        self.assertIsInstance(module, EncoderRGBDFusionConcat)
        self.assertEqual(tuple(module.projection.conv.weight.shape), (N, 2 * N, 1, 1))
        self.assertTrue(hasattr(module, 'weighting_rgb') and hasattr(module, 'weighting_depth'))
        self.assertFalse(hasattr(make('concat'), 'weighting_rgb'))

    def test_output_matches_hand_computed_se_cat_conv_bn_relu(self):
        for name in ('concat', 'se-concat'):
            with self.subTest(name=name):
                module = make(name)
                # Non-trivial BN statistics so the eval path is actually exercised.
                bn = module.projection.norm
                bn.running_mean.uniform_(-0.5, 0.5)
                bn.running_var.uniform_(0.5, 2.0)
                bn.weight.data.uniform_(0.5, 1.5)
                bn.bias.data.uniform_(-0.2, 0.2)
                module.eval()
                rgb, depth = self.rgb, self.depth
                if name.startswith('se-'):
                    def se(x, block):
                        return x * block.layers(F.adaptive_avg_pool2d(x, 1))
                    rgb, depth = se(rgb, module.weighting_rgb), se(depth, module.weighting_depth)
                conv = F.conv2d(torch.cat((rgb, depth), 1), module.projection.conv.weight)
                expected = F.relu(F.batch_norm(conv, bn.running_mean, bn.running_var,
                                               bn.weight, bn.bias, False, 0.0, bn.eps))
                with torch.no_grad():
                    out = module({'rgb': self.rgb, 'depth': self.depth})
                torch.testing.assert_close(out['rgb'], expected)
                torch.testing.assert_close(out['depth'], expected)

    def test_channel_order_is_rgb_then_depth_and_not_a_sum(self):
        eye = torch.eye(N).view(N, N, 1, 1)
        for selected, half in (('rgb', slice(0, N)), ('depth', slice(N, 2 * N))):
            with self.subTest(selected=selected):
                module = make('concat').eval()
                weight = torch.zeros(N, 2 * N, 1, 1)
                weight[:, half] = eye
                module.projection.conv.weight.data.copy_(weight)
                rgb, depth = self.rgb.abs(), self.depth.abs()  # ReLU is identity here
                with torch.no_grad():
                    out = module({'rgb': rgb, 'depth': depth})['rgb']
                source = rgb if selected == 'rgb' else depth
                scale = (1 + module.projection.norm.eps) ** -0.5
                torch.testing.assert_close(out, source * scale)
                # Changing only the ignored modality must not change the result.
                other = {'rgb': rgb, 'depth': depth + 5} if selected == 'rgb' else \
                    {'rgb': rgb + 5, 'depth': depth}
                with torch.no_grad():
                    torch.testing.assert_close(module(other)['rgb'], out)

    def test_destinations_follow_emsanet_naming(self):
        for name in KNOWN_CONCAT_FUSIONS:
            with self.subTest(name=name):
                out = make(name).eval()({'rgb': self.rgb, 'depth': self.depth})
                fused = out['depth'] if name.endswith('uni-depth') else out['rgb']
                self.assertEqual(out['rgb'].shape, self.rgb.shape)
                self.assertEqual(out['depth'].shape, self.depth.shape)
                if name.endswith('uni-rgb'):
                    self.assertIs(out['depth'], self.depth)
                elif name.endswith('uni-depth'):
                    self.assertIs(out['rgb'], self.rgb)
                else:
                    self.assertIs(out['rgb'], out['depth'])
                self.assertFalse(torch.equal(fused, self.rgb))

    def test_channels_last_layout_matches_nchw(self):
        for name in ('concat', 'se-concat-uni-rgb'):
            with self.subTest(name=name):
                nchw, nhwc = make(name, 'nchw').eval(), make(name, 'nhwc').eval()
                nhwc.load_state_dict(nchw.state_dict())
                with torch.no_grad():
                    expected = nchw({'rgb': self.rgb, 'depth': self.depth})['rgb']
                    out = nhwc({'rgb': self.rgb.permute(0, 2, 3, 1),
                                'depth': self.depth.permute(0, 2, 3, 1)})['rgb']
                torch.testing.assert_close(out, expected.permute(0, 2, 3, 1))

    def test_gradients_reach_both_modalities_and_projection(self):
        module = make('se-concat-uni-rgb').train()
        rgb, depth = self.rgb.clone().requires_grad_(), self.depth.clone().requires_grad_()
        module({'rgb': rgb, 'depth': depth})['rgb'].square().mean().backward()
        for grad in (rgb.grad, depth.grad, module.projection.conv.weight.grad):
            self.assertIsNotNone(grad)
            self.assertGreater(float(grad.abs().sum()), 0)

    def test_invalid_names_layouts_and_mismatched_scales_are_rejected(self):
        with self.assertRaises(ValueError):
            get_concat_fusion_class('add')
        with self.assertRaises(ValueError):
            make('concat', layout='chw')
        with self.assertRaises(ValueError):
            make('concat')({'rgb': self.rgb, 'depth': self.depth[..., :4]})


def build_emsanet():
    from emsanet.args import ArgParserEMSANet
    from emsanet.data import get_dataset
    from emsanet.model import EMSANet
    # Same architecture as rtabmap_semantic.emsanet_detector.
    args = ArgParserEMSANet().parse_args([
        '--dataset', 'nyuv2', '--input-modalities', 'rgb', 'depth',
        '--tasks', 'semantic', 'instance', 'orientation', 'scene', '--enable-panoptic',
        '--rgb-encoder-backbone', 'resnet34', '--depth-encoder-backbone', 'resnet34',
        '--rgb-encoder-backbone-resnet-block', 'nonbottleneck1d',
        '--depth-encoder-backbone-resnet-block', 'nonbottleneck1d',
        '--no-pretrained-backbone',
    ], verbose=False)
    dataset = get_dataset(args, args.validation_split)
    return args, EMSANet(args, dataset_config=dataset.config)


@unittest.skipIf(torch is None, 'EMSANet runtime (torch) is not available')
class ConcatFusionInEmsanetTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        torch.manual_seed(0)
        torch.set_num_threads(max(1, min(4, os.cpu_count() or 1)))
        cls.args, cls.add_model = build_emsanet()
        cls.add_model.eval()
        _, cls.concat_model = build_emsanet()
        use_concat_fusion(cls.concat_model.encoder, 'se-concat-uni-rgb',
                          cls.args.encoder_normalization, cls.args.activation)
        cls.concat_model.eval()
        cls.batch = {'rgb': torch.randn(1, 3, 96, 128), 'depth': torch.randn(1, 1, 96, 128)}

    def test_every_stage_uses_concat_with_backbone_channels(self):
        fusions = self.concat_model.encoder.fusions
        channels = self.concat_model.encoder.backbone_rgb.stages_n_channels
        self.assertEqual(len(fusions), len(channels))
        for fusion, n in zip(fusions, channels):
            self.assertIsInstance(fusion, EncoderRGBDFusionConcat)
            self.assertEqual(tuple(fusion.projection.conv.weight.shape), (n, 2 * n, 1, 1))
            self.assertFalse(fusion.training)
        added = sum(p.numel() for f in fusions for p in f.projection.parameters())
        self.assertEqual(added, sum(2 * n * n + 2 * n for n in channels))

    def test_encoder_features_and_skip_scales_match_add_fusion(self):
        inputs = {'rgb': self.batch['rgb'], 'depth': self.batch['depth']}
        with torch.no_grad():
            add_out, add_skips = self.add_model.encoder(inputs)
            cat_out, cat_skips = self.concat_model.encoder(inputs)
        self.assertEqual({k: v.shape for k, v in add_out.items()},
                         {k: v.shape for k, v in cat_out.items()})
        self.assertEqual(set(add_skips), set(cat_skips))
        for scale in add_skips:
            for modality in ('rgb', 'depth'):
                self.assertEqual(add_skips[scale][modality].shape, cat_skips[scale][modality].shape)

    def test_full_network_forward_shapes_match_and_are_finite(self):
        with torch.no_grad():
            add_outputs = list(tensors(self.add_model(self.batch)))
            cat_outputs = list(tensors(self.concat_model(self.batch)))
        self.assertTrue(cat_outputs)
        self.assertEqual([t.shape for t in add_outputs], [t.shape for t in cat_outputs])
        self.assertTrue(all(torch.isfinite(t).all() for t in cat_outputs if t.is_floating_point()))

    def test_training_backward_reaches_both_encoders_through_concat(self):
        _, model = build_emsanet()
        use_concat_fusion(model.encoder, 'se-concat-uni-rgb')
        model.train()
        batch = {'rgb': torch.randn(2, 3, 64, 96), 'depth': torch.randn(2, 1, 64, 96)}
        loss = sum(t.float().mean() for t in tensors(model(batch)) if t.is_floating_point())
        loss.backward()
        encoder = model.encoder
        for name, parameter in (('rgb', next(encoder.backbone_rgb.parameters())),
                                ('depth', next(encoder.backbone_depth.parameters())),
                                ('projection', encoder.fusions[0].projection.conv.weight)):
            with self.subTest(parameter=name):
                self.assertIsNotNone(parameter.grad)
                self.assertGreater(float(parameter.grad.abs().sum()), 0)

    @unittest.skipUnless(WEIGHTS.is_file(), 'NYUv2 EMSANet checkpoint is not installed')
    def test_pretrained_checkpoint_only_lacks_new_projection_layers(self):
        state = torch.load(str(WEIGHTS), map_location='cpu', weights_only=True)
        state = state.get('state_dict', state)
        state = {k.replace('fused_encoders.', 'encoder.'): v for k, v in state.items()}
        _, model = build_emsanet()
        model.load_state_dict(state, strict=True)  # the unchanged add model still loads
        use_concat_fusion(model.encoder)
        with self.assertRaises(RuntimeError):
            model.load_state_dict(state, strict=True)
        result = model.load_state_dict(state, strict=False)
        self.assertEqual(result.unexpected_keys, [])
        self.assertTrue(result.missing_keys)
        self.assertTrue(all('.projection.' in key for key in result.missing_keys), result.missing_keys)
        # SE weighting of the add checkpoint is reused by the concat fusion.
        torch.testing.assert_close(model.encoder.fusions[2].weighting_depth.layers[0].weight,
                                   state['encoder.fusions.2.weighting_depth.layers.0.weight'])


if __name__ == '__main__':
    unittest.main()
