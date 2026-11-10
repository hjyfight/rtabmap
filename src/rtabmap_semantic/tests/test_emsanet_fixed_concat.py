"""No-training fusion: exact values, original checkpoint, actual cat dataflow."""
from pathlib import Path
from types import SimpleNamespace
import unittest
from unittest.mock import patch

import numpy as np
try:
    import torch
    from torch import nn
    from nicr_mt_scene_analysis.model.encoder_fusion import get_encoder_fusion_class
    from rtabmap_semantic.emsanet_fusion import (
        EncoderRGBDFusionFixedConcat, FixedConcatProjection, use_fixed_concat_fusion,
        get_concat_fusion_class,
    )
except ImportError:
    torch = None

ROOT = Path(__file__).resolve().parents[3]
WEIGHTS = ROOT / 'models/emsanet/nyuv2_r34_NBt1D_pre.pth'


@unittest.skipIf(torch is None, 'EMSANet runtime required')
class FixedConcatTests(unittest.TestCase):
    def setUp(self):
        torch.manual_seed(8)
        torch.set_num_threads(2)

    def source(self, name='se-add-uni-rgb', layout='nchw', dtype=None):
        return get_encoder_fusion_class(name)(n_channels_in=32, input_memory_layout=layout).to(
            dtype=dtype or torch.float32).eval()

    def test_exact_values_routing_and_preserved_weights_for_both_layouts(self):
        for name in ('add', 'se-add', 'se-add-uni-rgb', 'se-add-uni-depth'):
            for layout in ('nchw', 'nhwc'):
                for dtype in (torch.float32, torch.float64):
                    with self.subTest(name=name, layout=layout, dtype=dtype):
                        source = self.source(name, layout, dtype)
                        shape = (2, 32, 7, 9) if layout == 'nchw' else (2, 7, 9, 32)
                        x = {k: torch.randn(shape, dtype=dtype) for k in ('rgb', 'depth')}
                        rng = torch.get_rng_state().clone()
                        fixed = EncoderRGBDFusionFixedConcat(source)
                        self.assertTrue(torch.equal(torch.get_rng_state(), rng))
                        self.assertEqual(dict(source.named_parameters()).keys(), dict(fixed.named_parameters()).keys())
                        for key, value in source.named_parameters():
                            self.assertIs(dict(fixed.named_parameters())[key], value)
                        fixed.load_state_dict(source.state_dict(), strict=True)
                        self.assertFalse(list(fixed.projection.parameters()))
                        self.assertFalse(fixed.projection.training)
                        with torch.inference_mode():
                            expected, actual = source(x), fixed(x)
                        for key in x:
                            torch.testing.assert_close(actual[key], expected[key], rtol=0, atol=0)
                        if name.endswith('uni-rgb'):
                            self.assertIs(actual['depth'], x['depth'])
                        elif name.endswith('uni-depth'):
                            self.assertIs(actual['rgb'], x['rgb'])

    def test_projection_consumes_real_concatenation_and_does_not_clip_negative_values(self):
        fixed = EncoderRGBDFusionFixedConcat(self.source('add'))
        rgb = -torch.ones(1, 32, 3, 4)
        depth = torch.full_like(rgb, -3)
        seen = []
        def modify_projection_input(module, args):
            seen.append(args[0].clone())
            # A bypass directly adding the original tensors would fail this test.
            return (args[0] + 1,)
        handle = fixed.projection.register_forward_pre_hook(modify_projection_input)
        with patch('torch.cat', wraps=torch.cat) as cat:
            actual = fixed({'rgb': rgb, 'depth': depth})['rgb']
        handle.remove()
        self.assertEqual(cat.call_count, 1)
        self.assertEqual(tuple(seen[0].shape), (1, 64, 3, 4))
        torch.testing.assert_close(seen[0][:, :32], rgb, rtol=0, atol=0)
        torch.testing.assert_close(seen[0][:, 32:], depth, rtol=0, atol=0)
        torch.testing.assert_close(actual, rgb + depth + 2, rtol=0, atol=0)

    def test_projection_has_no_learnable_state_and_keeps_input_gradients(self):
        projection = FixedConcatProjection(2, 'nchw')
        self.assertFalse(projection.state_dict())
        x = torch.randn(1, 4, 2, 3, requires_grad=True)
        projection(x).sum().backward()
        torch.testing.assert_close(x.grad, torch.ones_like(x), rtol=0, atol=0)
        with self.assertRaises(ValueError):
            projection(torch.zeros(1, 3, 2, 3))

    def test_conversion_is_idempotent_and_rejects_learned_concat(self):
        encoder = nn.Module()
        encoder.backbone_rgb, encoder.backbone_depth = nn.Identity(), nn.Identity()
        encoder.fusions = nn.ModuleList([self.source()])
        use_fixed_concat_fusion(encoder)
        first = encoder.fusions[0]
        use_fixed_concat_fusion(encoder)
        self.assertIs(first, encoder.fusions[0])
        learned = get_concat_fusion_class()(n_channels_in=32, input_memory_layout='nchw')
        encoder.fusions = nn.ModuleList([self.source(), learned])
        before = list(encoder.fusions)
        with self.assertRaises(ValueError):
            use_fixed_concat_fusion(encoder)
        self.assertEqual(before, list(encoder.fusions))
        with self.assertRaises(ValueError):
            use_fixed_concat_fusion(SimpleNamespace())


@unittest.skipUnless(torch is not None and WEIGHTS.is_file(), 'Official checkpoint/runtime required')
class FixedConcatRealModelTests(unittest.TestCase):
    def test_official_checkpoint_outputs_and_all_fusion_stages_are_unchanged(self):
        import cv2
        from rtabmap_semantic.emsanet_detector import EmsanetDetector
        image = cv2.imread(str(ROOT / '.tools/emsanet/samples/sample_rgb.png'))
        depth = cv2.imread(str(ROOT / '.tools/emsanet/samples/sample_depth.png'), -1)
        original = EmsanetDetector(WEIGHTS)
        fixed = EmsanetDetector(WEIGHTS, fusion='se-concat-fixed-uni-rgb')
        self.addCleanup(original.stage_timer.close)
        self.addCleanup(fixed.stage_timer.close)
        self.assertEqual(list(original.model.state_dict()), list(fixed.model.state_dict()))
        fixed.model.load_state_dict(original.model.state_dict(), strict=True)
        seen = []
        handles = []
        for layer in fixed.model.encoder.fusions:
            self.assertIsInstance(layer, EncoderRGBDFusionFixedConcat)
            handles.append(layer.projection.register_forward_pre_hook(
                lambda module, args: seen.append(tuple(args[0].shape))))
        expected = original.detect(image, depth)
        actual = fixed.detect(image, depth)
        for handle in handles:
            handle.remove()
        self.assertGreater(len(expected), 0)
        self.assertEqual([(d['label'], d['confidence']) for d in actual],
                         [(d['label'], d['confidence']) for d in expected])
        for a, b in zip(actual, expected):
            np.testing.assert_array_equal(a['mask'], b['mask'])
        self.assertEqual([shape[1] for shape in seen], [128, 128, 256, 512, 1024])
        self.assertEqual(fixed.last_timing['state'], 'valid')
        self.assertEqual(fixed.last_timing['fusion_name'], 'se-concat-fixed-uni-rgb')
        self.assertEqual(sum(s['group'] == 'rgbd_fusion' for s in fixed.last_timing['sections']), 5)
        # Check pre-decoding feature tensors too; final class labels alone are insufficient.
        batch = {'rgb': torch.randn(1, 3, 96, 128), 'depth': torch.randn(1, 1, 96, 128)}
        with torch.inference_mode():
            expected_features = original.model.encoder(batch)
            actual_features = fixed.model.encoder(batch)
        def tensors(value):
            if torch.is_tensor(value):
                yield value
            elif isinstance(value, dict):
                for item in value.values():
                    yield from tensors(item)
            elif isinstance(value, (tuple, list)):
                for item in value:
                    yield from tensors(item)
        a, b = list(tensors(actual_features)), list(tensors(expected_features))
        self.assertEqual(len(a), len(b))
        for actual_tensor, expected_tensor in zip(a, b):
            torch.testing.assert_close(actual_tensor, expected_tensor, rtol=0, atol=0)


if __name__ == '__main__':
    unittest.main()
