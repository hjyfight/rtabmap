from concurrent.futures import ThreadPoolExecutor
from itertools import count
import importlib.util
from pathlib import Path
from types import SimpleNamespace
import unittest

import numpy as np
try:
    import torch
    from torch import nn
except ImportError:
    torch = None
    nn = SimpleNamespace(Module=object)

from rtabmap_semantic.emsanet_timing import EmsanetStageTimer


class Fusion(nn.Module):
    def forward(self, x):
        return {'rgb': x['rgb']+x['depth'], 'depth': x['depth']}


class Model(nn.Module):
    def __init__(self):
        super().__init__()
        self.args = SimpleNamespace(encoder_fusion='test-add')
        self.encoder = nn.Module()
        for name in ('backbone_rgb', 'backbone_depth'):
            backbone = nn.Module()
            backbone.stages = nn.ModuleList([nn.Linear(4, 4), nn.Linear(4, 4)])
            setattr(self.encoder, name, backbone)
        self.encoder.fusions = nn.ModuleList([Fusion(), Fusion()])
        self.context_module = nn.ReLU()
        self.decoders = nn.ModuleDict({'head': nn.Linear(4, 4)})
        self.fail = False

    def forward(self, rgb, depth):
        if self.fail:
            raise RuntimeError('test inference failure')
        x = {'rgb':rgb, 'depth':depth}
        for i in range(2):
            x['rgb'] = self.encoder.backbone_rgb.stages[i](x['rgb'])
            x['depth'] = self.encoder.backbone_depth.stages[i](x['depth'])
            x = self.encoder.fusions[i](x)
        return self.decoders['head'](self.context_module(x['rgb']))


@unittest.skipIf(torch is None, 'EMSANet torch runtime is unavailable')
class StageTimingTests(unittest.TestCase):
    def setUp(self):
        self.model = Model().eval()
        self.inputs = (torch.randn(1,4), torch.randn(1,4))

    def test_exact_disjoint_accounting_and_identical_output(self):
        expected = self.model(*self.inputs).detach()
        keys = list(self.model.state_dict())
        clock = count(1000, 1000).__next__
        timer = EmsanetStageTimer(self.model, clock=clock)
        result, report = timer.measure(lambda: self.model(*self.inputs))
        torch.testing.assert_close(result, expected, rtol=0, atol=0)
        self.assertEqual(list(self.model.state_dict()), keys)
        self.assertEqual(report['state'], 'valid')
        g = report['groups_ms']
        self.assertEqual(len(report['sections']), 8)
        self.assertAlmostEqual(g['rgb_backbone'], .002)
        self.assertAlmostEqual(g['depth_backbone'], .002)
        self.assertAlmostEqual(g['feature_extraction'], .004)
        self.assertAlmostEqual(g['rgbd_fusion'], .002)
        self.assertAlmostEqual(g['model_forward'], .017)
        self.assertAlmostEqual(g['model_other'], .009)
        timer.close()
        self.assertFalse(timer.handles)

    def test_missing_coverage_is_invalid_not_zero_time(self):
        timer = EmsanetStageTimer(self.model)
        _, report = timer.measure(lambda: self.model.context_module(self.inputs[0]))
        self.assertEqual(report['state'], 'invalid')
        timer.close()

    def test_failed_inference_does_not_poison_next_worker_call(self):
        timer = EmsanetStageTimer(self.model)
        self.model.fail = True
        with self.assertRaises(RuntimeError):
            timer.measure(lambda: self.model(*self.inputs))
        self.model.fail = False
        with ThreadPoolExecutor(max_workers=1) as pool:
            _, report = pool.submit(timer.measure, lambda: self.model(*self.inputs)).result()
        self.assertEqual(report['state'], 'valid')
        self.assertEqual(len(report['sections']), 8)
        timer.close()

    def test_gpu_launches_are_not_reported_as_cpu_execution(self):
        timer = EmsanetStageTimer(self.model, device='cuda:0')
        _, report = timer.measure(lambda: self.model(*self.inputs))
        self.assertEqual(report['state'], 'unsupported_device')
        self.assertNotIn('groups_ms', report)

    def test_rejects_parent_child_double_counting(self):
        self.model.decoders['bad'] = self.model.encoder.backbone_rgb.stages[0]
        with self.assertRaises(ValueError):
            EmsanetStageTimer(self.model)


@unittest.skipUnless(torch is not None and importlib.util.find_spec('emsanet') is not None and (Path(__file__).resolve().parents[3]/
                     'models/emsanet/nyuv2_r34_NBt1D_pre.pth').is_file(),
                     'Official checkpoint and EMSANet runtime are required')
class RealModelTimingTests(unittest.TestCase):
    def test_real_checkpoint_masks_unchanged_and_all_stages_covered(self):
        import cv2
        from rtabmap_semantic.emsanet_detector import EmsanetDetector
        root = Path(__file__).resolve().parents[3]
        detector = EmsanetDetector(root/'models/emsanet/nyuv2_r34_NBt1D_pre.pth')
        image = cv2.imread(str(root/'.tools/emsanet/samples/sample_rgb.png'))
        depth = cv2.imread(str(root/'.tools/emsanet/samples/sample_depth.png'), -1)
        self.assertIsNotNone(image)
        measured = detector.detect(image, depth)
        timing = detector.last_timing
        self.assertEqual(timing['state'], 'valid', timing)
        self.assertEqual(timing['fusion_name'], 'se-add-uni-rgb')
        self.assertEqual(sum(r['group']=='rgbd_fusion' for r in timing['sections']), 5)
        self.assertEqual(sum(r['group']=='rgb_backbone' for r in timing['sections']), 5)
        self.assertEqual(sum(r['group']=='depth_backbone' for r in timing['sections']), 5)
        self.assertLess(timing['groups_ms']['model_forward'], timing['detector_total_ms'])
        detector.stage_timer.close()
        detector.stage_timer = SimpleNamespace(measure=lambda fn: (fn(), {'state':'disabled'}))
        plain = detector.detect(image, depth)
        self.assertGreater(len(measured), 0)
        self.assertEqual([(d['label'],d['confidence']) for d in measured],
                         [(d['label'],d['confidence']) for d in plain])
        for a,b in zip(measured, plain):
            np.testing.assert_array_equal(a['mask'],b['mask'])
        detector.detect(image, np.zeros_like(depth))
        self.assertIsNone(detector.last_timing)


if __name__ == '__main__':
    unittest.main()
