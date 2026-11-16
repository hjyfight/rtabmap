from copy import deepcopy
from pathlib import Path
import sys
import unittest
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src/rtabmap_semantic'))
from rtabmap_semantic.segmentation import prepare_instances, dense_maps, render_images, select_candidate


class SegmentationTests(unittest.TestCase):
    def setUp(self):
        self.labels = {0: 'person', 1: 'chair'}
        mask = np.zeros((4, 7), np.uint8)
        mask[1:3, 2:6] = 1
        rows = [dict(mask=mask, confidence=.9, class_id=0, bbox_xyxy=[2, 1, 6, 3])]
        self.instances = prepare_instances(rows, mask.shape, self.labels)
        self.baseline = dict(session_id='fixture', sequence=1, stamp_ns=10_000_000_000,
            frame_id='camera', model_sha256='fixture', vocabulary_version='v1',
            instances=self.instances, revision=0, result_source='rgb_yolo')
        self.candidate = {k: v for k, v in self.baseline.items() if k != 'instances'}
        self.candidate.update(provider_id='test-provider', calibration_version='test-calibration',
            score_calibration_version='test-score-calibration', refinements=[dict(instance_id=1,
                class_id=1, score=.91, valid_mask=[True, True, False, False], weights=[.5,.5,0,0],
                source_stamps=[10.,10.,0.,0.], time_alignment_valid=True, spatial_alignment_valid=True)])

    def select(self, candidate=None, **options):
        settings = dict(mode='guarded', approved=['test-provider'], current=True,
                        age=.1, timeout=1., tolerance=.05, min_score=.8)
        settings.update(options)
        return select_candidate(candidate or self.candidate, self.baseline, self.labels, **settings)

    def test_non_square_original_coordinates_overlap_and_unknown(self):
        other = deepcopy(self.instances[0])
        other.update(instance_id=2, class_id=1, confidence=.8)
        classes, identities = dense_maps([other, *self.instances], (4, 7))
        self.assertEqual(classes[0, 0], -1)
        self.assertEqual(classes[1, 2], 0)
        self.assertEqual(identities[1, 2], 1)
        self.assertEqual(int((classes == 0).sum()), 8)
        rgb = np.full((4,7,3), 100, np.uint8)
        overlay, _ = render_images(rgb, classes)
        np.testing.assert_array_equal(overlay[0,0], rgb[0,0])
        self.assertFalse(np.array_equal(overlay[1,2], rgb[1,2]))

    def test_rejects_network_size_masks_instead_of_silently_stretching(self):
        row = dict(mask=np.ones((640,640),np.uint8), confidence=.9, class_id=0,bbox_xyxy=[0,0,7,4])
        with self.assertRaisesRegex(ValueError, 'original image'):
            prepare_instances([row], (4,7), self.labels)
        classes, identities = dense_maps([], (4,7))
        self.assertTrue((classes == -1).all())
        self.assertTrue((identities == 0).all())

    def test_guarded_refinement_only_changes_classification_not_geometry_or_yolo_score(self):
        selected, reason = self.select()
        self.assertEqual(reason, 'fusion_selected')
        self.assertEqual(selected['instances'][0]['class_id'], 1)
        for key in ('mask', 'bbox_xyxy', 'mask_offset', 'confidence', 'instance_id'):
            np.testing.assert_array_equal(selected['instances'][0][key], self.instances[0][key])
        self.assertEqual(self.baseline['instances'][0]['class_id'], 0)

    def test_shadow_off_unapproved_and_superseded_preserve_identical_baseline(self):
        for options, reason in [({'mode':'shadow'},'shadow_candidate'), ({'mode':'off'},'fusion_disabled'),
                                ({'approved':[]},'provider_not_approved'), ({'current':False},'superseded_frame'),
                                ({'age':2},'candidate_expired')]:
            selected, actual = self.select(**options)
            self.assertIs(selected, self.baseline)
            self.assertEqual(actual, reason)

    def test_rejects_wrong_frame_model_vocabulary_and_unreliable_modalities(self):
        mutations = [lambda c: c.update(session_id='other'), lambda c: c.update(sequence=2),
            lambda c: c.update(stamp_ns=11), lambda c: c.update(frame_id='nir'),
            lambda c: c.update(model_sha256='other'), lambda c: c.update(vocabulary_version='other'),
            lambda c: c.update(calibration_version='unverified'),
            lambda c: c['refinements'][0].update(instance_id=9),
            lambda c: c['refinements'][0].update(class_id=99),
            lambda c: c['refinements'][0].update(score=float('nan')),
            lambda c: c['refinements'][0].update(score=.5),
            lambda c: c['refinements'][0].update(weights=[1,1,0,0]),
            lambda c: c['refinements'][0].update(valid_mask=[True,False,False,False]),
            lambda c: c['refinements'][0].update(valid_mask=[True,False,False,True]),
            lambda c: c['refinements'][0].update(source_stamps=[10,11,0,0]),
            lambda c: c['refinements'][0].update(spatial_alignment_valid=False),
            lambda c: c['refinements'].append(deepcopy(c['refinements'][0]))]
        for mutate in mutations:
            with self.subTest(mutation=mutate):
                candidate = deepcopy(self.candidate)
                mutate(candidate)
                selected, reason = self.select(candidate)
                self.assertIs(selected, self.baseline)
                self.assertNotEqual(reason, 'fusion_selected')


if __name__ == '__main__':
    unittest.main()
