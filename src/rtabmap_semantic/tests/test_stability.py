import unittest

import numpy as np

from rtabmap_semantic.class_names_zh import CLASS_NAMES_ZH
from rtabmap_semantic.labels import canonical_label, chinese_label
from rtabmap_semantic.fusion import SemanticState


def detection(label='chair'):
    return dict(label=canonical_label(label), position=[0., 0., 2.],
                extent=[0.4, 0.4, 0.4], confidence=0.8, sigma=0.02)


class StableMappingTests(unittest.TestCase):
    def setUp(self):
        self.state = SemanticState(min_observations=4, confirmation_window=6,
                                   dynamic_min_observations=2)
        self.state.update_graph({i: np.eye(4) for i in range(1, 101)})

    def frame(self, i, labels):
        self.state.add_frame(i, float(i), [detection(label) for label in labels], now=float(i))

    def test_all_builtin_names_have_chinese_display(self):
        self.assertEqual(len(CLASS_NAMES_ZH), 4585)
        for name in CLASS_NAMES_ZH:
            self.assertTrue(any('\u3400' <= c <= '\u9fff' for c in chinese_label(canonical_label(name))), name)
        self.assertEqual(chinese_label('laptop'), '笔记本电脑')
        self.assertEqual(chinese_label('已有中文名称'), '已有中文名称')

    def test_aliases_keep_one_track_and_require_four_distinct_keyframes(self):
        for i, label in enumerate(['chair', 'office chair', 'computer chair', 'swivel chair'], 1):
            self.frame(i, [label])
            self.frame(i, [label])  # Replay/retry is not another observation.
            self.assertEqual(len(self.state.visible_objects(now=float(i))), int(i == 4))
        self.assertEqual(len(self.state.objects), 1)
        self.assertEqual(self.state.objects[0].label, 'chair')

    def test_sparse_false_positives_never_accumulate_to_confirmation(self):
        for i in range(1, 21):
            self.frame(i, ['chair'] if i % 4 == 1 else [])
            self.assertEqual(self.state.visible_objects(now=float(i)), [])

    def test_tentative_track_expires_but_confirmed_static_label_survives_misses(self):
        self.frame(1, ['chair'])
        for i in range(2, 8):
            self.frame(i, [])
        self.assertEqual(self.state.objects, [])
        for i in range(8, 12):
            self.frame(i, ['chair'])
        track_id = self.state.objects[0].object_id
        for i in range(12, 21):
            self.frame(i, [])
        self.assertEqual([o.object_id for o in self.state.visible_objects(now=20)], [track_id])

    def test_person_roles_are_dynamic_confirm_twice_then_expire(self):
        self.frame(1, ['composer'])
        self.assertEqual(self.state.visible_objects(now=1), [])
        self.frame(2, ['researcher'])
        person = self.state.visible_objects(now=2)[0]
        self.assertEqual(person.label, 'person')
        self.assertTrue(person.dynamic)
        self.assertEqual(len(person.observations), 1)
        self.assertEqual(self.state.to_dict()['objects'], [])
        self.assertEqual(self.state.visible_objects(now=6), [])

    def test_confirmation_uses_processed_frames_not_numeric_id_gaps(self):
        for i in [1, 10, 40, 90]:
            self.frame(i, ['chair'])
        self.assertEqual(len(self.state.visible_objects(now=90)), 1)

    def test_pending_confirmation_and_confirmed_state_survive_reload_and_reset(self):
        for i in range(1, 4):
            self.frame(i, ['chair'])
        saved = self.state.to_dict()
        self.state = SemanticState(min_observations=4, confirmation_window=6,
                                   dynamic_min_observations=2)
        self.state.restore(saved)
        self.state.update_graph({i: np.eye(4) for i in range(1, 10)})
        self.assertEqual(self.state.visible_objects(now=4), [])
        self.frame(4, ['chair'])
        self.assertEqual(len(self.state.visible_objects(now=4)), 1)
        self.state.restore(self.state.to_dict())
        self.assertEqual(len(self.state.visible_objects(now=4)), 1)
        self.state.add_frame(1, 101., [detection()], now=5)
        self.assertEqual(self.state.recent_nodes, [1])
        self.assertEqual(self.state.visible_objects(now=5), [])

    def test_legacy_saved_objects_do_not_need_new_confirmation_fields(self):
        old = SemanticState()
        old.update_graph(self.state.poses)
        for i in range(1, 4):
            old.add_frame(i, float(i), [detection()])
        saved = old.to_dict()
        self.assertNotIn('recent_nodes', saved)
        for obj in saved['objects']:
            self.assertNotIn('confirmation_ids', obj)
            self.assertNotIn('confirmed', obj)
        old.restore(saved)
        self.assertEqual(len(old.visible_objects()), 1)


if __name__ == '__main__':
    unittest.main()
