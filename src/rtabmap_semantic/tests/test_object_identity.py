import tempfile
import unittest
from pathlib import Path

import numpy as np

from rtabmap_semantic.fusion import SemanticState, SemanticStore


def detection(label='monitor', x=0., size=0.4, confidence=0.8):
    return dict(label=label, position=[x, 0., 2.], extent=[size, size, size],
                sigma=0.02, confidence=confidence)


class ObjectIdentityTests(unittest.TestCase):
    def state(self, unique=True):
        state = SemanticState(min_observations=4, confirmation_window=6,
                              dynamic_min_observations=2, unique_objects=unique)
        state.update_graph({i: np.eye(4) for i in range(1, 101)})
        return state

    def test_alternating_names_keep_one_object_and_one_stable_label(self):
        state = self.state()
        for i in range(1, 21):
            label = 'monitor' if i % 2 else 'television'
            state.add_frame(i, float(i), [detection(label, x=0.02*(i % 2))])
            self.assertEqual(len(state.objects), 1)
            self.assertEqual(state.objects[0].object_id, 1)
        self.assertEqual([o.label for o in state.visible_objects()], ['monitor'])

    def test_label_change_needs_multiple_frames_and_keeps_identity(self):
        state = self.state()
        for i in range(1, 5):
            state.add_frame(i, float(i), [detection('monitor', confidence=0.5)])
        for i in range(5, 8):
            state.add_frame(i, float(i), [detection('television', confidence=0.95)])
            self.assertEqual(state.objects[0].label, 'monitor')
        state.add_frame(8, 8., [detection('television', confidence=0.95)])
        self.assertEqual([(o.object_id, o.label) for o in state.visible_objects()], [(1, 'television')])

    def test_nearby_coobserved_objects_remain_separate_after_reconciliation(self):
        state = self.state()
        for i in range(1, 5):
            state.add_frame(i, float(i), [detection('monitor'), detection('television', x=0.04)])
        self.assertFalse(state.reconcile())
        self.assertEqual(len(state.visible_objects()), 2)

    def test_distant_or_contained_objects_are_not_cross_associated(self):
        for new in [detection('television', x=0.35), detection('laptop', size=0.1)]:
            with self.subTest(new=new):
                state = self.state()
                for i in range(1, 5):
                    state.add_frame(i, float(i), [detection('desk')])
                for i in range(5, 9):
                    state.add_frame(i, float(i), [new])
                self.assertFalse(state.reconcile())
                self.assertEqual(len(state.visible_objects()), 2)

    def test_old_cross_class_duplicates_merge_when_geometry_becomes_available(self):
        old = self.state(unique=False)
        for i in range(1, 5):
            old.add_frame(i, float(i), [detection('mobility scooter')])
        for i in range(5, 9):
            old.add_frame(i, float(i), [detection('television camera', x=0.04)])
        state = self.state()
        state.restore(old.to_dict())
        self.assertTrue(state.reconcile())
        self.assertEqual([o.object_id for o in state.visible_objects()], [1])
        self.assertEqual({o.label for o in state.objects[0].observations}, {'mobility scooter', 'television camera'})
        restored = self.state()
        restored.restore(state.to_dict())
        self.assertEqual(restored.to_dict(), state.to_dict())

    def test_first_identity_upgrade_backs_up_old_database_and_keeps_v8_format(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)/'test.sqlite'
            old = self.state(unique=False)
            old.add_frame(1, 1., [detection()])
            payload = old.to_dict()
            self.assertNotIn('label', payload['objects'][0]['observations'][0])
            store = SemanticStore(path, '/tmp/map.db', {'model': 'test'})
            store.save(old)
            store.load(self.state())
            self.assertTrue(path.with_name('test.before_unique_objects.sqlite').is_file())
            store.close()


if __name__ == '__main__':
    unittest.main()
