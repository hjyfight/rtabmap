"""Accepted-event notification, hold time and sensor-loss behavior without DDS."""

import importlib.util
from pathlib import Path
import unittest

spec = importlib.util.spec_from_file_location(
    'loop_closure_panel', Path(__file__).resolve().parents[1] / 'scripts/loop_closure_panel.py')
panel = importlib.util.module_from_spec(spec)
spec.loader.exec_module(panel)


class LoopStatusTests(unittest.TestCase):
    def setUp(self):
        self.now = 100.0
        self.status = panel.LoopStatus(clock=lambda: self.now)

    def success(self, kind='GLOBAL_LOOP', ref=40, target=5):
        return {'type': kind, 'ref_id': ref, 'target_id': target, 'time': 1793187052.0}

    def detection(self, state='SEARCHING', **fields):
        return {'type': 'DETECTION_STATUS', 'state': state,
                'state_label': state, 'nodes': 20, **fields}

    def test_candidates_and_rejections_never_trigger_success(self):
        for state in ('CANDIDATE_UNCONFIRMED', 'CANDIDATE_REJECTED', 'CONFIRMED'):
            # Only a dedicated accepted event may trigger the notification.
            self.assertFalse(self.status.update(self.detection(state)))
            self.assertFalse(self.status.update({'type': 'LOOP_CANDIDATE', 'target_id': 5}))
        self.assertEqual(sum(self.status.counts.values()), 0)
        self.assertIsNone(self.status.last_success)

    def test_success_survives_next_detection_and_expires_after_eight_seconds(self):
        self.assertTrue(self.status.update(self.success()))
        self.now += 1
        self.status.update(self.detection('CANDIDATE_REJECTED'))
        self.assertEqual(self.status.display()[0], '回环成功')
        self.now += 7
        self.status.update(self.detection())
        self.assertEqual(self.status.display()[0], '回环检测中')
        self.assertIn('#40 ↔ #5', self.status.display()[2])

    def test_duplicates_do_not_beep_count_or_extend_highlight(self):
        event = self.success()
        self.status.update(event)
        until = self.status.highlight_until
        self.now += 2
        self.assertFalse(self.status.update(event))
        self.assertEqual(self.status.counts['GLOBAL_LOOP'], 1)
        self.assertEqual(self.status.highlight_until, until)

    def test_global_and_proximity_counts_are_distinct(self):
        self.status.update(self.success())
        self.status.update(self.success('PROXIMITY_ICP', target=6))
        self.assertEqual(self.status.counts, {'GLOBAL_LOOP': 1, 'PROXIMITY_ICP': 1})
        self.assertIn('空间邻近回环', self.status.display()[2])
        self.assertFalse(self.status.update(self.success(target=0)))

    def test_slam_timeout_overrides_success_but_keeps_last_accepted_pair(self):
        self.status.update(self.success())
        self.status.update({'type': 'DETECTION_HEARTBEAT', 'state': 'SLAM_TIMEOUT',
                            'state_label': '数据中断', 'slam_ok': False})
        self.assertEqual(self.status.display()[0], 'SLAM 数据已中断')
        self.assertIn('#40 ↔ #5', self.status.display()[2])

    def test_lost_updates_timeout_uses_receipt_clock(self):
        self.status.update(self.detection())
        self.now += 7.5
        self.assertEqual(self.status.display()[0], 'SLAM 数据已中断')
        self.status.update(self.detection())
        self.assertEqual(self.status.display()[0], '回环检测中')

    def test_stationary_rehearsal_is_explained(self):
        self.status.update(self.detection('COLLECTING_HISTORY', rehearsal_merged_id=1))
        self.assertIn('相邻帧合并', self.status.label)


if __name__ == '__main__':
    unittest.main()
