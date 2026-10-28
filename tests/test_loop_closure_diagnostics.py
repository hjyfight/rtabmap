"""Verify diagnostic behavior with real Info messages and no DDS dependency."""

import importlib.util
import json
import math
from pathlib import Path
import tempfile
import time
import unittest
from unittest.mock import Mock, patch

from rclpy.node import Node
from rtabmap_msgs.msg import Info


ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location(
    'loop_diagnostics', ROOT / 'scripts/loop_closure_diagnostics.py')
diagnostics = importlib.util.module_from_spec(spec)
spec.loader.exec_module(diagnostics)


def make_info(ref=40, global_id=0, proximity_id=0, **overrides):
    msg = Info(ref_id=ref, loop_closure_id=global_id, proximity_detection_id=proximity_id)
    msg.header.stamp.sec = ref
    msg.wm_state = list(range(1, 11))
    stats = {
        'Loop/Highest_hypothesis_id/': 0,
        'Loop/Highest_hypothesis_value/': 0,
        'Loop/RejectedHypothesis/': 0,
        'Memory/Working_memory_size/': 10,
        'Memory/Local_graph_size/': 20,
        'Keypoint/Current_frame/': 200,
        **overrides,
    }
    msg.stats_keys = list(stats)
    msg.stats_values = [float(value) for value in stats.values()]
    return msg


class DiagnosticsTests(unittest.TestCase):
    def setUp(self):
        temporary = tempfile.TemporaryDirectory(prefix='loop_diagnostics_unit_')
        self.addCleanup(temporary.cleanup)
        self.log_file = Path(temporary.name) / 'nested' / 'events.jsonl'
        self.logger = Mock()
        for method, kwargs in (
                ('__init__', {'return_value': None}),
                ('create_subscription', {}), ('create_timer', {}),
                ('get_logger', {'return_value': self.logger})):
            patcher = patch.object(Node, method, **kwargs)
            patcher.start()
            self.addCleanup(patcher.stop)
        self.node = diagnostics.LoopClosureDiagnostics(log_file=str(self.log_file))

    def events(self, event_type=None):
        events = [json.loads(line) for line in self.log_file.read_text().splitlines()]
        return events if event_type is None else [event for event in events if event['type'] == event_type]

    def output(self):
        return '\n'.join(call.args[0] for call in self.logger.info.call_args_list)

    def test_search_and_history_are_visible_before_a_success(self):
        self.node._info_cb(make_info(**{'Memory/Working_memory_size/': 0}))
        self.node._info_cb(make_info(ref=41))
        self.assertEqual([event['state'] for event in self.events('DETECTION_STATUS')],
                         ['COLLECTING_HISTORY', 'SEARCHING'])
        self.assertEqual(self.node.loop_count, 0)
        self.assertIn('[回环检测 #2]', self.output())
        self.assertNotIn('[回环成功', self.output())

    def test_low_score_candidate_is_not_reported_as_a_loop(self):
        self.node._info_cb(make_info(**{
            'Loop/Highest_hypothesis_id/': 5,
            'Loop/Highest_hypothesis_value/': 0.03}))
        event = self.events('LOOP_CANDIDATE')[0]
        self.assertEqual(event['state'], 'CANDIDATE_UNCONFIRMED')
        self.assertEqual(event['candidate_id'], 5)
        self.assertAlmostEqual(event['candidate_score'], 0.03, places=5)
        self.assertIn('分数:0.0300', self.output())
        self.assertEqual(self.node.loop_count, 0)

    def test_real_unit_keys_and_search_during_small_movement(self):
        self.node._info_cb(make_info(**{
            'Keypoint/Current_frame/words': 123,
            'Timing/Likelihood_computation/ms': 2,
            'Timing/Posterior_computation/ms': 1,
            'Timing/Hypotheses_creation/ms': 0.5,
            'Memory/Small_movement/': 1}))
        event = self.events('DETECTION_STATUS')[0]
        self.assertEqual(event['visual_words'], 123)
        self.assertTrue(event['search_performed'])
        self.assertEqual(event['search_time_ms'], 3.5)
        self.assertEqual(event['state'], 'SEARCHING')

    def test_reported_zero_timings_do_not_claim_a_search(self):
        self.node._info_cb(make_info(**{
            'Timing/Likelihood_computation/ms': 0,
            'Timing/Posterior_computation/ms': 0,
            'Timing/Hypotheses_creation/ms': 0}))
        event = self.events('DETECTION_STATUS')[0]
        self.assertFalse(event['search_performed'])
        self.assertEqual(event['state'], 'SEARCH_SKIPPED')

    def test_empty_working_memory_is_explicit_even_for_small_movement(self):
        self.node._info_cb(make_info(**{
            'Memory/Working_memory_size/': 0,
            'Memory/Small_movement/': 1}))
        self.assertEqual(self.events('DETECTION_STATUS')[0]['state'], 'COLLECTING_HISTORY')

    def test_rejected_candidate_has_a_result_without_a_success(self):
        self.node._info_cb(make_info(**{
            'Loop/Highest_hypothesis_id/': 5,
            'Loop/Highest_hypothesis_value/': 0.8,
            'Loop/RejectedHypothesis/': 1,
            'Loop/Visual_matches/': 40,
            'Loop/Visual_inliers/': 6}))
        event = self.events('LOOP_CANDIDATE')[0]
        self.assertEqual(event['state'], 'CANDIDATE_REJECTED')
        self.assertEqual(event['visual_inliers'], 6)
        self.assertIn('候选被后端拒绝', self.output())
        self.assertNotIn('[回环成功', self.output())

    def test_success_preserves_optimization_stats_and_is_deduplicated(self):
        msg = make_info(global_id=5, **{
            'Loop/Highest_hypothesis_id/': 5,
            'Loop/Highest_hypothesis_value/': 0.9,
            'Loop/Optimization_error/': 0.25,
            'Loop/Optimization_iterations/': 8})
        self.node._info_cb(msg)
        self.node._info_cb(msg)
        loops = self.events('GLOBAL_LOOP')
        self.assertEqual(len(loops), 1)
        self.assertEqual(loops[0]['target_id'], 5)
        self.assertEqual(loops[0]['optimization_error'], 0.25)
        self.assertEqual(loops[0]['optimization_iterations'], 8)
        self.assertEqual(self.node.detection_count, 1)
        self.assertEqual(self.node.loop_count, 1)
        self.assertIn('[回环成功 #1]', self.output())

    def test_delayed_duplicate_and_both_success_types_are_counted_once(self):
        msg = make_info(global_id=5, proximity_id=6)
        self.node._info_cb(msg)
        self.node._info_cb(make_info(ref=41))
        self.node._info_cb(msg)
        self.assertEqual(self.node.detection_count, 2)
        self.assertEqual(self.node.loop_count, 2)
        self.assertEqual(len(self.events('GLOBAL_LOOP')), 1)
        self.assertEqual(len(self.events('PROXIMITY_ICP')), 1)

    def test_proximity_success_does_not_accept_a_rejected_global_candidate(self):
        self.node._info_cb(make_info(proximity_id=6, **{
            'Loop/Highest_hypothesis_id/': 5,
            'Loop/RejectedHypothesis/': 1}))
        self.assertEqual(self.events('DETECTION_STATUS')[0]['state'], 'CONFIRMED')
        self.assertEqual(self.events('LOOP_CANDIDATE')[0]['state'], 'CANDIDATE_REJECTED')
        self.assertEqual(len(self.events('PROXIMITY_ICP')), 1)
        self.assertEqual(len(self.events('GLOBAL_LOOP')), 0)

    def test_movement_flags_do_not_turn_a_retained_candidate_into_a_new_loop(self):
        for ref, flag, expected in (
                (40, 'Memory/Small_movement/', 'SMALL_MOVEMENT'),
                (41, 'Memory/Fast_movement/', 'FAST_MOVEMENT')):
            self.node._info_cb(make_info(ref=ref, **{
                'Loop/Highest_hypothesis_id/': 5, flag: 1}))
            self.assertEqual(self.events('LOOP_CANDIDATE')[-1]['state'], expected)
        self.assertEqual(self.node.loop_count, 0)
        self.assertIn('可能沿用上一帧', self.output())

    def test_missing_features_and_missing_stats_are_explicit(self):
        self.node._info_cb(make_info(**{'Keypoint/Current_frame/': 0}))
        self.node._info_cb(Info(ref_id=41))
        self.assertEqual([event['state'] for event in self.events('DETECTION_STATUS')],
                         ['NO_VISUAL_FEATURES', 'STATS_UNAVAILABLE'])

    def test_nonfinite_stats_become_null_and_never_use_odometry_metrics(self):
        self.node._info_cb(make_info(global_id=5, **{
            'Loop/Highest_hypothesis_value/': math.nan,
            'Loop/Optimization_error/': math.inf,
            'Odometry/ICPInliersRatio/': 0.9}))
        event = self.events('GLOBAL_LOOP')[0]
        self.assertIsNone(event['candidate_score'])
        self.assertIsNone(event['optimization_error'])
        self.assertIsNone(event['icp_inliers_ratio'])
        self.assertNotIn('NaN', self.log_file.read_text())
        self.assertNotIn('Infinity', self.log_file.read_text())
        self.assertIn('重合率:N/A', self.output())

    def test_heartbeat_distinguishes_waiting_and_stale_data(self):
        self.node.last_report_time = time.monotonic() - 5
        self.node._report()
        event = self.events('DETECTION_HEARTBEAT')[-1]
        self.assertEqual(event['state'], 'WAITING_FOR_SLAM')
        self.assertFalse(event['slam_ok'])
        self.node._info_cb(make_info())
        self.node.last_info_time = time.monotonic() - 8
        last_info_time = self.node.last_info_time
        self.node._info_cb(make_info())  # A replayed frame cannot refresh SLAM health.
        self.assertEqual(self.node.last_info_time, last_info_time)
        self.node._report()
        event = self.events('DETECTION_HEARTBEAT')[-1]
        self.assertEqual(event['state'], 'SLAM_TIMEOUT')
        self.assertFalse(event['slam_ok'])


if __name__ == '__main__':
    unittest.main()
