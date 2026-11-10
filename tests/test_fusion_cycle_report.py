from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'scripts'))
from fusion_cycle_report import summarize, model_metrics


def cycle(i=1, offset=0):
    base = 1_000_000_000 + offset
    fields = dict(node_id=i, node_stamp=float(i), session_id=0, clock_id='boot')
    bridge = [dict(fields, source='keyframe_bridge', run_id='bridge', sequence=2*i-1,
                   event='queued', event_ns=base, received_ns=base),
              dict(fields, source='keyframe_bridge', run_id='bridge', sequence=2*i,
                   event='dispatched', event_ns=base+30_000_000, received_ns=base,
                   dispatch_ns=base+20_000_000, decoded_ns=base+28_000_000,
                   published_ns=base+30_000_000, replay=False)]
    stages = {k: base+v*1_000_000 for k,v in dict(worker_start_ns=50, prepared_ns=60,
        detected_ns=160, worker_end_ns=170, fusion_start_ns=180, fusion_end_ns=200,
        persist_end_ns=220, publish_start_ns=230, publish_end_ns=240).items()}
    return bridge+[dict(fields, source='semantic_mapper', run_id='mapper', sequence=2*i-1,
                        event='accepted', event_ns=base+40_000_000, received_ns=base+40_000_000),
                   dict(fields, source='semantic_mapper', run_id='mapper', sequence=2*i,
                        event='completed', event_ns=base+250_000_000, received_ns=base+40_000_000,
                        submitted_ns=base+45_000_000, stages=stages)]


class CycleReportTests(unittest.TestCase):
    def test_model_parts_are_separate_from_object_fusion_and_not_double_counted(self):
        rows = cycle()
        profile = dict(state='valid', device='cpu', groups_ms=dict(
            rgb_backbone=20., depth_backbone=21., feature_extraction=41., rgbd_fusion=4.,
            context=3., decoders_and_postprocessing=40., model_other=2., model_forward=90.))
        rows[-1]['stages']['model_timing'] = profile
        r = summarize(rows)
        self.assertEqual(r['metrics_ms']['emsanet_feature_extraction']['mean_ms'], 41.)
        self.assertEqual(r['metrics_ms']['emsanet_feature_fusion']['mean_ms'], 4.)
        self.assertEqual(r['metrics_ms']['object_fusion']['mean_ms'], 20.)
        self.assertEqual(r['model_timing_coverage']['valid'], 1)
        profile['groups_ms']['feature_extraction'] = 100.
        with self.assertRaises(ValueError):
            model_metrics(profile)
        r = summarize(rows)
        self.assertEqual(r['model_timing_coverage']['invalid_or_unsupported'], 1)
        self.assertNotIn('emsanet_feature_fusion', r['metrics_ms'])

    def test_full_cycle_includes_waiting_and_only_matching_frame(self):
        r = summarize(list(reversed(cycle())), period_ms=240, minimum_samples=1)
        self.assertEqual(r['metrics_ms']['bridge_to_semantic']['max_ms'], 240)
        self.assertEqual(r['metrics_ms']['mapper_cycle']['max_ms'], 200)
        self.assertEqual(r['metrics_ms']['network_and_masks']['max_ms'], 100)
        self.assertEqual(r['evaluation'], 'within_limit_for_measured_scope')
        self.assertFalse(r['whole_system_compliance_proven'])

    def test_maximum_not_mean_decides_strict_limit(self):
        r = summarize(cycle(), period_ms=239, minimum_samples=1)
        self.assertEqual(r['evaluation'], 'exceeded')
        self.assertEqual(r['threshold_exceedances'], 1)

    def test_no_limit_no_samples_and_insufficient_samples_never_pass(self):
        self.assertEqual(summarize(cycle())['evaluation'], 'not_configured')
        self.assertEqual(summarize([], 100)['evaluation'], 'insufficient_data')
        self.assertEqual(summarize(cycle(), 300)['evaluation'], 'inconclusive')

    def test_different_clocks_or_frame_ids_do_not_match(self):
        for field, value in [('clock_id','other'), ('node_stamp',2.), ('session_id',3)]:
            rows = cycle()
            rows[-1][field] = value
            r = summarize(rows, 300, minimum_samples=1)
            self.assertEqual(r['metrics_ms']['bridge_to_semantic']['count'], 0)
            self.assertEqual(r['evaluation'], 'insufficient_data')

    def test_transport_duplicates_are_not_double_counted(self):
        rows = cycle()
        r = summarize(rows+rows)
        self.assertEqual(r['metrics_ms']['bridge_to_semantic']['count'], 1)
        self.assertEqual(r['quality']['duplicate_events'], 4)

    def test_bad_boundary_and_unfinished_work_are_visible(self):
        rows = cycle()
        rows[-1]['stages']['detected_ns'] = 10
        r = summarize(rows, 300, minimum_samples=1)
        self.assertEqual(r['quality']['invalid_records'], 1)
        self.assertNotEqual(r['evaluation'], 'within_limit_for_measured_scope')
        r = summarize(cycle()+cycle(2, 1_000_000_000)[:-1], 300, minimum_samples=1)
        self.assertEqual(r['quality']['mapper_unfinished_at_end'], 1)
        self.assertEqual(r['evaluation'], 'inconclusive')

    def test_warmup_and_output_interval_are_separate_from_cycle(self):
        r = summarize(cycle()+cycle(2, 1_000_000_000)+cycle(3, 2_000_000_000), warmup=1)
        self.assertEqual(r['metrics_ms']['mapper_cycle']['count'], 2)
        self.assertEqual(r['metrics_ms']['result_interval']['max_ms'], 1000)
        self.assertEqual(len(r['first_cycles']), 1)

    def test_missing_event_and_failure_prevent_pass(self):
        rows = cycle()+cycle(2, 1_000_000_000)
        rows[-1]['event'] = 'failed'
        r = summarize(rows, 300, minimum_samples=1)
        self.assertEqual(r['quality']['failures_or_drops'], 1)
        self.assertEqual(r['evaluation'], 'inconclusive')
        r = summarize(cycle()+cycle(3, 2_000_000_000), 300, minimum_samples=1)
        self.assertEqual(r['quality']['sequence_gaps'], 4)
        self.assertEqual(r['evaluation'], 'inconclusive')

    def test_native_stages_never_replace_full_cycle(self):
        r = summarize([dict(source='native', run_id='probe', sequence=1, event='sample',
                           metrics_ms={'icp_estimation':20, 'slam_total':40})], 100, minimum_samples=1)
        self.assertEqual(r['metrics_ms']['slam_total']['max_ms'], 40)
        self.assertEqual(r['evaluation'], 'insufficient_data')


if __name__ == '__main__':
    unittest.main()
