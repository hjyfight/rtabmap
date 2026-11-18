"""Portable receiver config and commands must not operate robot-side services."""
import importlib.util
from pathlib import Path
import tempfile
import unittest

import yaml

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('orin_profile', ROOT / 'scripts/orin_network_perception.py')
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


class OrinProfileTests(unittest.TestCase):
    def setUp(self):
        self.profile = module.load_profile(ROOT / 'config/orin_network_perception.yaml')

    def test_profile_routes_display_independently_of_robot_sensor_address(self):
        self.profile['network'].update(interface='lo', sensor_host='192.168.1.200', display_host='127.0.0.1')
        env = module.environment(self.profile)
        self.assertIn('127.0.0.1', env['CYCLONEDDS_URI'])
        self.assertNotIn('192.168.1.200', env['CYCLONEDDS_URI'])
        command = module.commands_for('receive', self.profile, Path('/tmp/run'))[0][1]
        self.assertIn('192.168.1.200', command)
        self.assertNotIn('ssh', command)

    def test_all_perception_workers_share_cuda_and_configured_clocks(self):
        self.profile['network'].update(sensor_clock_offset_s=4.0, audio_clock_offset_s=1.0)
        params = module.runtime_parameters(self.profile)
        self.assertEqual(params['fourmodal_features']['ros__parameters']['clock_offsets_s'], [4., 4., 4., 1.])
        self.assertEqual(params['yolo_segmentation']['ros__parameters']['fusion_mode'], 'shadow')
        for values in params.values():
            self.assertEqual(values['ros__parameters']['device'], 'cuda:0')
        commands = module.commands_for('perception', self.profile, Path('/tmp/run'))
        self.assertEqual([name for name, _ in commands], ['rgb', 'nir', 'fusion', 'features'])
        self.assertTrue(all('ros2_wired_env.sh' not in ' '.join(command) for _, command in commands))

    def test_missing_mounting_calibration_cannot_start_semantic_mapping(self):
        with self.assertRaisesRegex(ValueError, 'measured mounting extrinsics'):
            module.commands_for('semantic', self.profile, Path('/tmp/run'))
        self.profile['mapping']['camera_to_lidar'] = [0., 0., 0., 0., 0., 0.]
        command = module.commands_for('semantic', self.profile, Path('/tmp/run'))[0][1]
        self.assertTrue(any('start_orin_slam.sh' in v for v in command))

    def test_target_pipeline_can_run_without_duplicate_global_and_rgb_nir_models(self):
        commands=module.commands_for('target-perception',self.profile,Path('/tmp/run'))
        self.assertEqual([n for n,_ in commands],['rgb','targets'])
        self.assertTrue(any('run_object_multimodal_fusion.py' in v for v in commands[-1][1]))
        self.assertEqual([n for n,_ in module.commands_for('target-fusion',self.profile,Path('/tmp/run'))],['targets'])

    def test_audio_is_explicit_and_legacy_receiver_arguments_remain_available(self):
        self.assertNotIn('--audio', module.commands_for('receive', self.profile, Path('/tmp/run'))[0][1])
        self.profile['network']['audio_transport'] = True
        self.assertIn('--audio', module.commands_for('receive', self.profile, Path('/tmp/run'))[0][1])

    def test_invalid_peer_or_geometry_is_rejected_before_starting_processes(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'profile.yaml'
            for bad in ['1.2.3.999', '127.0.0.1; touch /tmp/unwanted']:
                self.profile['network']['sensor_host'] = bad
                path.write_text(yaml.safe_dump(self.profile))
                with self.assertRaises(ValueError):
                    module.load_profile(path)


if __name__ == '__main__':
    unittest.main()
