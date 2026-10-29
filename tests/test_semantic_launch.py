"""Exercise nested launch actions without starting ROS processes or modifying maps."""
import os
from pathlib import Path
import unittest

from launch import LaunchContext, LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, LogInfo,
    OpaqueFunction, SetLaunchConfiguration,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.utilities import perform_substitutions
from launch_ros.actions import Node
from launch_ros.utilities import evaluate_parameters


ROOT = Path(__file__).resolve().parents[1]
os.environ.setdefault('ROS_LOG_DIR', '/tmp/semantic_launch_test_logs')


def resolve_launch(*, inspect_action=None, **overrides):
    context = LaunchContext()
    nodes = {}

    def visit(action):
        condition = getattr(action, 'condition', None)
        if condition and not condition.evaluate(context):
            return
        if inspect_action:
            inspect_action(action, context)
        if isinstance(action, Node):
            parameters = {}
            for entry in evaluate_parameters(context, action._Node__parameters or []):
                if isinstance(entry, dict):
                    parameters.update(entry)
            nodes[str(action.node_executable)] = (
                parameters, perform_substitutions(context, action.process_description.prefix))
        elif isinstance(action, LaunchDescription):
            for child in action.entities:
                visit(child)
        elif isinstance(action, (IncludeLaunchDescription, DeclareLaunchArgument,
                                 SetLaunchConfiguration, OpaqueFunction)):
            # execute(), unlike --show-args, checks required include arguments.
            for child in action.execute(context) or []:
                visit(child)
        elif not isinstance(action, (ExecuteProcess, LogInfo)):
            raise AssertionError(f'Unhandled launch action: {type(action).__name__}')

    visit(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(ROOT / 'launch/semantic_slam.launch.py')),
        launch_arguments=overrides.items()))
    return nodes


class SemanticLaunchTests(unittest.TestCase):
    def test_only_validated_rgbd_reaches_backend(self):
        remaps, commands = {}, []

        def inspect(action, context):
            if isinstance(action, Node):
                remaps[str(action.node_executable)] = dict(
                    (perform_substitutions(context, source), perform_substitutions(context, target))
                    for source, target in (action._Node__remappings or []))
            elif isinstance(action, ExecuteProcess):
                commands.append([perform_substitutions(context, part) for part in action.cmd])

        resolve_launch(inspect_action=inspect)
        self.assertEqual(remaps['rgbd_sync']['rgbd_image'], 'rgbd_image_unchecked')
        self.assertEqual(remaps['rtabmap']['rgbd_image'], 'rgbd_image')
        guards = [cmd for cmd in commands if any(x.endswith('/guard_rgbd_input.py') for x in cmd)]
        self.assertEqual(len(guards), 1)
        self.assertIn('input:=rgbd_image_unchecked', guards[0])
        self.assertIn('output:=rgbd_image', guards[0])

    def assert_paired_map(self, nodes, database_path):
        self.assertIn('rgbd_sync', nodes)
        self.assertIn('icp_odometry', nodes)
        self.assertIn('keyframe_bridge', nodes)
        self.assertEqual(nodes['rtabmap'][0]['database_path'], database_path)
        self.assertEqual(nodes['semantic_mapper'][0]['database_path'], database_path)
        self.assertEqual(nodes['rtabmap_viz'][0]['semantic_map_key'],
                         str(Path(database_path).expanduser().resolve()))
        self.assertTrue(nodes['rtabmap_viz'][0]['semantic_enabled'])

    def test_default_launch_passes_required_arguments(self):
        nodes = resolve_launch()
        mapper, prefix = nodes['semantic_mapper']
        database = Path(mapper['database_path'])
        self.assertEqual(database.parent, ROOT / 'maps')
        self.assertTrue(database.name.startswith('semantic_'))
        self.assertEqual(database.suffix, '.db')
        self.assert_paired_map(nodes, str(database))
        self.assertEqual(mapper['model_path'], str(ROOT / 'models/yolov8n-seg.pt'))
        self.assertEqual(prefix, str(ROOT / '.venv-semantic/bin/python3'))

    def test_overrides_reach_slam_mapper_and_gui(self):
        database = '/tmp/semantic launch test/map.db'
        model = '/tmp/semantic launch test/custom.pt'
        python = '/tmp/custom-venv/bin/python3'
        nodes = resolve_launch(database_path=database, model_path=model,
                               semantic_python=python, semantic_device='cuda:0',
                               semantic_database_path='/tmp/custom.semantic.sqlite',
                               semantic_replay_existing='false')
        self.assert_paired_map(nodes, database)
        mapper, prefix = nodes['semantic_mapper']
        self.assertEqual(mapper['model_path'], model)
        self.assertEqual(prefix, python)
        self.assertEqual(mapper['device'], 'cuda:0')
        self.assertEqual(mapper['semantic_database_path'], '/tmp/custom.semantic.sqlite')
        self.assertFalse(nodes['keyframe_bridge'][0]['replay_existing'])

    def test_localization_uses_the_same_saved_map(self):
        database = '/tmp/semantic launch test/saved.db'
        nodes = resolve_launch(database_path=database, localization='true')
        self.assert_paired_map(nodes, database)
        self.assertEqual(nodes['rtabmap_viz'][0]['init_cache_path'], database)
        self.assertEqual(nodes['rtabmap'][0]['Mem/IncrementalMemory'], 'False')


if __name__ == '__main__':
    unittest.main()
