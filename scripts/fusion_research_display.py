"""Reuse the 3000 display nodes with private output topics for 3001."""
import json

PREFIX = '/miti/research/display'
TOPICS = [
    '/miti/aligned_depth/image_raw', '/miti/aligned_depth/camera_info',
    '/miti/trajectory', '/miti/semantic_markers', '/miti/clean_map',
    '/miti/live_scan', '/miti/dynamic_cloud_status', '/miti/walls',
    '/miti/map_status', '/miti/grid_map', '/miti/grid_status', '/miti/loop_status', '/robot_pose',
]
REMAPS = {topic: PREFIX + topic.removeprefix('/miti') if topic.startswith('/miti/')
          else PREFIX + topic for topic in TOPICS}


def private_display_source(source):
    for original, private in REMAPS.items():
        source = source.replace(original, private)
    return source


def private_layout(value):
    return json.loads(private_display_source(json.dumps(value, ensure_ascii=False)))


def display_commands(root):
    remaps = [arg for topic, private in REMAPS.items() for arg in ('-r', topic + ':=' + private)]
    commands = [(name, ['/usr/bin/python3', '-u', str(root/'scripts'/script), '--ros-args',
                   *(['--params-file', str(root/'config'/config)] if config else []), *remaps])
            for name, script, config in [
                ('scene', 'research_scene_adapter.py', None),
                ('cloud', 'dynamic_cloud_node.py', 'dynamic_cloud.yaml'),
                ('geometry', 'map_geometry_node.py', 'map_geometry.yaml'),
                ('grid', 'grid_projection_node.py', 'grid_projection.yaml'),
            ]]
    commands.append(('loops',['/usr/bin/python3','-u',str(root/'scripts/loop_closure_diagnostics.py'),
        '--status-window','false','--depth-topic','/camera/depth_registered/image_rect',
        '--log-file',str(root/'logs/research/loops_events.jsonl'),'--ros-args',*remaps]))
    return commands
