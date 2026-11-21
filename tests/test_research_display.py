import importlib.util
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'scripts'))
from fusion_research_display import REMAPS, display_commands


def test_3001_uses_all_four_3000_display_services_with_private_topics():
    commands = display_commands(ROOT)
    assert len(commands)==5
    assert any(name=='loops' and '--status-window' in command and 'false' in command for name,command in commands)
    for name, command in commands:
        for original, private in REMAPS.items():
            assert original+':='+private in command
        assert '/cloud_map:=/miti/research/display/cloud_map' not in command
    assert REMAPS['/robot_pose']=='/miti/research/display/robot_pose'


def test_research_layout_keeps_sensor_and_slam_topics_but_isolates_display():
    spec=importlib.util.spec_from_file_location('research_installer',ROOT/'scripts/install_fusion_research_dashboard.py')
    installer=importlib.util.module_from_spec(spec);spec.loader.exec_module(installer)
    value=installer.layout()
    encoded=json.dumps(value)
    assert len(value['widgets'])==9
    assert any(w['i']=='research-rgbnir' for w in value['widgets'])
    for original in REMAPS:
        assert '"'+original+'"' not in encoded
    assert '/cloud_map' in encoded and '/miti/research/display/loop_status' in encoded
    assert '/miti/research/display/grid_map' in encoded
    assert '/miti/research/display/aligned_depth/image_raw' in encoded
