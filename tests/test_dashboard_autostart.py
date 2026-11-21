"""Exercise YOLOE launcher ownership with isolated fake drivers and no map writes."""
import os
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]


class DashboardAutostartTests(unittest.TestCase):
    def launch(self, check_only):
        with tempfile.TemporaryDirectory(prefix='dashboard_launch_') as temporary:
            project = Path(temporary)
            for directory in ('scripts', 'logs', 'install/semantic', '.venv-yoloe/bin',
                              'models', 'bin'):
                (project / directory).mkdir(parents=True, exist_ok=True)
            for name in ('start_semantic_yoloe.sh', 'start_semantic_slam.sh'):
                shutil.copy(ROOT / 'scripts' / name, project / 'scripts' / name)
            (project / 'setup.bash').write_text('true\n')
            for name in ('setup.bash', 'local_setup.bash'):
                (project / 'install/semantic' / name).write_text('true\n')
            (project / 'models/yoloe-11s-seg-pf.pt').write_text('fake model, never loaded\n')
            runtime = project / '.venv-yoloe/bin/python3'
            runtime.write_text('#!/bin/sh\nexit 0\n'); runtime.chmod(0o755)
            (project / 'scripts/check_registered_depth.py').write_text("print('reuse')\n")
            (project / 'scripts/check_rgbd_input.py').write_text('pass\n')
            (project / 'scripts/verify_fusion_input.py').write_text('pass\n')
            (project / 'scripts/start_dashboard_perception.py').write_text('''
import os, signal, time
from pathlib import Path
root=Path(__file__).resolve().parents[1]
def stop(*_):
    (root/'stopped').write_text(str(os.getpid()))
    raise SystemExit(0)
signal.signal(signal.SIGTERM,stop)
(root/'started').write_text(str(os.getpid()))
while True: time.sleep(.05)
''')
            ros2 = project / 'bin/ros2'
            ros2.write_text('''#!/usr/bin/python3
import time
from pathlib import Path
root=Path(__file__).resolve().parents[1]
end=time.monotonic()+5
while not (root/'started').exists() and time.monotonic()<end: time.sleep(.02)
assert (root/'started').exists(), 'automatic perception was not started'
(root/'mapped').write_text('mapping stub ran')
''')
            ros2.chmod(0o755)
            env = dict(os.environ, PATH=str(project / 'bin') + ':' + os.environ['PATH'],
                       ROS_DOMAIN_ID='0', SEMANTIC_DASHBOARD_PERCEPTION='1')
            command = ['bash', str(project / 'scripts/start_semantic_yoloe.sh')]
            if check_only:
                command.append('--check-input-only')
            result = subprocess.run(command, env=env, capture_output=True, text=True, timeout=10)
            self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
            if check_only:
                self.assertFalse((project / 'started').exists())
                self.assertFalse((project / 'mapped').exists())
            else:
                self.assertTrue((project / 'mapped').exists())
                self.assertEqual((project / 'started').read_text(), (project / 'stopped').read_text())

    def test_normal_mapping_starts_and_cleans_its_perception_child(self):
        self.launch(False)

    def test_input_check_never_starts_models_or_mapping(self):
        self.launch(True)


if __name__ == '__main__':
    unittest.main()
