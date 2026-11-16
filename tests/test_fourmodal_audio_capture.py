"""ALSA process recovery without hardware or ROS; never mix recording sessions."""
import importlib.util
from pathlib import Path
import subprocess
import sys
import unittest

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('audio_capture', ROOT / 'scripts/publish_fourmodal_audio.py')
capture = importlib.util.module_from_spec(spec)
spec.loader.exec_module(capture)


class AudioRecoveryTests(unittest.TestCase):
    def stream(self, scripts, timeout=.5):
        processes, reasons, delays = [], [], []
        scripts = iter(scripts)

        def spawn(command, **kwargs):
            script = next(scripts)
            if isinstance(script, Exception):
                raise script
            process = subprocess.Popen([sys.executable, '-c', script], **kwargs)
            processes.append(process)
            return process

        stream = capture.capture_payloads(['test-recorder'], 4, running=lambda: True,
            on_retry=reasons.append, wait=delays.append, process_factory=spawn, timeout=timeout)
        self.addCleanup(stream.close)
        return stream, processes, reasons, delays

    def test_eof_discards_incomplete_chunk_and_marks_new_session(self):
        stream, processes, reasons, delays = self.stream([
            "import os; os.write(1,b'AAAAxx'); os.write(2,b'device disconnected')",
            "import os; os.write(1,b'BBBBCCCC')"])
        self.assertIsNone(next(stream))
        self.assertEqual(next(stream), b'AAAA')
        self.assertIsNone(next(stream))
        self.assertEqual(next(stream), b'BBBB')
        self.assertEqual(next(stream), b'CCCC')
        self.assertIn('device disconnected', reasons[0])
        self.assertEqual(delays, [2.0])
        self.assertIsNotNone(processes[0].poll())

    def test_pipe_fragments_are_assembled_without_restart(self):
        stream, processes, reasons, delays = self.stream([
            "import os,time; os.write(1,b'AB'); time.sleep(.02); os.write(1,b'CD')"])
        self.assertIsNone(next(stream))
        self.assertEqual(next(stream), b'ABCD')
        self.assertEqual(reasons, [])
        self.assertEqual(delays, [])

    def test_stalled_recorder_is_reaped_before_reopening(self):
        stream, processes, reasons, _ = self.stream([
            'import time; time.sleep(10)', "import os; os.write(1,b'NEXT')"], timeout=.2)
        self.assertIsNone(next(stream))
        self.assertIsNone(next(stream))
        self.assertIn('timed out', reasons[0])
        self.assertIsNotNone(processes[0].poll())
        self.assertEqual(next(stream), b'NEXT')
        stream.close()
        self.assertIsNotNone(processes[1].poll())

    def test_open_failure_retries_without_ending_producer(self):
        stream, _, reasons, delays = self.stream([
            OSError('device unavailable'), "import os; os.write(1,b'LIVE')"])
        self.assertIsNone(next(stream))
        self.assertEqual(next(stream), b'LIVE')
        self.assertIn('device unavailable', reasons[0])
        self.assertEqual(delays, [2.0])


if __name__ == '__main__':
    unittest.main()
