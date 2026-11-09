"""Verify network clock estimation without changing the workstation clock."""
from pathlib import Path
import socket
import sys
import threading
import time
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from sync_hotspot_clock import serve, sync


class ClockSyncTests(unittest.TestCase):
    def test_udp_roundtrip_estimates_same_host_time_without_clock_step(self):
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as reservation:
            reservation.bind(('127.0.0.1', 0))
            port = reservation.getsockname()[1]
        worker = threading.Thread(target=serve, args=('127.0.0.1', port, 1), daemon=True)
        worker.start()
        time.sleep(0.05)
        with patch('time.clock_settime') as setter:
            sync('127.0.0.1', port)
            setter.assert_called_once()
            clock, instant = setter.call_args.args
            self.assertEqual(clock, time.CLOCK_REALTIME)
            self.assertLess(abs(instant - time.time()), 0.1)
        worker.join(timeout=2)


if __name__ == '__main__':
    unittest.main()
