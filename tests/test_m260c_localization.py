import importlib.util
import json
from pathlib import Path
import sys
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'scripts'))
from m260c_localization import DirectionState, wakeup_result


def wake(angle=115):
    return {'type': 'aiui_event', 'content': {'eventType': 4, 'info': json.dumps({
        'ivw': {'angle': angle, 'start_ms': 2900, 'end_ms': 3710, 'keyword': 'xiao3 wei1 xiao3 wei1'}})}}


class LocalizationTests(unittest.TestCase):
    def test_official_nested_wakeup_result_is_accepted(self):
        self.assertEqual(wakeup_result(wake())['angle_deg'], 115)

    def test_beam_commands_other_events_and_invalid_angles_are_rejected(self):
        for data in [{'type':'manual_wakeup','content':{'beam':1,'angle':60}},
                     {'type':'version','content':{'angle':30}},
                     {'type':'aiui_event','content':{'eventType':3,'info':{'angle':12}}},
                     *[wake(a) for a in [True, -1, 360, float('nan'), float('inf'), '115']]]:
            self.assertIsNone(wakeup_result(data))

    def test_duplicate_events_and_heartbeats_never_refresh_old_bearings(self):
        now = [100.]
        state = DirectionState(lambda: now[0])
        event = {'event':'frame','sid':1,'payload':wake()}
        state.receive(event)
        self.assertEqual(state.snapshot()['state'], 'tracking')
        now[0] += 6
        state.receive(event)
        data = state.snapshot()
        self.assertEqual(data['sequence'], 1)
        self.assertEqual(data['age_s'], 6)
        self.assertEqual(data['state'], 'stale')
        self.assertFalse(data['calibration_verified'])
        self.assertIsNone(data['distance_m'])
        self.assertIsNone(data['target_id'])

    def test_serial_errors_keep_the_last_angle_historical(self):
        state = DirectionState()
        state.receive({'event':'frame','sid':1,'payload':wake()})
        state.error('device disconnected')
        data = state.snapshot()
        self.assertEqual(data['state'], 'serial_error')
        self.assertEqual(data['angle_deg'], 115)


if __name__ == '__main__':
    unittest.main()
