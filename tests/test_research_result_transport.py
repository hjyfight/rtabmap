import importlib.util
import json
from pathlib import Path
import tempfile
import threading
import unittest
import urllib.error
import urllib.request
from http.server import ThreadingHTTPServer

MODULE = Path(__file__).resolve().parents[1] / 'scripts/research_result_transport.py'
spec = importlib.util.spec_from_file_location('research_transport', MODULE)
transport = importlib.util.module_from_spec(spec)
spec.loader.exec_module(transport)


def snapshot(sequence=1):
    return dict(schema='downstream_fusion/1', session_id='test-session', task='pmf', published_at=1000.,
                state='ready', detail='', inputs={'rgb': {'stamp': 998.}}, ros_error='',
                model={'sha256': 'verified-test-checkpoint'}, result={'sequence': sequence, 'source_stamp': 997.,
                'feature_download': '/research/pmf_features.npz', 'images': {'raw': 'test-image'}})


class ResultTransportTest(unittest.TestCase):
    def test_status_refresh_does_not_refetch_large_result(self):
        value = snapshot()
        first = transport.result_id(value)
        value.update(published_at=1001., state='running', inputs={'rgb': {'stamp': 1000.}})
        self.assertEqual(first, transport.result_id(value))
        value['result']['sequence'] = 2
        self.assertNotEqual(first, transport.result_id(value))

    def test_clock_conversion_preserves_acquisition_and_producer_times(self):
        value = snapshot()
        displayed = transport.display_snapshot(value, 55., 'http://192.168.1.103:7448')
        self.assertEqual(displayed['published_at'], 1000.)
        self.assertEqual(displayed['display_published_at'], 1055.)
        self.assertEqual(displayed['inputs']['rgb']['stamp'], 998.)
        self.assertEqual(displayed['inputs']['rgb']['display_stamp'], 1053.)
        self.assertEqual(displayed['result']['source_stamp'], 997.)
        self.assertEqual(displayed['result']['display_source_stamp'], 1052.)
        self.assertEqual(displayed['result']['feature_download'], 'http://192.168.1.103:7448/pmf_features.npz')
        self.assertNotIn('display_stamp', value['inputs']['rgb'])

    def test_http_manifest_is_small_and_only_named_artifacts_are_served(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            transport.atomic_json(root / 'pmf.json', snapshot())
            (root / 'private.txt').write_text('must not be served')
            (root / 'mapping').mkdir()
            transport.atomic_json(root / 'mapping/status.json', {'state': 'receiving'})
            (root / 'process').mkdir()
            transport.atomic_json(root / 'process/rgbnir.json', {'task': 'rgbnir'})
            service = ThreadingHTTPServer(('127.0.0.1', 0), transport.handler_for(root))
            worker = threading.Thread(target=service.serve_forever, daemon=True)
            worker.start()
            url = 'http://127.0.0.1:' + str(service.server_port)
            client = urllib.request.build_opener(urllib.request.ProxyHandler({}))
            try:
                with client.open(url + '/manifest.json') as response:
                    value = json.load(response)
                self.assertEqual(value['schema'], 'orin_research_manifest/1')
                self.assertNotIn('result', value['tasks']['pmf']['status'])
                self.assertEqual(value['tasks']['pmf']['result_id'], transport.result_id(snapshot()))
                with client.open(url + '/pmf.json') as response:
                    self.assertEqual(json.load(response)['result']['sequence'], 1)
                with client.open(url + '/mapping/status.json') as response:
                    self.assertEqual(json.load(response)['state'], 'receiving')
                with client.open(url + '/process/rgbnir.json') as response:
                    self.assertEqual(json.load(response)['task'], 'rgbnir')
                with self.assertRaises(urllib.error.HTTPError) as nested:
                    client.open(url + '/mapping/../private.txt')
                self.assertEqual(nested.exception.code, 404)
                with self.assertRaises(urllib.error.HTTPError) as error:
                    client.open(url + '/private.txt')
                self.assertEqual(error.exception.code, 404)
            finally:
                service.shutdown()
                service.server_close()
                worker.join(timeout=2)


if __name__ == '__main__':
    unittest.main()
