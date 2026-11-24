import json
from pathlib import Path
import sys
import threading
from types import SimpleNamespace

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'scripts'))
from research_validation import input_record, interface_audit, sync_audit, verification_revision
from export_research_process import build_sample, timing


def make_archive(tmp_path, *, change=None):
    inputs = [input_record('rgb', '/rgb', 'sensor_msgs/msg/Image', 10., 50.,
                frame='camera', shape=[2,3,3], dtype='uint8', format='rgb8', array_ref={'key':'input_rgb'}),
              input_record('nir', '/nir', 'sensor_msgs/msg/Image', 10.02, 50.,
                frame='infrared', shape=[2,3], dtype='uint8', format='mono8', array_ref={'key':'source_nir'})]
    context = dict(inputs_used=inputs, rgb_stamp=10., nir_stamp=10.02, sync_gap_s=.02,
        clock_offsets_s=dict(rgb=50.,nir=50.,lidar=50.,audio=0.),
        runtime_pair_tolerance_ms=50., output_contract={'output_shape':[1,6], 'classes':[]})
    feature = dict(name='P3.rgb', shape=[1,4,2,3], saved_shape=[1,4,2,3], operation='backbone')
    meta = dict(session='session', sequence=2, task='rgbnir', source_stamp=60., context=context,
                model={'sha256':'modelhash'}, features=[feature])
    if change: change(meta)
    path = tmp_path/'sample.npz'
    np.savez(path, metadata=np.array(json.dumps(meta)), input_rgb=np.zeros((2,3,3),np.uint8),
        source_nir=np.zeros((2,3),np.uint8), P3_rgb=np.zeros((1,4,2,3),np.float32))
    snapshot = dict(task='rgbnir', session_id='session', inputs={'rgb':{'raw_stamp':999999.,'topic':'/newer'}},
        model={'sha256':'modelhash'}, result={'sequence':2,'source_stamp':60.,'features':[feature],
        'output_shape':[1,6], 'classes':[]})
    return path, snapshot


def policy(**values):
    result = dict(sync_limit_ms=50., pair_check_limit_ms=50.,
                  reference='test', measurement_file='measurements.json')
    result.update(values)
    return result


def test_selected_archive_fields_are_used_instead_of_latest_snapshot(tmp_path):
    path, snapshot = make_archive(tmp_path)
    with np.load(path, allow_pickle=False) as archive:
        sample = build_sample('rgbnir', snapshot, archive)
    assert sample['interface_example']['rgb']['raw_stamp'] == 10.
    assert sample['interface_example']['rgb']['topic'] == '/rgb'
    assert sample['interface_validation']['state'] == 'pass'
    assert sample['evaluation']['verified'] is False


def test_old_archive_never_borrows_new_input_metadata(tmp_path):
    path, snapshot = make_archive(tmp_path, change=lambda m:m['context'].pop('inputs_used'))
    with np.load(path, allow_pickle=False) as archive:
        sample = build_sample('rgbnir', snapshot, archive)
    assert sample['interface_example'] == {}
    assert sample['interface_validation']['state'] == 'missing'


@pytest.mark.parametrize('change', [
    lambda m:m['context']['inputs_used'][0].update(aligned_stamp=100.),
    lambda m:m['context']['inputs_used'][0].update(dtype='float32'),
    lambda m:m['context']['inputs_used'][0].update(shape=[9,9,3]),
    lambda m:m['features'][0].update(saved_shape=[1,4,3,2]),
    lambda m:m['model'].update(sha256='different_model'),
    lambda m:m['context']['output_contract'].update(output_shape=[8,8]),
])
def test_inconsistent_fields_and_outputs_cannot_pass(tmp_path, change):
    path, snapshot = make_archive(tmp_path, change=change)
    with np.load(path, allow_pickle=False) as archive:
        meta = json.loads(str(archive['metadata']))
        result = interface_audit('rgbnir', snapshot, meta, archive)
    assert result['state'] == 'fail'


def test_missing_source_identifier_is_reported(tmp_path):
    path, snapshot = make_archive(tmp_path, change=lambda m:m['context']['inputs_used'][0].pop('source_id'))
    with np.load(path, allow_pickle=False) as archive:
        meta = json.loads(str(archive['metadata']))
        result = interface_audit('rgbnir', snapshot, meta, archive)
    assert result['state'] == 'missing'
    assert any('source_id' in c['detail'] for c in result['checks'])


def test_pair_statistics_do_not_prove_sync_accuracy(tmp_path):
    meta = dict(task='avs', session='s', sequence=1, context=dict(
        clock_alignment=dict(method='receive_time_estimate',hardware_synchronization_validated=True)))
    result = sync_audit('avs', meta, {'rows':[{'a':1.,'b':1.01},{'a':2.,'b':2.09}]}, policy(), tmp_path)
    assert result['pair_state'] == 'fail'
    assert result['statistics']['maximum_ms'] == pytest.approx(90.)
    assert result['statistics']['over_limit_count'] == 1
    assert result['measurement_state'] == result['state'] == 'pending'


def write_measurement(tmp_path, **changes):
    record = dict(task='rgbnir', session_id='s',sequence=1,max_error_ms=20.,uncertainty_ms=2.,
                  method='common acquisition event',evidence='measurement-evidence',covers_acquisition_time=True)
    record.update(changes)
    (tmp_path/'measurements.json').write_text(json.dumps(dict(schema='research_sync_measurements/1',records=[record])))
    return dict(task='rgbnir',session='s',sequence=1,context={})


def test_sync_measurement_must_match_batch_and_cover_acquisition(tmp_path):
    meta = write_measurement(tmp_path, session_id='other-session')
    assert sync_audit('rgbnir',meta,{'rows':[]},policy(),tmp_path)['measurement_state']=='pending'
    meta = write_measurement(tmp_path, covers_acquisition_time=False)
    result=sync_audit('rgbnir',meta,{'rows':[]},policy(),tmp_path)
    assert result['measurement_state']=='pending'
    assert result['measurement_error']


def test_measured_error_plus_uncertainty_is_checked_against_limit(tmp_path):
    meta = write_measurement(tmp_path)
    assert sync_audit('rgbnir',meta,{'rows':[]},policy(),tmp_path)['state']=='pass'
    meta = write_measurement(tmp_path,max_error_ms=49.,uncertainty_ms=2.)
    assert sync_audit('rgbnir',meta,{'rows':[]},policy(),tmp_path)['state']=='fail'


def test_sync_uses_configured_limit_without_a_sampling_period_gate(tmp_path):
    meta = write_measurement(tmp_path,max_error_ms=11.,uncertainty_ms=0.)
    result=sync_audit('rgbnir',meta,{'rows':[]},policy(),tmp_path)
    assert result['effective_sync_limit_ms']==50.
    assert result['state']=='pass'
    assert not any(k.startswith('sampling_') for k in result)


def test_verification_revision_changes_when_measurement_changes(tmp_path):
    (tmp_path/'config').mkdir()
    config=policy();config['schema']='research_verification/1'
    (tmp_path/'config/research_verification.json').write_text(json.dumps(config))
    before=verification_revision(tmp_path)
    write_measurement(tmp_path)
    assert verification_revision(tmp_path)!=before


def test_audio_provenance_records_actual_contributors_and_original_channels(monkeypatch):
    import run_fusion_research as runtime
    from rtabmap_research.inputs import AudioTimeline
    research=runtime.Research.__new__(runtime.Research)
    research.lock=threading.RLock()
    research.cfg=dict(audio_clock_offset_s=0.,audio_topic='/audio',avs_sync_tolerance_s=.25,max_input_age_s=3.)
    research.inputs={};research.audio=AudioTimeline();research.audio_sources=runtime.deque(maxlen=512)
    research.frames=runtime.deque()
    monkeypatch.setattr(runtime.time,'time',lambda:110.1)
    for i in range(21):
        start=98.+i*.5
        sec=int(start);nano=int(round((start-sec)*1e9))
        samples=np.tile(np.array([.2,.4],np.float32),16000)
        message=SimpleNamespace(channels=2,samples=samples,sample_rate=8000,source_id=f'capture-{i}',
            stamp_quality='capture_estimate',header=SimpleNamespace(stamp=SimpleNamespace(sec=sec,nanosec=nano),frame_id='microphone'))
        research.receive_audio(message)
    for i in range(10):
        stamp=100.+.4875+i*.96
        image=np.zeros((2,3,3),np.uint8)
        record=input_record('rgb','/rgb','sensor_msgs/msg/Image',stamp,0.,frame='camera',shape=image.shape,dtype='uint8',format='rgb8')
        research.frames.append((stamp,image,'camera',record))
    bundle=research.avs_bundle()
    np.testing.assert_allclose(bundle['pcm'],.3,rtol=1e-6)
    records=[r for r in bundle['inputs_used'] if r['modality']=='audio']
    assert records and all(r['channels']==2 and r['model_channels']==1 for r in records)
    owners=np.zeros(len(bundle['pcm']),np.uint8)
    for r in records:
        for a,b in r['used_sample_ranges']:owners[a:b]+=1
    assert np.all(owners==1)
    assert all(r['source_id']!='capture-0' for r in records)
