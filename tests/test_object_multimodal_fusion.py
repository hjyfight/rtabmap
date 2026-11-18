"""Target attribution, ambiguity, provenance and use restrictions; no hardware/control calls."""
import copy
import json
from pathlib import Path
import sys
import tempfile
import unittest
import numpy as np

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'src/rtabmap_features'))
from rtabmap_features.quality import signal_quality,SourceMonitor
from rtabmap_features.calibration import load_calibration,rigid_matrix
from rtabmap_features.fusion import Sample,MODALITIES
from rtabmap_features.target_association import select_target_points,acoustic_bearing,match_sound_to_target
from rtabmap_features.target_tracking import TargetTracker
from rtabmap_features.object_multimodal import ObjectMultimodalEngine
from rtabmap_features.standard_interface import seal_result,validate_result,require_control_approval


def sound_fixture(direction=(0,0,1)):
    positions=np.array([[0,0,0],[.06,0,0],[0,.05,0],[0,0,.04]])
    random=np.random.default_rng(7);base=random.normal(0,.02,5000)
    times=np.arange(4096)+64
    samples=np.column_stack([np.interp(times+(p@np.asarray(direction))*16000/343,np.arange(len(base)),base) for p in positions])
    return samples.astype(np.float32),positions


def scene():
    random=np.random.default_rng(2);rgb=random.integers(30,220,(64,64,3),dtype=np.uint8)
    mask=np.zeros((64,64),bool);mask[16:48,16:48]=True;depth=np.full((64,64),2.,np.float32)
    y,x=np.nonzero(mask);points=np.column_stack([(x-32)*2/100,(y-32)*2/100,np.full(len(x),2)]).astype(np.float32)
    samples,_=sound_fixture()
    sources={n:dict(present=True,modality=n,source_id='fixture:'+n,frame_id=n,stamp=100.,
                   quality=dict(usable=True,score=1.,grade='nominal'),authenticated=False) for n in MODALITIES}
    return dict(stamp=100.,sequence=1,rgb=rgb,depth_m=depth,rgb_k=[100,0,32,0,100,32,0,0,1],rgb_frame='rgb',
        instances=[dict(instance_id=7,label='person',full_mask=mask)],sources=sources,
        nir_warped=rgb[:,:,0],nir_valid=np.ones(mask.shape,bool),nir_frame='infrared',
        lidar=Sample(points,100.,'lidar','fixture-lidar',attributes={'units':'m'}),camera_from_lidar=np.eye(4),
        audio=Sample(samples,99.9,'audio','fixture-audio',len(samples)/16000,{'sample_rate':16000}),
        common_from_rgb=np.eye(4),common_from_audio=np.eye(4))


class FixtureEncoder:
    provenance=dict(checkpoint_sha256='explicit_unit_test_fixture',inference_device='cpu')
    def __init__(self,conflict=False):self.conflict=conflict;self.received=[]
    def encode_text(self,texts):
        values=np.zeros((len(texts),1024),np.float32)
        for i in range(len(texts)):values[i,i//2]=1
        return values
    def encode(self,samples,anchor):
        self.received.append(set(samples));features={}
        for name in samples:
            vector=np.zeros(1024,np.float32);vector[1 if self.conflict and name=='infrared' else 0]=1
            features[name]=vector
        return features,{n:{'fixture':True} for n in samples},{'fixture_s':0.}


class QualityTests(unittest.TestCase):
    def test_signal_defects_are_measured_and_distinct_from_hardware_health(self):
        self.assertIn('overexposure',signal_quality('rgb',np.full((16,16,3),255,np.uint8))['reasons'])
        self.assertIn('silence',signal_quality('audio',np.zeros((1600,4)))['reasons'])
        self.assertFalse(signal_quality('lidar',np.array([[np.nan,0,0]]))['usable'])
        self.assertIn('insufficient_distinct_points',signal_quality('lidar',np.ones((10,3)))['reasons'])

    def test_regression_duplicate_and_constant_signal_are_not_silently_fused(self):
        monitor=SourceMonitor(freeze_s=1.)
        image=np.zeros((8,8,3),np.uint8)
        first=monitor.observe('rgb',image,10.,'rgb:1','camera',10.)
        self.assertEqual(first['sensor_health'],'unverified')
        duplicate=monitor.observe('rgb',image,10.,'rgb:1','camera',10.1)
        self.assertFalse(duplicate['quality']['usable'])
        frozen=monitor.observe('rgb',image,11.1,'rgb:2','camera',11.1)
        self.assertIn('suspected_frozen_or_constant_signal',frozen['continuity'])
        regressed=monitor.observe('rgb',image,9.,'rgb:3','camera',11.2)
        self.assertFalse(regressed['quality']['usable'])


class AssociationTests(unittest.TestCase):
    def test_target_cloud_excludes_other_pixels_and_occluded_surfaces(self):
        bundle=scene();points=bundle['lidar'].payload
        extra=np.array([[0,0,4],[1,1,2],[0,0,-1],[np.nan,0,2]])
        selected,details=select_target_points(np.vstack([points,extra]),np.eye(4),bundle['rgb_k'],bundle['instances'][0]['full_mask'],bundle['depth_m'])
        self.assertEqual(len(selected),1024);self.assertEqual(details['target_distinct_points'],1024)

    def test_gcc_phat_recovers_direction_from_delayed_pcm(self):
        samples,positions=sound_fixture()
        bearing=acoustic_bearing(samples,16000,positions)
        self.assertTrue(bearing['valid'],bearing)
        self.assertGreater(np.asarray(bearing['directions'][0])@[0,0,1],.95)
        self.assertFalse(bearing['distance_estimated'])

    def test_sound_ambiguity_does_not_duplicate_audio_across_targets(self):
        samples,positions=sound_fixture();bearing=acoustic_bearing(samples,16000,positions)
        matched=match_sound_to_target(bearing,np.eye(4),[{'position_common':[0,0,2]},{'position_common':[0,0,3]}])
        self.assertIsNone(matched['target_index']);self.assertIn('multiple_targets',matched['reason'])
        unique=match_sound_to_target(bearing,np.eye(4),[{'position_common':[0,0,2]},{'position_common':[2,0,2]}])
        self.assertEqual(unique['target_index'],0)

    def test_silence_and_linear_array_do_not_claim_a_unique_direction(self):
        samples,positions=sound_fixture()
        self.assertFalse(acoustic_bearing(np.zeros_like(samples),16000,positions)['valid'])
        positions[:,1:]=0
        self.assertFalse(acoustic_bearing(samples,16000,positions)['valid'])


class TrackingTests(unittest.TestCase):
    def test_identity_survives_short_occlusion_and_retains_measured_motion(self):
        tracker=TargetTracker('map','fixture')
        observation=dict(label='person',position_common=[0,0,2],feature=np.eye(1,1024)[0])
        first,_=tracker.update([observation],100.)
        tracker.update([],100.5,{first[0]['target_id']:'occluded'})
        observation['position_common']=[.4,0,2]
        second,events=tracker.update([observation],101.)
        self.assertEqual(first[0]['target_id'],second[0]['target_id'])
        self.assertTrue(second[0]['reassociated']);self.assertGreater(second[0]['speed_m_s'],.15)
        self.assertEqual(events[0]['type'],'target_reassociated')

    def test_persistent_removal_requires_repeated_clear_visibility(self):
        tracker=TargetTracker('map','fixture');observation=dict(label='box',position_common=[0,0,2],feature=np.eye(1,1024)[0])
        result,_=tracker.update([observation],100.);identity=result[0]['target_id']
        for stamp in (100.1,100.2,100.3):tracker.update([observation],stamp)
        for stamp in (100.4,100.5):
            _,events=tracker.update([],stamp,{identity:'clear_line_of_sight'})
            self.assertNotEqual(events[0]['type'],'persistent_removed')
        _,events=tracker.update([],100.6,{identity:'clear_line_of_sight'})
        self.assertEqual(events[0]['type'],'persistent_removed')

    def test_saved_identity_is_scoped_to_calibration_and_rejects_replayed_time(self):
        with tempfile.TemporaryDirectory() as directory:
            path=Path(directory)/'state.json';observation=dict(label='box',position_common=[0,0,2],feature=np.eye(1,1024)[0])
            original=TargetTracker('map','v1',state_path=path);first,_=original.update([observation],100.)
            loaded=TargetTracker('map','v1',state_path=path);next_,_=loaded.update([observation],100.5)
            self.assertEqual(first[0]['target_id'],next_[0]['target_id'])
            with self.assertRaises(ValueError):loaded.update([observation],100.)
            changed=TargetTracker('map','v2',state_path=path);new,_=changed.update([observation],101.)
            self.assertNotEqual(new[0]['target_id'],first[0]['target_id'])


class EngineTests(unittest.TestCase):
    def setUp(self):
        self.task=json.loads((ROOT/'config/object_multimodal_task.json').read_text())
        self.calibration=load_calibration(ROOT/'config/object_multimodal_calibration.json')
        self.calibration['sensors']['lidar']['status']='measured'
        audio=self.calibration['sensors']['audio'];audio['status']='measured'
        _,geometry=sound_fixture();audio['microphone_positions_m']=geometry.tolist()

    def test_complete_four_modal_target_uses_real_geometric_association_and_keeps_restrictions(self):
        encoder=FixtureEncoder();engine=ObjectMultimodalEngine(encoder,self.task,self.calibration)
        result=engine.compute(scene());target=result['targets'][0]
        self.assertEqual(encoder.received[0],set(MODALITIES));self.assertTrue(target['features']['complete'])
        self.assertEqual(len(target['features']['joint']),4096)
        self.assertTrue(target['modalities']['audio']['associated']);self.assertTrue(target['modalities']['lidar']['associated'])
        self.assertTrue(result['safety']['prohibit_automatic_execution'])
        self.assertEqual(result['timing']['period_verdict'],'not_assessed_task_period_unspecified')
        self.assertFalse(result['calibration']['synchronization_verified'])
        with self.assertRaises(ValueError):require_control_approval(result)

    def test_unverified_sound_mount_is_explicitly_excluded(self):
        self.calibration['sensors']['audio']['status']='unverified'
        result=ObjectMultimodalEngine(FixtureEncoder(),self.task,self.calibration).compute(scene())
        target=result['targets'][0];self.assertFalse(target['features']['valid_mask'][3])
        self.assertEqual(target['features']['weights'][3],0)
        self.assertEqual(target['modalities']['audio']['basis'],'audio_geometry_or_mounting_unverified')

    def test_conflicting_semantics_keeps_baseline_and_records_participants(self):
        result=ObjectMultimodalEngine(FixtureEncoder(True),self.task,self.calibration).compute(scene())
        target=result['targets'][0];self.assertEqual(target['label'],'person')
        conflict=target['uncertainty']['conflicts'][0]
        self.assertIn('infrared',conflict['participating_modalities'])
        self.assertEqual(conflict['handling'],'retain_rgb_detection_and_require_review')

    def test_tampered_payload_signature_and_vector_layout_are_rejected(self):
        result=ObjectMultimodalEngine(FixtureEncoder(),self.task,self.calibration).compute(scene())
        signed=seal_result(copy.deepcopy(result),b'test-key');validate_result(signed,b'test-key')
        with self.assertRaises(ValueError):validate_result(signed,b'wrong-key')
        signed['targets'][0]['label']='forged'
        with self.assertRaises(ValueError):validate_result(signed,b'test-key')
        result['targets'][0]['features']['joint'][0]=9
        with self.assertRaises(ValueError):validate_result(seal_result(result))

    def test_invalid_calibration_rotation_and_unsubstantiated_measured_record_fail(self):
        wrong=np.eye(4);wrong[0,0]=2
        with self.assertRaises(ValueError):rigid_matrix(wrong)
        with tempfile.TemporaryDirectory() as directory:
            profile=copy.deepcopy(self.calibration);profile['sensors']['rgb']['status']='measured'
            path=Path(directory)/'profile.json';path.write_text(json.dumps(profile))
            with self.assertRaisesRegex(ValueError,'evidence'):load_calibration(path)


if __name__=='__main__':unittest.main()
