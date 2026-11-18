"""Independent geometric/time references, lost targets and malformed consumer inputs."""
import copy
import hashlib
import importlib.util
import json
from pathlib import Path
import sys
import tempfile
import unittest
import numpy as np

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'src/rtabmap_features'))
from test_object_multimodal_fusion import FixtureEncoder,scene,sound_fixture
from rtabmap_features.calibration import load_calibration
from rtabmap_features.object_multimodal import ObjectMultimodalEngine
from rtabmap_features.target_association import target_geometry,target_absence_evidence,acoustic_window,match_sound_to_target
from rtabmap_features.standard_interface import seal_result,validate_result
from rtabmap_features.result_journal import publication_observation,validate_observation,ResultJournal
from rtabmap_features.object_policy import validate_task
from rtabmap_features.target_tracking import TargetTracker

def script(name):
    spec=importlib.util.spec_from_file_location(name,ROOT/'scripts'/f'{name}.py')
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module);return module

fit=script('fit_object_multimodal_calibration');cycle=script('measure_object_multimodal_cycle')
live=script('prepare_object_multimodal_live')

def measurements():
    points=np.array([[0,0,0],[1,0,0],[0,1,0],[0,0,1],[1,1,1]],float)
    angle=.31;rotation=np.array([[np.cos(angle),-np.sin(angle),0],[np.sin(angle),np.cos(angle),0],[0,0,1]])
    result=dict(schema=1,version='synthetic_fit',common_frame='base_link',measurement_scope='synthetic_fixture',
        reference_event_method='generated_common_events',instrument_reference='explicit_synthetic_instrument',sensors={})
    for name in ('rgb','infrared','lidar','audio'):
        result['sensors'][name]=dict(frame_id=name,source_points_m=points.tolist(),reference_points_m=(points@rotation.T+[.1,.2,.3]).tolist(),
            source_event_stamps=list(np.arange(10.)+100),reference_event_stamps=list(np.arange(10.)+100.023),
            instrument_uncertainty_s=.0001,max_geometry_rmse_m=.01,sampling_period_s=.05)
    result['sensors']['audio']['microphone_positions_m']=sound_fixture()[1].tolist()
    return result

class EvidenceTests(unittest.TestCase):
    def setUp(self):
        self.task=json.loads((ROOT/'config/object_multimodal_task.json').read_text())
        self.profile=load_calibration(ROOT/'config/object_multimodal_calibration.json')
        self.result=ObjectMultimodalEngine(FixtureEncoder(),self.task,self.profile).compute(scene())

    def test_live_clock_estimate_preserves_mounting_and_original_profile(self):
        original=ROOT/'config/object_multimodal_calibration.json';original_bytes=original.read_bytes()
        with tempfile.TemporaryDirectory() as directory:
            clock=Path(directory)/'clock.json';output=Path(directory)/'live.json'
            clock.write_text(json.dumps(dict(method='ssh_wall_clock_midpoint_min_rtt',measured_at=1000.,
                workstation_minus_robot_s=57.5,round_trip_uncertainty_s=.1,hardware_synchronization_validated=False)))
            live.prepare_profile(original,clock,output,now=1001.)
            profile=load_calibration(output)
            self.assertEqual(profile['measurement_scope'],'estimated_clock_offsets')
            for name,sensor in profile['sensors'].items():
                self.assertEqual(sensor['clock_offset_s'],57.5)
                self.assertEqual(sensor['status'],self.profile['sensors'][name]['status'])
                self.assertEqual(sensor['common_from_sensor'],self.profile['sensors'][name]['common_from_sensor'])
            result=ObjectMultimodalEngine(FixtureEncoder(),self.task,profile).compute(scene())
            self.assertFalse(result['calibration']['synchronization_verified'])
        self.assertEqual(original.read_bytes(),original_bytes)

    def test_live_clock_estimate_rejects_bad_reports_and_measured_calibration(self):
        with tempfile.TemporaryDirectory() as directory:
            clock=Path(directory)/'clock.json';output=Path(directory)/'live.json'
            report=dict(method='ssh_wall_clock_midpoint_min_rtt',measured_at=1000.,
                workstation_minus_robot_s=57.5,round_trip_uncertainty_s=.1,hardware_synchronization_validated=False)
            for change in ({'measured_at':-90000.},{'measured_at':100002.},{'workstation_minus_robot_s':True},
                           {'round_trip_uncertainty_s':-1.},{'hardware_synchronization_validated':True}):
                with self.subTest(change=change):
                    clock.write_text(json.dumps(dict(report,**change)))
                    with self.assertRaises(ValueError):live.prepare_profile(
                        ROOT/'config/object_multimodal_calibration.json',clock,output,now=1001.)
            filename,_=fit.fit_measurements(measurements(),directory)
            clock.write_text(json.dumps(report))
            with self.assertRaisesRegex(ValueError,'Preserve measured calibration'):
                live.prepare_profile(filename,clock,output,now=1001.)

    def test_fit_known_rotation_offset_and_evidence_integrity(self):
        with tempfile.TemporaryDirectory() as directory:
            filename,assessment=fit.fit_measurements(measurements(),directory)
            profile=load_calibration(filename)
            self.assertAlmostEqual(profile['sensors']['rgb']['clock_offset_s'],.023,places=9)
            self.assertLess(json.loads((Path(directory)/'fit_evidence.json').read_text())['sensors']['rgb']['geometry']['rmse_m'],1e-12)
            self.assertFalse(assessment['synchronization_verified'])
            evidence=Path(directory)/'fit_evidence.json';evidence.write_text('{}')
            with self.assertRaisesRegex(ValueError,'evidence'):load_calibration(filename)

    def test_clock_and_geometry_bad_reference_rejected(self):
        with self.assertRaises(ValueError):fit.fit_clock([1,2,3,4,5],[1,2,3,4,5],float('nan'))
        with self.assertRaises(ValueError):fit.fit_rigid([[0,0,0]]*4,[[1,0,0]]*4)
        with tempfile.TemporaryDirectory() as directory:
            bad=measurements();bad['sensors']['rgb']['reference_points_m'][-1][0]+=1
            filename,assessment=fit.fit_measurements(bad,directory)
            self.assertIn('rgb',assessment['missing'])

    def test_audio_analysis_window_follows_anchor(self):
        samples=np.column_stack([np.arange(32000)]*4)
        selected,meta=acoustic_window(samples,16000,1.5)
        self.assertAlmostEqual(meta['start_offset_s'],1.375)
        self.assertEqual(selected[0,0],22000)
        with self.assertRaises(ValueError):acoustic_window(samples,16000,3)

    def test_planar_array_reflection_cannot_claim_unique_source(self):
        bearing=dict(valid=True,directions=[[0,0,1],[0,0,-1]],planar_reflection_ambiguity=True)
        self.assertIsNone(match_sound_to_target(bearing,np.eye(4),[{'position_common':[0,0,2]}])['target_index'])

    def test_depth_absence_requires_background_beyond_old_volume(self):
        bundle=scene();tracks={'target':dict(position=[0,0,2],extent=[.5,.5,.1])}
        clear=np.full((64,64),4.,np.float32);occupied=np.zeros((64,64),bool)
        self.assertEqual(target_absence_evidence(tracks,np.eye(4),bundle['rgb_k'],clear,occupied),{'target':'clear_line_of_sight'})
        self.assertEqual(target_absence_evidence(tracks,np.eye(4),bundle['rgb_k'],clear*.25,occupied),{'target':'occluded'})
        self.assertEqual(target_absence_evidence(tracks,np.eye(4),bundle['rgb_k'],bundle['depth_m'],occupied),{})
        occupied[32,32]=True
        self.assertEqual(target_absence_evidence(tracks,np.eye(4),bundle['rgb_k'],clear,occupied),{})

    def test_runtime_removal_and_moving_departure_have_distinct_events(self):
        engine=ObjectMultimodalEngine(FixtureEncoder(),self.task,self.profile)
        bundle=scene();first=engine.compute(bundle);identity=first['targets'][0]['target_id']
        for i in (1,2,3):
            current=scene();current.update(stamp=100+i*.1,sequence=i+1);engine.compute(current)
        for i in (1,2,3):
            current=scene();current.update(stamp=101+i*.1,sequence=i+4,instances=[]);current['depth_m'][:]=4
            result=engine.compute(current)
        self.assertIn('persistent_removed',[e['type'] for e in result['events'] if e['target_id']==identity])

    def test_unknown_disappearance_and_measured_motion_are_not_static_removal(self):
        for moving in (False,True):
            tracker=TargetTracker('map','test');obs=dict(label='person',position_common=[0,0,2],feature=np.eye(1,1024)[0],extent=[.4,.4,1.8])
            first,_=tracker.update([obs],100.);identity=first[0]['target_id']
            if moving:
                obs['position_common']=[.4,0,2];tracker.update([obs],101.)
            for stamp in (101.1,101.2,101.3):_,events=tracker.update([],stamp,{identity:'clear_line_of_sight'})
            self.assertEqual(events[0]['type'],'transient_departed' if moving else 'disappeared_unclassified')

    def test_tracked_geometry_change_is_explicit_without_fabricated_door_state(self):
        tracker=TargetTracker('map','test');obs=dict(label='door',position_common=[0,0,2],feature=np.eye(1,1024)[0],extent=[.1,1,2])
        first,_=tracker.update([obs],100.);obs['extent']=[.8,.2,2]
        second,events=tracker.update([obs],100.1)
        self.assertEqual(first[0]['target_id'],second[0]['target_id'])
        self.assertIn('structure_geometry_changed',[e['type'] for e in events])

    def test_distorted_camera_rays_recover_correct_point(self):
        import cv2
        k=np.array([[100,0,32],[0,100,32],[0,0,1]],float);d=np.array([.5,0,0,0,0])
        depth=np.full((64,64),2.);mask=np.zeros((64,64),bool);mask[44:50,44:50]=True
        geometry=target_geometry(mask,depth,k,d)
        rays=cv2.undistortPoints(np.array([[[46.5,46.5]]]),k,d).reshape(2)*2
        np.testing.assert_allclose(geometry['position'][:2],rays,atol=.001)

    def test_consumer_rejects_semantic_tampering_even_with_resealed_hash(self):
        changes=[lambda r:r.update(sequence=True),
            lambda r:r['targets'][0]['features']['valid_mask'].__setitem__(0,1),
            lambda r:r['targets'][0]['features']['fused'].__setitem__(5,.4),
            lambda r:r['targets'][0]['modalities']['rgb'].update(associated=False),
            lambda r:r['targets'][0]['position'].update(frame_id='wrong'),
            lambda r:r['targets'][0]['safety'].update(prohibit_automatic_execution=False)]
        for change in changes:
            result=copy.deepcopy(self.result);change(result)
            with self.assertRaises(ValueError):validate_result(seal_result(result))

    def test_timing_unknown_period_and_complete_boundaries(self):
        observed=publication_observation(self.result,10.,10.2,10.9,11.)
        report=cycle.cycle_report([observed]);self.assertIsNone(report['task_period_s'])
        self.assertEqual(report['verdict'],'not_assessed_task_period_unspecified')
        self.assertAlmostEqual(report['timing']['processing_s']['max_s'],.8)
        self.assertEqual(cycle.cycle_report([observed],.7)['verdict'],'measured_processing_exceeds_period')
        self.assertEqual(cycle.cycle_report([self.result],2.)['verdict'],'incomplete_processing_boundary')
        with self.assertRaises(ValueError):cycle.cycle_report([observed,self.result])
        observed['timing']['processing_s']=.01
        with self.assertRaises(ValueError):validate_observation(observed)

    def test_journal_rotates_bounded_evidence_and_invalid_policy_fails(self):
        observed=publication_observation(self.result,10.,10.2,10.9,11.)
        with tempfile.TemporaryDirectory() as directory:
            path=Path(directory)/'results.jsonl';journal=ResultJournal(path,max_bytes=100,backups=2)
            for _ in range(4):journal.append(observed)
            self.assertEqual(len(list(Path(directory).iterdir())),3)
            validate_observation(json.loads(path.read_text()))
        for key,value in [('base_weights',[1,1,-1,1]),('max_targets',True),('task_period_s',0),('classes',[])]:
            task=copy.deepcopy(self.task);task[key]=value
            with self.assertRaises(ValueError):validate_task(task)

if __name__=='__main__':unittest.main()
