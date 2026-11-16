from copy import deepcopy
from pathlib import Path
import sys
import tempfile
import unittest
import numpy as np

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'src/rtabmap_semantic'))
from rtabmap_semantic.fusion_geometry import projection,find_transform,masked_square_crop,project_labels,rectified_reference_transform
from rtabmap_semantic.object_fusion import TaskVocabulary,ObjectFusionEngine
from rtabmap_semantic.fusion_evaluation import confusion,metrics,evaluate,load_passed_report,policy_prediction
from rtabmap_semantic.segmentation import class_label_anchors,dense_maps
from rtabmap_semantic.fusion_preview import build_preview
import cv2
import json


def camera(width=8,height=6):
    return dict(width=width,height=height,frame_id='optical',k=[1,0,0,0,1,0,0,0,1],
        p=[1,0,0,0,0,1,0,0,0,0,1,0],r=[1,0,0,0,1,0,0,0,1],d=[0.,0.,0.,0.,0.],distortion_model='plumb_bob')


class GeometryTests(unittest.TestCase):
    def test_stereo_right_P_translation_applies_once_in_reference_frame(self):
        rgb=camera();nir=camera();nir['p'][3]=-1
        u,v,valid=projection(np.ones((6,8),np.float32),rgb,nir,np.eye(4))
        self.assertEqual(u[2,3],2);self.assertEqual(v[2,3],2)
        self.assertFalse(valid[2,0])
        labels=np.arange(48,dtype=np.int32).reshape(6,8)
        mapped=project_labels(labels,u,v,valid)
        self.assertEqual(mapped[2,3],labels[2,2]);self.assertEqual(mapped[2,0],-1)

    def test_depth_and_transforms_do_not_assume_flat_scene(self):
        depth=np.ones((6,8),np.float32);depth[3,4]=2
        depth[3,5]=0
        nir=camera();nir['p'][3]=-1
        u,_,_=projection(depth,camera(),nir,np.eye(4))
        self.assertAlmostEqual(float(u[3,4]),3.5)
        depth[3,5]=1
        _,_,visible=projection(depth,camera(),nir,np.eye(4))
        self.assertFalse(visible[3,4], 'Nearer surface must occlude this correspondence')
        transforms={'right':dict(parent='left',child='right',translation=[.05,0,0],rotation=[0,0,0,1])}
        np.testing.assert_allclose(find_transform(transforms,'left','right')[:3,3],[-.05,0,0])
        with self.assertRaises(ValueError):find_transform(transforms,'unknown','right')

    def test_invalid_depth_and_crop_padding_preserve_validity(self):
        depth=np.ones((6,8),np.float32);depth[2,2]=0;depth[2,3]=np.nan
        with np.errstate(invalid='ignore'):
            _,_,valid=projection(depth,camera(),camera(),np.eye(4))
        self.assertFalse(valid[2,2]);self.assertFalse(valid[2,3])
        image=np.full((6,8,3),200,np.uint8);mask=np.zeros((6,8),bool);mask[2:4,2:6]=True
        crop=masked_square_crop(image,mask,[2,2,6,4])
        self.assertEqual(crop.shape,(4,4,3));self.assertTrue((crop[0]==127).all());self.assertTrue((crop[1]==200).all())

    def test_bad_calibration_is_rejected(self):
        with self.assertRaisesRegex(ValueError,'dimensions'):
            projection(np.ones((4,4)),camera(),camera(),np.eye(4))
        bad=camera();bad['k'][0]=0
        with self.assertRaisesRegex(ValueError,'intrinsics'):
            projection(np.ones((6,8)),bad,camera(),np.eye(4))

    def test_differing_nir_frame_requires_the_actual_stereo_baseline(self):
        nir=camera();nir['frame_id']='left';nir['p'][3]=-.05
        transforms={'right':dict(parent='left',child='right',translation=[.05,0,0],rotation=[0,0,0,1])}
        np.testing.assert_allclose(rectified_reference_transform(transforms,'left',nir,'right'),np.eye(4))
        nir['p'][3]=-.10
        with self.assertRaisesRegex(ValueError,'stereo P'):
            rectified_reference_transform(transforms,'left',nir,'right')


class FakeEncoder:
    provenance={'checkpoint_sha256':'explicit-test-fixture'}
    def encode_text(self,texts):
        data=np.zeros((len(texts),1024),np.float32)
        for i in range(len(texts)):data[i,i//2]=1
        return data
    def encode_image_crops(self,images,flags):
        self.received=images,flags
        data=np.zeros((len(images),1024),np.float32)
        for i in range(len(images)):data[i,1]=1
        return data


class FusionTests(unittest.TestCase):
    def test_actual_two_branch_computation_preserves_geometry_and_exports_independent_nir_map(self):
        task=TaskVocabulary(ROOT/'config/segmentation_fusion_task.json');encoder=FakeEncoder()
        with tempfile.TemporaryDirectory() as cache:
            engine=ObjectFusionEngine(encoder,task,cache)
            mask=np.ones((12,12),np.uint8)
            row=dict(instance_id=1,class_id=0,label='person',confidence=.9,mask_offset=[2,2],mask=mask)
            row['bbox_xyxy']=[2,2,14,14]
            nir_row={**row,'class_id':1,'label':'robot'}
            bundle=dict(rgb=np.full((18,18,3),100,np.uint8),nir=(np.indices((18,18))[0]*10).astype(np.uint8),
                depth_m=np.ones((18,18),np.float32),rgb_info=camera(18,18),nir_info=camera(18,18),
                transform=np.eye(4),rgb_instances=[row],nir_instances=[nir_row],native_names={0:'person',1:'robot'})
            result=engine.fuse(bundle)
            self.assertEqual(encoder.received[1],[False,True])
            self.assertEqual(result['features'].shape,(1,2,1024))
            self.assertEqual(result['fused_features'].shape,(1,1024))
            self.assertEqual(result['rgb_map'][5,5],0);self.assertEqual(result['nir_map'][5,5],1)
            self.assertEqual(result['fused_map'][5,5],1)
            self.assertEqual(int((result['rgb_map']>=0).sum()),int((result['fused_map']>=0).sum()))
            self.assertEqual(result['objects'][0]['weights'],[.5,.5,0.,0.])
            for name,expected in [('rgb_map',0),('nir_map',1),('fused_map',1)]:
                anchors=class_label_anchors(result[name])
                self.assertEqual(len(anchors),1);self.assertEqual(anchors[0]['class_id'],expected)
                self.assertEqual(result[name][anchors[0]['y'],anchors[0]['x']],expected)


class PreviewTests(unittest.TestCase):
    def setUp(self):
        shape=(6,8)
        self.bundle=dict(rgb=np.zeros((*shape,3),np.uint8),nir=np.zeros(shape,np.uint8),
            rgb_instances=[],nir_instances=[],native_names={1258:'floor',4097:'technician',1:'robot'})
        v,u=np.indices(shape,dtype=np.float32)
        self.result=dict(projection_u=u,projection_v=v,eval_mask=np.ones(shape,bool))

    def row(self,instance_id,class_id,label,x,y,width=1,height=1,confidence=.9):
        return dict(instance_id=instance_id,class_id=class_id,label=label,confidence=confidence,
            mask_offset=[x,y],mask=np.ones((height,width),np.uint8))

    def test_rgb_and_untouched_fusion_preserve_native_classes_and_small_same_class_instances(self):
        rows=[self.row(1,1258,'floor',0,0,8,6,confidence=.5),
            self.row(2,4097,'technician',1,1),self.row(3,4097,'technician',2,1)]
        self.bundle['rgb_instances']=rows
        preview=build_preview(self.bundle,self.result,{})
        expected,identities=dense_maps(rows,(6,8))
        for kind in ('rgb','fused'):
            np.testing.assert_array_equal(preview['maps'][kind],expected)
            self.assertEqual(preview['instance_counts'][kind],len(rows))
            self.assertEqual([r['instance_id'] for r in preview['image_labels'][kind]],[1,2,3])
            for label in preview['image_labels'][kind]:
                self.assertEqual(identities[label['y'],label['x']],label['instance_id'])
                self.assertTrue(label['label'])
            self.assertEqual(preview['image_labels'][kind][1]['pixels'],1)
            self.assertEqual(preview['image_labels'][kind][1]['class_id'],4097)
        self.assertEqual(preview['instance_counts']['nir'],0)

    def test_refinement_changes_only_the_specified_instance_and_keeps_original_rgb(self):
        rows=[self.row(1,4097,'technician',1,1,2,2),self.row(2,1258,'floor',4,3,2,2)]
        self.bundle['rgb_instances']=rows
        original,identities=dense_maps(rows,(6,8))
        preview=build_preview(self.bundle,self.result,{1:1})
        expected=original.copy();expected[identities==1]=1
        np.testing.assert_array_equal(preview['maps']['rgb'],original)
        np.testing.assert_array_equal(preview['maps']['fused'],expected)
        self.assertEqual([r['class_id'] for r in preview['image_labels']['fused']],[1,1258])
        self.assertEqual(preview['image_labels']['fused'][0]['label'],'机器人')
        self.assertEqual(rows[0]['class_id'],4097)

    def test_completely_overlapped_detection_still_has_an_instance_label(self):
        self.bundle['rgb_instances']=[self.row(1,1258,'floor',1,1,2,2),
            self.row(2,4097,'technician',1,1,2,2,confidence=.5)]
        preview=build_preview(self.bundle,self.result,{})
        self.assertEqual(preview['instance_counts']['rgb'],2)
        hidden=preview['image_labels']['rgb'][1]
        self.assertEqual(hidden['visible_pixels'],0)
        self.assertTrue(1<=hidden['x']<3 and 1<=hidden['y']<3)

    def test_nir_uses_independent_native_instances_and_actual_projection(self):
        self.bundle['rgb_instances']=[self.row(1,1258,'floor',0,0,8,6)]
        self.bundle['nir_instances']=[self.row(1,4097,'technician',2,2,2,2),
            self.row(2,1,'robot',7,5)]
        self.result['projection_u']+=1
        self.result['eval_mask'][:,-1]=False
        self.result['eval_mask'][5,:]=False
        preview=build_preview(self.bundle,self.result,{})
        native,_=dense_maps(self.bundle['nir_instances'],(6,8))
        expected=project_labels(native,self.result['projection_u'],self.result['projection_v'],self.result['eval_mask'])
        np.testing.assert_array_equal(preview['maps']['nir'],expected)
        self.assertEqual(preview['instance_counts']['nir'],1)
        label=preview['image_labels']['nir'][0]
        self.assertEqual(label['class_id'],4097)
        self.assertEqual(expected[label['y'],label['x']],4097)
        self.assertTrue(1<=label['x']<3)


class EvaluationTests(unittest.TestCase):
    def make_dataset(self,root,repair=True,nir_best=True):
        samples=[]
        object_truth=np.tile(np.array([0,1],np.int32),10).reshape(4,5)
        truth=np.repeat(object_truth,20,axis=1)
        identities=np.repeat(np.arange(1,21,dtype=np.int32).reshape(4,5),20,axis=1)
        rgb=truth.copy();rgb[truth==0]=1
        nir=truth.copy() if nir_best else rgb.copy()
        predicted=object_truth.ravel() if repair else np.ones(20,np.int32)
        logits=np.zeros((20,3,2),np.float32)
        logits[np.arange(20),2,predicted]=1
        objects=[dict(instance_id=i+1,eligible=True,predicted_task_id=int(p),baseline_task_id=int(rgb.ravel()[i]))
                 for i,p in enumerate(predicted)]
        for i in range(32):
            sample_id=str(i);folder=root/sample_id;folder.mkdir()
            split='validation' if i<2 else 'test'
            group='validation-scene' if i<2 else 'test-scene-'+str(i%5)
            annotation=root/(sample_id+'.png')
            cv2.imwrite(str(annotation),truth.astype(np.uint16))
            np.savez_compressed(folder/'arrays.npz',rgb_map=rgb,nir_map=nir,
                instance_map=identities,eval_mask=np.ones_like(truth,bool),logits=logits)
            metadata=dict(sample_id=sample_id,task_sha256='fixture-task',pipeline_sha256='fixture-pipeline',
                objects=objects,rgb_model='fixture-rgb')
            (folder/'metadata.json').write_text(json.dumps(metadata))
            samples.append(dict(sample_id=sample_id,sample_path=sample_id,annotation=annotation.name,
                split=split,group_id=group,annotation_source='human_reviewed'))
        index=root/'index.json'
        index.write_text(json.dumps(dict(classes=['a','b'],task_sha256='fixture-task',samples=samples)))
        return index

    def test_unknown_predictions_count_as_misses_and_ignored_truth_does_not_count(self):
        matrix=confusion(np.array([[0,1,-1]]),np.array([[-1,1,0]]),np.ones((1,3),bool),2)
        self.assertEqual(metrics(matrix)['pixels'],2)
        self.assertEqual(metrics(matrix)['miou'],.5)

    def test_validation_selected_fusion_meets_both_single_modalities_and_intact_report_loads(self):
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory);report=evaluate(self.make_dataset(root))
            self.assertTrue(report['passed']);self.assertTrue(report['fusion_enabled'])
            self.assertEqual(report['metrics']['nir']['miou'],1.)
            self.assertEqual(report['metrics']['fusion']['miou'],1.)
            self.assertEqual(report['common_fov_delta_vs_best_single'],0.)
            path=root/'report.json';path.write_text(json.dumps(report))
            self.assertEqual(load_passed_report(path)['pipeline_sha256'],'fixture-pipeline')
            report['confidence_threshold']=.99;path.write_text(json.dumps(report))
            with self.assertRaisesRegex(ValueError,'intact'):load_passed_report(path)

    def test_rgb_passthrough_cannot_pass_when_nir_is_better(self):
        with tempfile.TemporaryDirectory() as directory:
            report=evaluate(self.make_dataset(Path(directory),repair=False))
            self.assertFalse(report['passed']);self.assertFalse(report['fusion_enabled'])
            self.assertLess(report['delta_vs_best_single'],0.)

    def test_missing_labels_and_scene_leakage_never_pass(self):
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory);index=self.make_dataset(root)
            (root/'0.png').unlink()
            report=evaluate(index);self.assertEqual(report['status'],'needs_annotations');self.assertFalse(report['passed'])
            data=json.loads(index.read_text());data['samples'][2]['group_id']='validation-scene'
            index.write_text(json.dumps(data))
            with self.assertRaisesRegex(ValueError,'leakage'):evaluate(index)

    def test_runtime_and_evaluation_gate_preserve_ineligible_objects(self):
        arrays=dict(rgb_map=np.array([[0,0]],np.int32),instance_map=np.array([[1,2]],np.int32),
            logits=np.array([[[0.,1.]]*3,[[0.,1.]]*3]))
        objects=[dict(instance_id=1,eligible=True),dict(instance_id=2,eligible=False)]
        result,changes=policy_prediction(arrays,objects,.07,.8)
        np.testing.assert_array_equal(result,[[1,0]]);self.assertEqual(changes,1)
        original,changes=policy_prediction(arrays,objects,.07,.8,False)
        np.testing.assert_array_equal(original,arrays['rgb_map']);self.assertEqual(changes,0)


if __name__=='__main__':unittest.main()
