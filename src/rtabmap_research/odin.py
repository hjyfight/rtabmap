"""Frozen indoor ODIN: posed RGB-D context queried at actual Livox points."""
import hashlib
from pathlib import Path
import numpy as np
import torch
from torch import nn
from torch.nn import functional as F

from .models import Trace,predictions,annotated_predictions,jpeg
from .point_scene import build_scene
from .ptv3 import CHINESE
from .odin_runtime import load_runtime,ODIN_COMMIT,D2_COMMIT

EXPECTED_SHA256='ecbf1292b60fe2ccbc301be283739eb08313f7c59a7b1ffc172d010b34d49c21'
SWIN_SHA256='fc4e7ffd4507ea334ed7d96c5be146b786fd18c4abd42cb39138bd39c71856f5'
# ODIN moves ScanNet wall/floor after its eighteen instance classes.
NATIVE_TO_SCANNET=np.asarray(list(range(2,20))+[0,1],np.uint8)
NAMES=[CHINESE[i] for i in NATIVE_TO_SCANNET]


class ODIN:
    def __init__(self,weights,device='cuda:0',dense=True,backbone_kind=None):
        backbone_kind=backbone_kind or ('swin' if 'swin' in Path(weights).name.lower() else 'resnet')
        cfg,backbone,head,geometry,shape=load_runtime(backbone_kind)
        self.cfg,self.geometry,self.device=cfg,geometry,device
        self.dense=dense
        self.net=nn.Module()
        self.net.backbone=backbone(cfg,shape(channels=3))
        self.net.sem_seg_head=head(cfg,self.net.backbone.output_shape())
        self.net.criterion=nn.Module()
        self.net.criterion.register_buffer('empty_weight',torch.empty(21))
        digest=hashlib.sha256()
        with Path(weights).open('rb') as stream:
            for block in iter(lambda:stream.read(1024*1024),b''):digest.update(block)
        expected=SWIN_SHA256 if backbone_kind=='swin' else EXPECTED_SHA256
        if digest.hexdigest()!=expected:raise ValueError('Official ODIN checkpoint SHA256 mismatch')
        checkpoint=torch.load(weights,map_location='cpu',weights_only=True)
        state=checkpoint['model']
        self.net.load_state_dict(state,strict=True)
        self.net.eval().requires_grad_(False).to(device)
        self.trace=Trace()
        self.metadata=dict(name='ODIN-Swin' if backbone_kind=='swin' else 'ODIN',backbone=backbone_kind,
            dataset='ScanNet20',classes=20,strict=True,tensors=len(state),frozen=True,
            backend='author PyTorch attention + exact Euclidean kNN',backend_kind='rgbd_lidar_odin',
            repository='https://github.com/ayushjain1144/odin',commit=ODIN_COMMIT,detectron2_commit=D2_COMMIT,
            file=Path(weights).name,sha256=digest.hexdigest(),input='posed RGB-D context + original Livox / valid depth point queries',
            native_classes=NAMES,native_to_shared_scannet=NATIVE_TO_SCANNET.tolist(),
            segment_pooling=False,extra_trainable_parameters=0)
        for name,module in self.net.backbone.named_modules():
            if not (name.startswith('cross_view_attn.') and name.count('.')==1 or name.endswith('.cross_view_attn')):continue
            def receive(_,args,out,name=name):self.trace.record('backbone.'+name+'.3d_context',out[0],'作者几何邻域注意力，实际RGB-D位姿对应')
            self.trace.handles.append(module.register_forward_hook(receive))
        for number,module in enumerate(self.net.sem_seg_head.pixel_decoder.cross_view_attn):
            def receive(_,args,out,number=number):self.trace.record('pixel.scale'+str(number)+'.3d_context',out[0],'作者跨视角几何邻域特征交互')
            self.trace.handles.append(module.register_forward_hook(receive))
        self.trace.handles.append(self.net.sem_seg_head.pixel_decoder.mask_features.register_forward_hook(
            lambda _,args,out:self.trace.record('geometry.mask_features',out,'RGB-D上下文插值到真实LiDAR／有效深度点坐标的256维特征')))
        def class_logits(_,args,out):
            self.trace.rows=[row for row in self.trace.rows if row['name']!='queries.class_logits']
            self.trace.record('queries.class_logits',out,'作者100个物体查询，20类加无物体；最终解码层')
        self.trace.handles.append(self.net.sem_seg_head.predictor.class_embed.register_forward_hook(class_logits))

    @torch.inference_mode()
    def infer_scene(self,scene,observations,matrix):
        import cv2
        self.trace.clear()
        rgb_frames=np.stack([row['rgb'] for row in observations])
        depths=[];missing=[]
        for row in observations:
            depth=row['depth'].astype(np.float32)
            if row['depth'].dtype==np.uint16:depth/=1000
            elif row['depth'].dtype!=np.float32:raise ValueError('Unknown aligned depth units')
            if depth.shape!=row['rgb'].shape[:2]:raise ValueError('ODIN requires registered RGB-D')
            if np.any(np.asarray(row['distortion'])!=0):raise ValueError('ODIN needs rectified RGB-D intrinsics')
            mask=~np.isfinite(depth)|(depth<=.1)|(depth>=10)
            if 1-mask.mean()<.35:raise ValueError('Insufficient valid RGB-D depth coverage')
            # Author's ScanNet preprocessing inpaints holes in millimetres, radius 5.
            millimetres=depth*1000;millimetres[mask]=0
            filled=cv2.inpaint(millimetres,mask.astype(np.uint8),5,cv2.INPAINT_TELEA)
            millimetres[mask]=filled[mask]
            depths.append(millimetres/1000);missing.append(mask)
        mean=np.asarray(self.cfg.MODEL.PIXEL_MEAN,np.float32)[None,:,None,None]
        std=np.asarray(self.cfg.MODEL.PIXEL_STD,np.float32)[None,:,None,None]
        images=torch.from_numpy((rgb_frames.transpose(0,3,1,2).astype(np.float32)-mean)/std).to(self.device)
        v,_,h,w=images.shape;pad_h=(-h)%32;pad_w=(-w)%32
        images=F.pad(images,(0,pad_w,0,pad_h));shape=[1,v,h+pad_h,w+pad_w]
        depth=torch.from_numpy(np.stack(depths)).to(self.device)
        depth=F.pad(depth,(0,pad_w,0,pad_h))[None]
        poses=torch.from_numpy(np.stack([row['world_from_lidar']@np.linalg.inv(matrix) for row in observations]).astype(np.float32)).to(self.device)[None]
        intrinsics=torch.from_numpy(np.asarray([row['k'] for row in observations],np.float32).reshape(1,v,3,3)).to(self.device)
        xyz=self.geometry.backproject_depth(depth,poses,intrinsics)
        scales=[]
        for stride in (32,16,8,4):
            scales.append(F.interpolate(xyz[0].permute(0,3,1,2),size=(shape[2]//stride,shape[3]//stride),mode='nearest').permute(0,2,3,1)[None])
        mv=dict(multi_scale_xyz=scales,multi_scale_p2v=self.geometry.multiscsale_voxelize(scales,self.cfg.INPUT.VOXEL_SIZE[::-1]))
        target=torch.from_numpy(scene['xyz']).to(self.device)[None]
        lidar_count=target.shape[1]
        dense_valid=None
        if self.dense:
            # Extra queries are genuine current-frame depth samples, never inpainted holes.
            raw_depth=observations[-1]['depth'].astype(np.float32)
            if observations[-1]['depth'].dtype==np.uint16:raw_depth/=1000
            dense_valid=np.isfinite(raw_depth)&(raw_depth>.1)&(raw_depth<10)
            safe_depth=np.where(dense_valid,raw_depth,0)
            dense_xyz=self.geometry.unproject(intrinsics[:,-1:],poses[:,-1:],
                torch.from_numpy(safe_depth).to(self.device)[None,None])[0,0]
            target=torch.cat([target,dense_xyz[torch.from_numpy(dense_valid).to(self.device)][None]],dim=1)
        target_p2v=self.geometry.voxelization(target,self.cfg.INPUT.VOXEL_SIZE[0])
        features=self.net.backbone(images,scales,shape=shape,multiview_data=mv,decoder_3d=True)
        output=self.net.sem_seg_head(features,shape=shape,multiview_data=mv,scannet_pc=target,
            scannet_p2v=target_p2v,segments=None,decoder_3d=True)
        logits=output['pred_logits'][0];masks=output['pred_masks'][0][:,target_p2v[0]]
        if logits.shape!=(100,21) or masks.shape!=(100,target.shape[1]):
            raise ValueError('Unexpected native ODIN output shape')
        scores=torch.einsum('qc,qn->cn',logits.softmax(-1)[:,:-1],masks.sigmoid())
        if not torch.isfinite(scores).all():raise ValueError('Nonfinite ODIN output')
        query_labels=scores.argmax(0).cpu().numpy().astype(np.uint8)
        labels=query_labels[:lidar_count]
        self.trace.arrays.update(query_logits=logits.cpu().numpy(),query_point_mask_logits=masks.cpu().numpy(),
            semantic_scores=scores.cpu().numpy(),native_point_labels=labels,shared_scannet_labels=NATIVE_TO_SCANNET[labels],
            native_query_labels=query_labels,target_query_xyz=target.cpu().numpy(),
            target_query_sensor=np.r_[np.zeros(lidar_count,np.uint8),np.ones(len(query_labels)-lidar_count,np.uint8)],
            target_point_to_voxel=target_p2v.cpu().numpy(),source_rgb_frames=rgb_frames,
            source_depth_frames=np.stack([row['depth'] for row in observations]),
            model_inpainted_depth_m=np.stack(depths),depth_inpaint_mask=np.stack(missing),
            source_camera_poses=poses.cpu().numpy(),source_rgb_intrinsics=intrinsics.cpu().numpy(),
            source_rgb_distortion=np.asarray([row['distortion'] for row in observations]),
            scene_xyz=scene['xyz'],scene_rgb=scene['rgb'],scene_normals=scene['normals'],point_source_indices=scene['source_indices'],
            source_lidar_poses=np.stack([row['world_from_lidar'] for row in observations]),
            projection_uv=scene['uv'],render_indices=scene['render_indices'],camera_from_lidar=matrix)
        for i,row in enumerate(observations):self.trace.arrays['source_lidar_'+str(i)]=row['points']
        if dense_valid is not None:
            dense_labels=np.zeros((h,w),np.uint8);dense_labels[dense_valid]=query_labels[lidar_count:]+1
            self.trace.arrays.update(dense_display_labels=dense_labels,dense_valid=dense_valid)
        return labels,dict(depth_inpaint_fraction=float(np.mean(missing)),views=v,points=len(labels),
            depth_query_points=len(query_labels)-lidar_count,
            voxel_points=int(target_p2v.max())+1,native_output_shape=list(scores.shape))

    def infer_bundle(self,bundle):
        import cv2
        scene=build_scene(bundle['observations'],bundle['matrix'])
        labels,stats=self.infer_scene(scene,bundle['observations'],bundle['matrix'])
        rgb=scene['current_rgb'];uv=scene['uv'];indices=scene['render_indices']
        display=np.zeros(rgb.shape[:2],np.uint8);display[uv[:,1],uv[:,0]]=labels[indices]+1
        mask=display>0
        pictures,classes=predictions(rgb,display,['忽略']+NAMES,mask)
        annotations=annotated_predictions(pictures,rgb,display,['忽略']+NAMES,mask,task='indoor')
        point_pictures=pictures
        if self.dense:
            dense=self.trace.arrays['dense_display_labels']
            pictures,classes=predictions(rgb,dense,['忽略']+NAMES)
            annotations=annotated_predictions(pictures,rgb,dense,['忽略']+NAMES,task='indoor_rgbd')
            pictures['lidar_overlay']=point_pictures['annotated_overlay']
            pictures['lidar_plain']=point_pictures['overlay']
        projection=rgb.copy()
        for u,v in uv:cv2.circle(projection,(int(u),int(v)),1,(66,235,198),-1)
        pictures['projection']=jpeg(projection)
        self.trace.arrays['display_labels']=display
        stamps=np.asarray([[row['rgb_stamp'],row['lidar_stamp']] for row in bundle['observations']])
        self.trace.arrays['source_stamps']=stamps
        return dict(mode='indoor_rgbd_lidar',images=pictures,classes=classes,annotations=annotations,
            features=list(self.trace.rows),output_shape=stats['native_output_shape'],projected_points=len(uv),
            scene_points=len(labels),observation_count=len(bundle['observations']),window_s=float(np.ptp(stamps[:,0])),
            fusion_quality=dict(projection_coverage=float(mask.mean()),valid_lidar_points=len(uv)),input_statistics=stats,
            baseline_note='作者ODIN完整室内权重；RGB-D几何上下文与真实LiDAR点查询。点段聚合关闭；非官方基准完整场景。'),display


class ODINComparison(ODIN):
    def __init__(self,weights,device='cuda:0',backbone_kind=None):
        super().__init__(weights,device,backbone_kind=backbone_kind)
        from .ptv3 import PointTransformer,ROOT
        self.reference=PointTransformer(ROOT/'models/ptv3_scannet_best.pth',device)
        self.metadata['comparison_model']=self.reference.metadata

    def infer_bundle(self,bundle):
        result,display=super().infer_bundle(bundle)
        arrays=self.trace.arrays
        with torch.random.fork_rng(devices=[]):
            torch.random.default_generator.manual_seed(123)
            labels,_=self.reference.infer_points(arrays['scene_xyz'],arrays['scene_rgb'],arrays['scene_normals'])
        uv=arrays['projection_uv'];indices=arrays['render_indices'];rgb=arrays['source_rgb_frames'][-1]
        old=np.zeros(rgb.shape[:2],np.uint8);old[uv[:,1],uv[:,0]]=labels[indices]+1
        pictures,_=predictions(rgb,old,['忽略']+CHINESE,old>0)
        annotated_predictions(pictures,rgb,old,['忽略']+CHINESE,old>0,task='indoor')
        result['images'].update(reference_overlay=pictures['annotated_overlay'],reference_plain=pictures['overlay'])
        result['comparison']=dict(kind='different_rgb_geometry_models',same_classes=True,same_target_points=True,
            independent_baseline=False,reference_model='PointTransformerV3',
            projected_disagreement=float(np.mean(labels[indices]!=arrays['shared_scannet_labels'][indices])))
        arrays['reference_ptv3_point_labels']=labels
        return result,display
