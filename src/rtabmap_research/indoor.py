"""Indoor coloured-LiDAR candidate and an explicit original PMF comparison."""
import numpy as np
import torch

from .models import ROOT,PMF,Trace,predictions,annotated_predictions,jpeg
from .inputs import project_pmf
from .point_scene import build_scene
from .ptv3 import PointTransformer,CHINESE


class IndoorComparison:
    def __init__(self,weights,device):
        self.core=PointTransformer(weights,device)
        self.reference=PMF(ROOT/'models/pmf_semantickitti.download',device)
        self.metadata=dict(self.core.metadata,comparison_model=self.reference.metadata)
        self.trace=Trace()
        self.recording=True
        def inputs(_,rows):
            if not self.recording:return
            x=rows[0].feat
            self.trace.record('inputs.rgb',x[:,:3].unsqueeze(0),'投影RGB颜色，按作者规范归一化')
            self.trace.record('inputs.lidar_normal',x[:,3:].unsqueeze(0),'真实LiDAR表面法向')
            self.trace.record('inputs.concat',x.unsqueeze(0),'cat([RGB, LiDAR normal], dim=1)；XYZ用于空间位置')
        self.trace.handles.append(self.core.net.backbone.embedding.register_forward_pre_hook(inputs))
        def stage(name):
            def receive(_,args,out):
                if not self.recording:return
                self.trace.record(name,out.feat.unsqueeze(0),'原作者稀疏编码与串行点注意力')
            return receive
        self.trace.handles.append(self.core.net.backbone.embedding.register_forward_hook(stage('point.embedding')))
        for name,module in self.core.net.backbone.enc.named_children():
            self.trace.handles.append(module.register_forward_hook(stage('point.'+name)))
        self.trace.handles.append(self.core.net.seg_head.register_forward_pre_hook(
            lambda _,rows:self.trace.record('point.decoded',rows[0].unsqueeze(0),'原作者解码特征') if self.recording else None))
        self.trace.handles.append(self.core.net.seg_head.register_forward_hook(
            lambda _,rows,out:self.trace.record('point.logits',out.unsqueeze(0),'20类室内任务分类头') if self.recording else None))

    def compare_scene(self,scene):
        self.trace.clear()
        # Use the same serialization RNG for both inputs; isolate it from other models.
        with torch.random.fork_rng(devices=[]):
            torch.random.default_generator.manual_seed(123)
            labels,stats=self.core.infer_points(scene['xyz'],scene['rgb'],scene['normals'])
        self.recording=False
        try:
            with torch.random.fork_rng(devices=[]):
                torch.random.default_generator.manual_seed(123)
                ablated,_=self.core.infer_points(scene['xyz'],np.full_like(scene['rgb'],127.5,dtype=np.float32),scene['normals'])
        finally:self.recording=True
        self.trace.arrays.update(voxel_indices=stats['selected_indices'],voxel_inverse=stats['inverse'],
            point_input_xyz=stats['input_xyz'],point_grid_coord=stats['grid_coord'],
            no_rgb_point_labels=ablated,point_labels=labels)
        return labels,ablated,stats

    def infer_bundle(self,bundle):
        import cv2
        scene=build_scene(bundle['observations'],bundle['matrix'])
        labels,ablated,stats=self.compare_scene(scene)
        rgb=scene['current_rgb'];uv=scene['uv'];indices=scene['render_indices']
        display=np.zeros(rgb.shape[:2],np.uint8)
        display[uv[:,1],uv[:,0]]=labels[indices]+1
        mask=display>0
        names=['忽略']+CHINESE
        pictures,classes=predictions(rgb,display,names,mask)
        annotations=annotated_predictions(pictures,rgb,display,names,mask,task='indoor')
        ablated_display=np.zeros_like(display)
        ablated_display[uv[:,1],uv[:,0]]=ablated[indices]+1
        ablated_pictures,_=predictions(rgb,ablated_display,names,mask)
        annotated_predictions(ablated_pictures,rgb,ablated_display,names,mask,task='indoor_ablation')
        pictures['no_rgb_overlay']=ablated_pictures['annotated_overlay']
        pictures['no_rgb_plain']=ablated_pictures['overlay']
        projection=rgb.copy()
        for u,v in uv[::max(1,len(uv)//3000)]:cv2.circle(projection,(int(u),int(v)),1,(66,235,198),-1)
        pictures['projection']=jpeg(projection)
        latest=bundle['observations'][-1]
        sparse,image,reference_mask,reference_uv,_=project_pmf(latest['points'],rgb,latest['k'],
            latest['distortion'],bundle['matrix'],bundle['calibration']['intensity_scale'])
        reference,reference_labels=self.reference.infer(rgb,sparse,image,reference_mask,reference_uv)
        pictures['reference_overlay']=reference['images']['annotated_overlay']
        pictures['reference_plain']=reference['images']['overlay']
        self.trace.arrays.update(point_labels=labels,display_labels=display,
            scene_xyz=scene['xyz'],scene_rgb=scene['rgb'],scene_normals=scene['normals'],
            point_source_indices=scene['source_indices'],render_indices=indices,projection_uv=uv,
            camera_from_world=scene['camera_from_world'],camera_from_lidar=bundle['matrix'],
            observation_poses=np.stack([o['world_from_lidar'] for o in bundle['observations']]),
            observation_stamps=np.asarray([[o['rgb_stamp'],o['lidar_stamp']] for o in bundle['observations']]),
            source_rgb_frames=np.stack([o['rgb'] for o in bundle['observations']]),
            source_depth_frames=np.stack([o['depth'] for o in bundle['observations']]),
            source_depth_types=np.asarray([str(o['depth'].dtype) for o in bundle['observations']]),
            source_rgb_intrinsics=np.asarray([o['k'] for o in bundle['observations']]),
            source_rgb_distortion=np.asarray([o['distortion'] for o in bundle['observations']]),
            current_rgb=rgb,reference_pmf_labels=reference_labels,no_rgb_display_labels=ablated_display)
        for number,observation in enumerate(bundle['observations']):
            self.trace.arrays[f'source_lidar_{number}']=observation['points']
        window=bundle['observations'][-1]['rgb_stamp']-bundle['observations'][0]['rgb_stamp']
        return dict(mode='indoor_rgb_lidar',images=pictures,classes=classes,annotations=annotations,
                    features=list(self.trace.rows),output_shape=[stats['voxels'],20],projected_points=len(uv),
                    scene_points=len(labels),observation_count=len(bundle['observations']),window_s=window,
                    fusion_quality=dict(projection_coverage=float(mask.mean()),valid_lidar_points=len(uv)),
                    comparison=dict(kind='rgb_input_ablation',same_geometry=True,same_classes=True,
                        rgb_value=127.5,serialization_seed=123,independent_baseline=False,
                        projected_disagreement=float(np.mean(labels[indices]!=ablated[indices])),
                        scene_disagreement=float(np.mean(labels!=ablated))),
                    baseline_note='PTv3输入为RGB颜色与LiDAR几何。PMF为道路类别视觉对照；类别空间不同，不能比较占比作为精度。'),display
