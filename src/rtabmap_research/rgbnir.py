"""Parameter-free RGB/NIR fusion before the original YOLOv8 segmentation neck.

The shared frozen backbone runs separately on RGB and registered mono NIR.
RGB backbone tensors stay pure; only the neck's backbone inputs are replaced.
"""
import hashlib
from pathlib import Path
import sys

import cv2
import numpy as np
import torch
import torch.nn.functional as F

from .models import ROOT,Trace,jpeg
from .inputs import NotReady
from .annotations import annotate,font


def letterbox(rgb,nir,valid,size):
    h,w=rgb.shape[:2]
    scale=min(size/h,size/w)
    nw,nh=round(w*scale),round(h*scale)
    dw,dh=(size-nw)/2,(size-nh)/2
    left,right=round(dw-.1),round(dw+.1)
    top,bottom=round(dh-.1),round(dh+.1)
    def prepare(image,padding):
        image=cv2.resize(image,(nw,nh),interpolation=cv2.INTER_LINEAR)
        return cv2.copyMakeBorder(image,top,bottom,left,right,cv2.BORDER_CONSTANT,value=padding)
    rgb=prepare(rgb,(114,114,114))
    nir=prepare(nir,114)
    mask=prepare(valid.astype(np.float32),0)
    return rgb,nir,mask,dict(scale=scale,pad_xy=[left,top],input_size=[size,size])


class RGBNIR:
    def __init__(self,weights,device='cuda:0',size=640,confidence=.35,nir_weight=.1):
        if not 0 <= nir_weight <= 1 or size < 32 or size % 32:
            raise ValueError('NIR weight must be in [0,1]; image size must be a positive multiple of 32')
        from ultralytics import YOLO
        self.yolo=YOLO(str(weights))
        if self.yolo.task!='segment' or 'v8' not in Path(weights).name:
            raise ValueError('Use original YOLOv8 COCO segmentation weights')
        self.net=self.yolo.model.eval().requires_grad_(False).to(device)
        self.device,self.size,self.confidence=device,size,confidence
        self.nir_weight=nir_weight
        self.trace=Trace()
        self.end=len(self.net.yaml['backbone'])
        self.levels=sorted({self.end-1}|{i for module in self.net.model[self.end:] for i in
             ([module.f] if isinstance(module.f,int) else module.f) if 0<=i<self.end})
        if self.levels!=[4,6,9]:raise ValueError('Unexpected YOLOv8 backbone/neck interface')
        self.phase=None;self.nir_features={};self.fused_features={};self.stats=[]
        self.valid=None
        for number,index in enumerate(self.levels):
            def capture(module,args,rgb_feature,index=index,number=number):
                if self.phase!='rgb':return
                nir_feature=self.nir_features[index]
                if rgb_feature.shape!=nir_feature.shape:raise ValueError('RGB/NIR feature shape mismatch')
                valid=(F.interpolate(self.valid,size=rgb_feature.shape[-2:],mode='area')>=.95).float()
                count=valid.sum()*rgb_feature.shape[1]
                if count<1:raise NotReady('当前尺度没有足够的有效NIR配准区域')
                rgb_rms=((rgb_feature.square()*valid).sum()/count).sqrt()
                nir_rms=((nir_feature.square()*valid).sum()/count).sqrt()
                gain=(rgb_rms/(nir_rms+1e-8)).clamp(.25,4)
                scaled=nir_feature*gain
                alpha=valid*self.nir_weight
                fused=(1-alpha)*rgb_feature+alpha*scaled
                self.fused_features[index]=fused
                for suffix,value,operation in [
                    ('rgb',rgb_feature,'纯RGB共享骨干输出'),
                    ('nir',nir_feature,'配准NIR复制3通道，共享同一冻结骨干'),
                    ('nir_scaled',scaled,'无训练参数的RMS尺度匹配'),
                    ('fused',fused,'有效区域0.9 RGB + 0.1 NIR；无效区域RGB')]:
                    self.trace.record(f'P{number+3}.{suffix}',value,operation)
                self.trace.arrays[f'P{number+3}_nir_weight']=alpha.detach().cpu().numpy()
                self.stats.append(dict(level=f'P{number+3}',shape=list(fused.shape),
                    rgb_rms=float(rgb_rms),nir_rms=float(nir_rms),nir_rms_gain=float(gain),
                    valid_fraction=float(valid.mean()),rgb_weight=1-self.nir_weight,nir_weight=self.nir_weight))
                # Return nothing: later RGB backbone stages must remain pure RGB.
            self.trace.handles.append(self.net.model[index].register_forward_hook(capture))
        for module in self.net.model[self.end:]:
            def neck_inputs(layer,args):
                if self.phase!='rgb':return
                value=args[0]
                if isinstance(layer.f,int):
                    index=self.end-1 if layer.i==self.end and layer.f==-1 else layer.f
                    if index in self.fused_features:return (self.fused_features[index],)
                else:
                    value=list(value)
                    for j,index in enumerate(layer.f):
                        if index==-1 and layer.i==self.end:index=self.end-1
                        if index in self.fused_features:value[j]=self.fused_features[index]
                    return (value,)
            self.trace.handles.append(module.register_forward_pre_hook(neck_inputs))
        state=self.net.state_dict()
        origin=(self.yolo.ckpt.get('ema') or self.yolo.ckpt['model']).state_dict()
        if set(state)!=set(origin) or any(not torch.equal(state[k].cpu(),origin[k].float().cpu()) for k in state):
            raise ValueError('YOLOv8 full checkpoint changed during loading')
        digest=hashlib.sha256()
        with Path(weights).open('rb') as stream:
            for block in iter(lambda:stream.read(1024*1024),b''):digest.update(block)
        self.metadata=dict(name='YOLOv8-seg RGB/NIR',dataset='COCO80',classes=len(self.yolo.names),
            file=Path(weights).name,sha256=digest.hexdigest(),strict=True,tensors=len(state),frozen=True,
            backend='torch',backend_kind='rgb_nir_fixed_features',extra_trainable_parameters=0,
            nominal_weights=[1-nir_weight,nir_weight],feature_levels=['P3','P4','P5'],
            fusion='shared backbone, RMS scale matching, fixed weighted features into original neck/head',
            nir_domain_trained=False)

    @torch.inference_mode()
    def forward_tensors(self,rgb,nir,valid,nir_weight=None):
        self.trace.clear();self.stats=[];self.fused_features={};self.nir_features={}
        self.valid=valid
        previous_weight=self.nir_weight
        if nir_weight is not None:self.nir_weight=nir_weight
        try:
            self.phase='nir'
            x=nir;y=[]
            for module in self.net.model[:self.end]:
                if module.f!=-1:
                    x=y[module.f] if isinstance(module.f,int) else [x if i==-1 else y[i] for i in module.f]
                x=module(x)
                y.append(x if module.i in self.net.save else None)
                if module.i in self.levels:self.nir_features[module.i]=x
            self.phase='rgb'
            return self.net(rgb)
        finally:self.phase=None;self.nir_weight=previous_weight

    def _tensor_inputs(self,rgb,nir,valid):
        rgb,nir,valid,info=letterbox(rgb,nir,valid,self.size)
        rgb_tensor=torch.from_numpy(np.ascontiguousarray(rgb.transpose(2,0,1)[None])).to(self.device).float()/255
        nir_tensor=torch.from_numpy(np.repeat(nir[None,None],3,axis=1).copy()).to(self.device).float()/255
        mask=torch.from_numpy(valid[None,None].copy()).to(self.device)
        return rgb_tensor,nir_tensor,mask,info

    @staticmethod
    def prediction_parts(prediction):
        # Original v8 returns ((detections, proto), (levels, coefficients, proto));
        # newer Ultralytics returns (detections, a dictionary including proto).
        if isinstance(prediction[0],tuple):return prediction[0]
        return prediction[0],prediction[1]['proto'] if isinstance(prediction[1],dict) else prediction[1][-1]

    def decode(self,prediction,shape):
        from ultralytics.utils import ops
        try:from ultralytics.utils.nms import non_max_suppression
        except ImportError:non_max_suppression=ops.non_max_suppression
        detection,prototypes=self.prediction_parts(prediction)
        pred=non_max_suppression(detection,self.confidence,.5,nc=len(self.yolo.names),max_det=30)[0]
        proto=prototypes[0]
        if not len(pred):return []
        pred[:,:4]=ops.scale_boxes((self.size,self.size),pred[:,:4],shape)
        masks=ops.process_mask_native(proto,pred[:,6:],pred[:,:4],shape).cpu().numpy()
        return [dict(class_id=int(row[5]),label=str(self.yolo.names[int(row[5])]),score=float(row[4]),
                     bbox_xyxy=row[:4].cpu().numpy().tolist(),mask=np.asarray(mask,bool))
                for row,mask in zip(pred,masks)]

    def picture(self,rgb,detections,title):
        from PIL import Image,ImageDraw
        sys.path.insert(0,str(ROOT/'src/rtabmap_semantic'))
        from rtabmap_semantic.labels import chinese_label
        from rtabmap_semantic.segmentation import color_for
        image=rgb.copy();identities=np.zeros(rgb.shape[:2],np.uint8)
        names=['背景'];colors=[(0,0,0)]
        for number,row in enumerate(detections,1):
            mask=row['mask']&(identities==0)
            identities[mask]=number
            color=color_for(row['class_id'])
            image[mask]=(.55*rgb[mask]+.45*np.asarray(color)).astype(np.uint8)
            names.append(chinese_label(row['label']));colors.append(color)
        annotated,regions=annotate(image,identities,names,np.asarray(colors,np.uint8),task='rgbnir')
        picture=Image.fromarray(annotated);draw=ImageDraw.Draw(picture)
        draw.rectangle((0,0,rgb.shape[1],23),fill=(12,24,35))
        draw.text((7,1),title,font=font(14),fill=(232,244,251))
        for row in regions:
            original=detections[row['id']-1]
            row.update(instance_id=row['id'],class_id=original['class_id'],score=original['score'])
        return np.asarray(picture),regions,image

    @torch.inference_mode()
    def infer(self,rgb,nir,valid):
        if nir.dtype!=np.uint8 or nir.shape!=rgb.shape[:2] or valid.shape!=nir.shape:
            raise ValueError('NIR must be mono8 registered to the RGB pixel grid')
        if valid.mean()<.35:raise NotReady('NIR有效配准覆盖不足35%')
        rgb_t,nir_t,mask,preprocessing=self._tensor_inputs(rgb,nir,valid)
        self.phase=None
        baseline=self.net(rgb_t)
        baseline_detections=self.decode(baseline,rgb.shape[:2])
        prediction=self.forward_tensors(rgb_t,nir_t,mask)
        detections=self.decode(prediction,rgb.shape[:2])
        image,annotations,plain=self.picture(rgb,detections,'RGB 90% + NIR 10% · YOLOv8实例预测')
        reference,baseline_annotations,reference_plain=self.picture(rgb,baseline_detections,'同帧 RGB基线 · 原YOLOv8')
        counts={}
        for row in detections:counts[row['class_id']]=counts.get(row['class_id'],0)+int(row['mask'].sum())
        from rtabmap_semantic.labels import chinese_label
        from rtabmap_semantic.segmentation import color_for
        total=max(1,sum(counts.values()))
        classes=[dict(id=i,label=self.yolo.names[i],display_label=chinese_label(self.yolo.names[i]),
                      count=n,fraction=n/total,color=list(color_for(i))) for i,n in sorted(counts.items(),key=lambda r:-r[1])]
        self.trace.arrays.update(input_rgb=rgb,input_registered_nir=nir,nir_valid=valid,
            instance_masks=np.asarray([r['mask'] for r in detections],np.uint8).reshape(-1,*rgb.shape[:2]),
            boxes=np.asarray([r['bbox_xyxy'] for r in detections],np.float32).reshape(-1,4),
            class_ids=np.asarray([r['class_id'] for r in detections],np.int32),
            scores=np.asarray([r['score'] for r in detections],np.float32),
            baseline_instance_masks=np.asarray([r['mask'] for r in baseline_detections],np.uint8).reshape(-1,*rgb.shape[:2]),
            baseline_boxes=np.asarray([r['bbox_xyxy'] for r in baseline_detections],np.float32).reshape(-1,4),
            baseline_class_ids=np.asarray([r['class_id'] for r in baseline_detections],np.int32),
            baseline_scores=np.asarray([r['score'] for r in baseline_detections],np.float32))
        return dict(mode='rgb_nir_feature_fusion',images=dict(raw=jpeg(rgb),overlay=jpeg(plain),
            annotated_overlay=jpeg(image),reference_overlay=jpeg(reference),reference_plain=jpeg(reference_plain),
            projection=jpeg(np.repeat(nir[:,:,None],3,axis=2))),classes=classes,annotations=annotations,
            features=list(self.trace.rows),output_shape=list(self.prediction_parts(prediction)[0].shape),
            rgb_weight=1-self.nir_weight,nir_weight=self.nir_weight,feature_statistics=self.stats,
            valid_coverage=float(valid.mean()),detection_count=len(detections),baseline_count=len(baseline_detections),
            baseline_annotations=baseline_annotations,
            preprocessing=preprocessing,
            baseline_note='共享RGB预训练骨干的90/10固定特征实验，无新增训练参数；COCO80实例类别，融合精度尚未验证'),detections
