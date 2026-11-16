"""Real, frozen UniBind RGB/NIR object-feature fusion and explicit task mapping."""
import hashlib
import json
import time
from pathlib import Path
import numpy as np
from .fusion_geometry import projection,project_image,project_labels,masked_square_crop
from .segmentation import dense_maps


def digest(value):
    return hashlib.sha256(json.dumps(value,sort_keys=True,allow_nan=False).encode()).hexdigest()


def normalized(values):
    values=np.asarray(values,np.float32)
    norms=np.linalg.norm(values,axis=-1,keepdims=True)
    if not np.isfinite(values).all() or (norms<1e-8).any():raise ValueError('Invalid embedding')
    return values/norms


class TaskVocabulary:
    def __init__(self,path):
        self.spec=json.loads(Path(path).read_text())
        self.names=[r['name'] for r in self.spec['classes']]
        self.aliases={}
        for i,row in enumerate(self.spec['classes']):
            for name in [row['name'],*row['aliases']]:
                key=name.strip().lower()
                if key in self.aliases:raise ValueError('Duplicate task alias: '+key)
                self.aliases[key]=i
        self.sha256=digest(self.spec)

    def task_id(self,name):return self.aliases.get(name.strip().lower(),-1)

    def label_map(self,instances,shape):
        rows=[{**r,'class_id':self.task_id(r['label'])} for r in instances]
        return dense_maps(rows,shape)[0]

    def native_ids(self,names):
        result={}
        for native,name in sorted(names.items()):
            task=self.task_id(name)
            if task>=0 and (task not in result or name==self.names[task]):result[task]=int(native)
        return result


class ObjectFusionEngine:
    def __init__(self,encoder,task,cache_dir,max_objects=8,min_coverage=.35):
        self.encoder,self.task=encoder,task
        self.max_objects,self.min_coverage=max_objects,min_coverage
        self.policy=dict(version='unibind_rgb_nir_roi_v1',task_sha256=task.sha256,
            backbone_sha256=encoder.provenance['checkpoint_sha256'],crop='masked_square_pad127',
            max_objects=max_objects,min_coverage=min_coverage,weights='rgb1_nir_coverage_normalized',
            min_pixels=64,min_nir_std=2.,softmax_temperature=.07,prototype_margin=.005,
            templates=task.spec['templates'])
        self.policy_sha256=digest(self.policy)
        path=Path(cache_dir)/(self.policy_sha256+'.npz')
        if path.is_file():
            with np.load(path,allow_pickle=False) as cached:self.centres=normalized(cached['centres'])
            if self.centres.shape!=(len(task.names),1024):raise ValueError('Invalid task prototype cache')
        else:
            texts=[template.format(name) for name in task.names for template in task.spec['templates']]
            embeddings=normalized(encoder.encode_text(texts))
            self.centres=normalized(embeddings.reshape(len(task.names),-1,1024).mean(axis=1))
            path.parent.mkdir(parents=True,exist_ok=True)
            np.savez_compressed(path,centres=self.centres)
        self.policy['centres_sha256']=hashlib.sha256(self.centres.tobytes()).hexdigest()
        self.policy_sha256=digest(self.policy)

    def fuse(self,bundle):
        started=time.monotonic()
        rgb,nir=bundle['rgb'],bundle['nir']
        shape=rgb.shape[:2]
        u,v,valid=projection(bundle['depth_m'],bundle['rgb_info'],bundle['nir_info'],bundle['transform'])
        warped=project_image(nir,u,v)
        rgb_map=self.task.label_map(bundle['rgb_instances'],shape)
        nir_native=self.task.label_map(bundle['nir_instances'],nir.shape)
        nir_map=project_labels(nir_native,u,v,valid)
        fused_map=rgb_map.copy()
        native_ids=self.task.native_ids(bundle['native_names'])
        selected=[];images=[];flags=[]
        identities=dense_maps(bundle['rgb_instances'],shape)[1]
        for row in sorted(bundle['rgb_instances'],key=lambda r:-int(np.count_nonzero(r['mask']))):
            x,y=row['mask_offset'];h,w=row['mask'].shape
            mask=np.zeros(shape,bool);mask[y:y+h,x:x+w]=row['mask'].astype(bool)
            count=int(mask.sum());coverage=float((mask&valid).sum()/max(count,1))
            if count<64 or coverage<self.min_coverage:continue
            pixels=warped[mask&valid]
            if not len(pixels) or float(pixels.std())<2:continue
            box=[x,y,x+w,y+h]
            images.extend([masked_square_crop(rgb,mask,box),masked_square_crop(warped,mask&valid,box)])
            flags.extend([False,True]);selected.append((row,coverage))
            if len(selected)>=self.max_objects:break
        encoding_started=time.monotonic()
        features=normalized(self.encoder.encode_image_crops(images,flags)).reshape(-1,2,1024) if images else np.empty((0,2,1024),np.float32)
        encoding_s=time.monotonic()-encoding_started
        object_results=[];fused_features=[];logits=[]
        for i,(row,coverage) in enumerate(selected):
            weights=np.asarray([1.,coverage],np.float32);weights/=weights.sum()
            fused=normalized((features[i]*weights[:,None]).sum(axis=0))
            scores=np.stack([features[i,0]@self.centres.T,features[i,1]@self.centres.T,fused@self.centres.T])
            predicted=int(np.argmax(scores[2]));ordered=np.sort(scores[2])
            eligible=predicted in native_ids and float(ordered[-1]-ordered[-2])>=.005
            probabilities=np.exp((scores[2]-scores[2].max())/.07);probabilities/=probabilities.sum()
            if eligible:fused_map[identities==row['instance_id']]=predicted
            object_results.append(dict(instance_id=row['instance_id'],baseline_task_id=self.task.task_id(row['label']),
                predicted_task_id=predicted,task_id=predicted if eligible else self.task.task_id(row['label']),
                native_class_id=native_ids[predicted] if eligible else row['class_id'],
                score=float(probabilities[predicted]),eligible=eligible,coverage=coverage,
                weights=[float(weights[0]),float(weights[1]),0.,0.],
                rgb_task_id=int(np.argmax(scores[0])),nir_task_id=int(np.argmax(scores[1])),
                bbox_xyxy=[float(x) for x in row['bbox_xyxy']]))
            fused_features.append(fused);logits.append(scores)
        return dict(rgb_map=rgb_map,nir_map=nir_map,fused_map=fused_map,eval_mask=valid,
            projection_u=u,projection_v=v,
            nir_warped=warped,nir_native_map=nir_native,objects=object_results,
            features=features,fused_features=np.asarray(fused_features,np.float32).reshape(-1,1024),
            logits=np.asarray(logits,np.float32).reshape(-1,3,len(self.task.names)),
            instance_map=identities,coverage=float(valid.mean()),policy=self.policy,policy_sha256=self.policy_sha256,
            timings_s=dict(geometry_and_crops=encoding_started-started,feature_encoding=encoding_s,
                classification=time.monotonic()-encoding_started-encoding_s))
