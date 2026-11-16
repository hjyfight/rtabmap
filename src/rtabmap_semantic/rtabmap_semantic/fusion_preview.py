"""Full native-vocabulary displays, separate from fixed-task evaluation maps."""
from copy import deepcopy
import cv2
import numpy as np
from .fusion_geometry import project_labels
from .labels import chinese_label
from .segmentation import dense_maps


def mask_anchor(mask):
    ys,xs=np.nonzero(mask)
    if not len(xs):return None
    left,top,right,bottom=int(xs.min()),int(ys.min()),int(xs.max())+1,int(ys.max())+1
    region=mask[top:bottom,left:right].astype(np.uint8)
    distance=cv2.distanceTransform(np.pad(region,1),cv2.DIST_L2,3)[1:-1,1:-1]
    y,x=np.unravel_index(np.argmax(distance),region.shape)
    return int(left+x),int(top+y)


def full_mask(row,shape):
    result=np.zeros(shape,bool);x,y=row['mask_offset'];height,width=row['mask'].shape
    result[y:y+height,x:x+width]=row['mask'].astype(bool)
    return result


def instance_labels(rows,shape,identities,project=None):
    labels=[]
    for row in rows:
        mask=full_mask(row,shape)
        if project is not None:
            u,v,valid=project
            mask=project_labels(mask.astype(np.int32),u,v,valid)==1
        visible=mask&(identities==row['instance_id'])
        anchor=mask_anchor(visible if visible.any() else mask)
        if anchor is None:continue
        labels.append(dict(instance_id=row['instance_id'],class_id=row['class_id'],
            label=chinese_label(row['label']),x=anchor[0],y=anchor[1],
            pixels=int(mask.sum()),visible_pixels=int(visible.sum())))
    return labels


def build_preview(bundle,result,refinements):
    """Keep every original RGB instance; apply only actual eligible refinements."""
    rgb_rows=bundle['rgb_instances'];shape=bundle['rgb'].shape[:2]
    rgb_map,rgb_ids=dense_maps(rgb_rows,shape)
    fused_rows=deepcopy(rgb_rows)
    for row in fused_rows:
        native=refinements.get(row['instance_id'])
        if native is not None:
            row['class_id']=native;row['label']=bundle['native_names'][native]
    fused_map,fused_ids=dense_maps(fused_rows,shape)
    native_nir,nir_ids=dense_maps(bundle['nir_instances'],bundle['nir'].shape)
    project=(result['projection_u'],result['projection_v'],result['eval_mask'])
    nir_map=project_labels(native_nir,*project)
    projected_ids=project_labels(nir_ids,*project)
    image_labels=dict(rgb=instance_labels(rgb_rows,shape,rgb_ids),
        nir=instance_labels(bundle['nir_instances'],bundle['nir'].shape,projected_ids,project),
        fused=instance_labels(fused_rows,shape,fused_ids))
    return dict(maps=dict(rgb=rgb_map,nir=nir_map,fused=fused_map),image_labels=image_labels,
        instance_counts={name:len(rows) for name,rows in image_labels.items()})
