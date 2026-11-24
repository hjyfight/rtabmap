"""Frozen ScanNet PointTransformerV3 for real RGB-coloured LiDAR points.

Import only the author's inference modules. Training-only losses, hooks, LoRA
and unrelated models are not loaded. Every inference parameter loads strictly.
"""
import ast
import codecs
from contextlib import contextmanager
import hashlib
import importlib
from pathlib import Path
import runpy
import subprocess
import sys
import types

import numpy as np
import torch

ROOT=Path(__file__).resolve().parents[2]
EXPECTED_SHA256='40206376ff2f83f48e4d1bc27d5c5d96be7c87c5d11eb45fe7be501959040e7f'
LABELS=['wall','floor','cabinet','bed','chair','sofa','table','door','window','bookshelf',
        'picture','counter','desk','curtain','refrigerator','shower curtain','toilet','sink',
        'bathtub','otherfurniture']
CHINESE=['墙','地面','柜子','床','椅子','沙发','桌子','门','窗户','书架',
         '画','柜台','书桌','窗帘','冰箱','浴帘','马桶','洗手池','浴缸','其他家具']


@contextmanager
def checkpoint_safe_globals(allowed):
    """Use the same temporary allowlist on JetPack nv24.08 and stable torch."""
    serialization = torch.serialization
    if hasattr(serialization, 'safe_globals'):
        with serialization.safe_globals(allowed):
            yield
        return
    # NVIDIA's 2.5 preview has the registry API but predates its context manager.
    # Copy the registry before adding entries and restore it even if loading fails.
    previous = list(serialization.get_safe_globals())
    serialization.add_safe_globals(allowed)
    try:
        yield
    finally:
        serialization.clear_safe_globals()
        serialization.add_safe_globals(previous)


def package(name,directory):
    module=types.ModuleType(name)
    module.__path__=[str(directory)]
    sys.modules[name]=module
    return module


def inference_modules(vendor):
    base=vendor/'pointcept'
    package('pointcept',base)
    package('pointcept.models',base/'models')
    package('pointcept.utils',base/'utils')
    # Point utility __init__ has only inference helpers and can execute normally.
    utils=importlib.import_module('pointcept.models.utils')
    from pointcept.models.utils.structure import Point
    module=types.ModuleType('pointcept.models.modules')
    module.__file__=str(base/'models/modules.py')
    source=ast.parse(Path(module.__file__).read_text())
    selected=[node for node in source.body if isinstance(node,(ast.FunctionDef,ast.ClassDef))
              and node.name in ('is_ocnn_module','PointModule','PointSequential')]
    import spconv.pytorch as spconv
    from collections import OrderedDict
    module.__dict__.update(sys=sys,nn=torch.nn,spconv=spconv,OrderedDict=OrderedDict,Point=Point,ocnn=None)
    exec(compile(ast.Module(body=selected,type_ignores=[]),module.__file__,'exec'),module.__dict__)
    sys.modules[module.__name__]=module
    prompt=package('pointcept.models.point_prompt_training',base/'models/point_prompt_training')
    prompt.PDNorm=importlib.import_module('pointcept.models.point_prompt_training.prompt_driven_normalization').PDNorm
    if importlib.util.find_spec('timm.layers') is None:
        # Older timm exposes the identical inference DropPath under this namespace.
        sys.modules['timm.layers']=importlib.import_module('timm.models.layers')
    package('pointcept.models.point_transformer_v3',base/'models/point_transformer_v3')
    importlib.import_module('pointcept.models.point_transformer_v3.point_transformer_v3m1_base')
    from pointcept.models.builder import MODELS,build_model
    # The exact upstream segmentor class; criteria are unused for inference.
    default_path=base/'models/default.py'
    tree=ast.parse(default_path.read_text())
    segmentor=next(node for node in tree.body if isinstance(node,ast.ClassDef) and node.name=='DefaultSegmentorV2')
    scope=dict(torch=torch,nn=torch.nn,Point=Point,MODELS=MODELS,build_model=build_model,
               build_criteria=lambda config:None)
    exec(compile(ast.Module(body=[segmentor],type_ignores=[]),str(default_path),'exec'),scope)
    return scope['DefaultSegmentorV2']


def normals_from_lidar(points,k=24):
    from scipy.spatial import cKDTree
    xyz=np.asarray(points,np.float64)
    _,neighbors=cKDTree(xyz).query(xyz,k=min(k,len(xyz)),workers=2)
    patches=xyz[neighbors]
    centered=patches-patches.mean(axis=1,keepdims=True)
    covariance=np.einsum('nki,nkj->nij',centered,centered)/patches.shape[1]
    eigenvalues,vectors=np.linalg.eigh(covariance)
    normals=vectors[:,:,0]
    normals[np.einsum('ij,ij->i',normals,xyz)>0]*=-1
    return normals.astype(np.float32),eigenvalues[:,0]/np.maximum(eigenvalues.sum(axis=1),1e-12)


def coloured_points(rgb,points,matrix,intrinsic,distortion,registered_depth=None,depth_tolerance=.2):
    from .inputs import NotReady,project_pmf
    points=np.asarray(points,np.float32)
    finite=np.isfinite(points).all(1)&(np.linalg.norm(points[:,:3],axis=1)>.15)
    original_indices=np.flatnonzero(finite)
    points=points[finite]
    _,_,_,uv,selected=project_pmf(points,rgb,intrinsic,distortion,matrix,1/255)
    camera=points[selected,:3]@matrix[:3,:3].T+matrix[:3,3]
    projected_count=len(uv)
    depth_errors=None
    if registered_depth is not None:
        if registered_depth.shape!=rgb.shape[:2]:
            raise ValueError('Occlusion depth must be registered to this RGB image')
        depth=registered_depth.astype(np.float32)
        if registered_depth.dtype==np.uint16:
            depth/=1000
        observed=depth[uv[:,1],uv[:,0]]
        depth_errors=np.abs(observed-camera[:,2])
        keep=np.isfinite(observed)&(observed>.15)&(observed<10)&(depth_errors<=depth_tolerance)
        uv,selected,camera=uv[keep],selected[keep],camera[keep]
        depth_errors=depth_errors[keep]
    if len(selected)<32:
        raise NotReady('与RGB深度表面一致的LiDAR点不足32个；等待更好的视野和配准')
    normals,curvature=normals_from_lidar(points[:,:3])
    return dict(xyz=points[selected,:3],rgb=rgb[uv[:,1],uv[:,0]],normals=normals[selected],
                curvature=curvature[selected],uv=uv,source_indices=original_indices[selected],
                camera_xyz=camera,projected_before_filter=projected_count,
                depth_errors=depth_errors,depth_tolerance_m=depth_tolerance)


class PointTransformer:
    def __init__(self,weights,device='cuda:0'):
        vendor=ROOT/'.tools/pointcept'
        segmentor=inference_modules(vendor)
        config=runpy.run_path(str(vendor/'configs/scannet/semseg-pt-v3m1-0-base.py'))['model']
        backbone=dict(config['backbone'])
        # Author's native PyTorch attention path; no new parameters or surrogate layers.
        backbone.update(enable_flash=False,upcast_attention=True,upcast_softmax=True)
        self.net=segmentor(config['num_classes'],config['backbone_out_channels'],backbone=backbone)
        digest=hashlib.sha256()
        with Path(weights).open('rb') as stream:
            for block in iter(lambda:stream.read(1024*1024),b''):
                digest.update(block)
        if digest.hexdigest()!=EXPECTED_SHA256:
            raise ValueError('Pointcept official checkpoint SHA256 mismatch')
        # This official checkpoint includes optimizer/scheduler metadata. Permit
        # only its known metadata types while keeping weights-only deserialization.
        with checkpoint_safe_globals([getattr,codecs.encode,
                torch.optim.lr_scheduler.OneCycleLR,np.core.multiarray.scalar,np.dtype,
                type(np.dtype('float64')),type(np.dtype('float32'))]):
            checkpoint=torch.load(weights,map_location='cpu',weights_only=True)
        state=checkpoint['state_dict']
        state={key.removeprefix('module.'):value for key,value in state.items()}
        self.net.load_state_dict(state,strict=True)
        self.net.eval().requires_grad_(False).to(device)
        self.device=device
        self.metadata=dict(name='PointTransformerV3',dataset='ScanNet20',classes=20,strict=True,
                           tensors=len(state),frozen=True,backend='torch · native attention',
                           backend_kind='rgb_coloured_lidar',
                           repository='https://github.com/Pointcept/Pointcept',
                           commit=subprocess.check_output(['git','-C',str(vendor),'rev-parse','HEAD'],text=True).strip(),
                           file=Path(weights).name,sha256=digest.hexdigest(),
                           input='RGB color + LiDAR normals, XYZ positions',
                           sampling='one-pass 2cm voxel sampling; no benchmark TTA')

    @torch.inference_mode()
    def infer_points(self,xyz,rgb,normals):
        xyz=np.asarray(xyz,np.float32).copy()
        rgb=np.asarray(rgb,np.float32)
        normals=np.asarray(normals,np.float32)
        if xyz.shape[0]<32 or xyz.shape!=rgb.shape or xyz.shape!=normals.shape or xyz.shape[1]!=3:
            raise ValueError('At least 32 real coloured LiDAR points with normals are required')
        if not all(np.isfinite(a).all() for a in (xyz,rgb,normals)):
            raise ValueError('Nonfinite point inputs')
        minimum,maximum=xyz.min(0),xyz.max(0)
        xyz-=np.array([(minimum[0]+maximum[0])/2,(minimum[1]+maximum[1])/2,minimum[2]],np.float32)
        grid=np.floor((xyz-xyz.min(0))/.02).astype(np.int64)
        _,selected,inverse=np.unique(grid,axis=0,return_index=True,return_inverse=True)
        features=np.concatenate([rgb/127.5-1,normals],axis=1).astype(np.float32)
        sample=dict(coord=torch.from_numpy(xyz[selected]).to(self.device),
                    grid_coord=torch.from_numpy(grid[selected]).to(self.device),
                    feat=torch.from_numpy(features[selected]).to(self.device),
                    offset=torch.tensor([len(selected)],device=self.device))
        prediction=self.net(sample)
        logits=prediction['seg_logits']
        if logits.shape!=(len(selected),20) or not torch.isfinite(logits).all():
            raise ValueError('Invalid PTv3 output')
        labels=logits.argmax(1).cpu().numpy()[inverse].astype(np.uint8)
        return labels,dict(voxels=len(selected),points=len(xyz),features=features,
                           input_xyz=xyz,grid_coord=grid,selected_indices=selected,inverse=inverse,
                           voxel_features=features[selected])
