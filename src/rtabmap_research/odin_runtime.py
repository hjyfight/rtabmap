"""Author ODIN inference modules with native PyTorch / exact kNN reference ops.

No training entry points, language model, or unused Detectron2 detection operators
are imported. Network classes and all learned parameters remain upstream code.
"""
import ast
import importlib
from pathlib import Path
import sys
import types
import numpy as np
import torch
from scipy.spatial import cKDTree

ROOT=Path(__file__).resolve().parents[2]
ODIN_COMMIT='b8225c8879bb1e563fc85b5ea423ee950af8005a'
D2_COMMIT='247691a1c292654867deb35cabb27f18d75a1363'
VENDOR=ROOT/'.tools'/('odin-'+ODIN_COMMIT)
D2=ROOT/'.tools'/('detectron2-'+D2_COMMIT)/'detectron2'


def package(name,path):
    module=types.ModuleType(name);module.__path__=[str(path)]
    sys.modules[name]=module
    if '.' in name:
        parent,child=name.rsplit('.',1)
        if parent in sys.modules:setattr(sys.modules[parent],child,module)
    return module


def module_source(name,path,omit_imports=(),omit_names=(),replacements=()):
    module=types.ModuleType(name);module.__file__=str(path);module.__package__=name.rpartition('.')[0]
    sys.modules[name]=module
    source=path.read_text()
    for old,new in replacements:
        if source.count(old)!=1:raise ValueError('Author compatibility point changed: '+old)
        source=source.replace(old,new)
    parsed=ast.parse(source)
    kept=[]
    for node in parsed.body:
        if isinstance(node,ast.ImportFrom):
            if node.module in omit_imports:continue
            node.names=[item for item in node.names if item.name not in omit_names]
            if not node.names:continue
        kept.append(node)
    exec(compile(ast.Module(body=kept,type_ignores=[]),str(path),'exec'),module.__dict__)
    parent,child=name.rsplit('.',1)
    if parent in sys.modules:setattr(sys.modules[parent],child,module)
    return module


def knnquery(k,xyz,new_xyz,offset,new_offset):
    """Batch-separated Euclidean kNN; same distance convention as pointops2."""
    source=xyz.detach().cpu().numpy();target=new_xyz.detach().cpu().numpy()
    starts=[0]+offset.cpu().tolist();queries=[0]+new_offset.cpu().tolist()
    indices=[];distances=[]
    for a,b,x,y in zip(starts,starts[1:],queries,queries[1:]):
        if b-a<k:raise ValueError('Not enough unique geometry samples for author kNN')
        distance,index=cKDTree(source[a:b]).query(target[x:y],k=k,workers=2)
        indices.append(np.asarray(index).reshape(y-x,k)+a)
        distances.append(np.asarray(distance).reshape(y-x,k))
    return (torch.from_numpy(np.concatenate(indices)).to(xyz.device,dtype=torch.int64),
            torch.from_numpy(np.concatenate(distances)).to(xyz.device,dtype=xyz.dtype))


def queryandgroup(nsample,xyz,new_xyz,feat,idx,offset,new_offset,use_xyz=True,return_indx=False):
    if new_xyz is None:new_xyz=xyz
    if idx is None:idx,_=knnquery(nsample,xyz,new_xyz,offset,new_offset)
    grouped_xyz=xyz[idx]-new_xyz[:,None]
    grouped_feat=feat[idx]
    output=torch.cat([grouped_xyz,grouped_feat],-1) if use_xyz else grouped_feat
    return (output,idx) if return_indx else output


def interpolation(xyz,new_xyz,feat,offset,new_offset,k=3):
    idx,dist=knnquery(k,xyz,new_xyz,offset,new_offset)
    weight=1/(dist+1e-8);weight/=weight.sum(1,keepdim=True)
    # Match the author's ordered accumulation rather than reducing in a new order.
    result=torch.zeros((len(new_xyz),feat.shape[1]),device=feat.device,dtype=feat.dtype)
    for i in range(k):result+=feat[idx[:,i]]*weight[:,i,None]
    return result


def load_runtime(backbone_kind='resnet'):
    sys.path.insert(0,str(ROOT/'.tools/odin-research-deps'))
    from fvcore.common.registry import Registry
    debug=types.ModuleType('ipdb')
    def fail_debug(*args,**kwargs):raise RuntimeError('Author ODIN debug guard reached during inference')
    debug.set_trace=fail_debug;sys.modules['ipdb']=debug
    package('detectron2',D2);package('detectron2.utils',D2/'utils')
    cfg_package=package('detectron2.config',D2/'config')
    config=importlib.import_module('detectron2.config.config')
    for key in ('CfgNode','configurable','get_cfg','global_cfg'):setattr(cfg_package,key,getattr(config,key))
    layers=package('detectron2.layers',D2/'layers')
    for name,attrs in [('wrappers',['Conv2d','BatchNorm2d']),('batch_norm',['FrozenBatchNorm2d','get_norm']),
                       ('blocks',['CNNBlockBase']),('shape_spec',['ShapeSpec'])]:
        module=importlib.import_module('detectron2.layers.'+name)
        for attr in attrs:setattr(layers,attr,getattr(module,attr))
    modeling=package('detectron2.modeling',D2/'modeling')
    modeling.SEM_SEG_HEADS_REGISTRY=Registry('SEM_SEG_HEADS')
    backbone=package('detectron2.modeling.backbone',D2/'modeling/backbone')
    backbone.Backbone=importlib.import_module('detectron2.modeling.backbone.backbone').Backbone
    build=types.ModuleType('detectron2.modeling.backbone.build');build.BACKBONE_REGISTRY=Registry('BACKBONE')
    sys.modules[build.__name__]=build
    modeling.BACKBONE_REGISTRY=build.BACKBONE_REGISTRY
    modeling.Backbone=backbone.Backbone;modeling.ShapeSpec=layers.ShapeSpec
    package('libs',VENDOR/'libs');package('libs.pointops2',VENDOR/'libs/pointops2')
    package('libs.pointops2.functions',VENDOR/'libs/pointops2/functions')
    ops=types.ModuleType('libs.pointops2.functions.pointops')
    ops.knnquery=knnquery;ops.queryandgroup=queryandgroup;ops.interpolation=interpolation
    sys.modules[ops.__name__]=ops
    package('odin',VENDOR/'odin');package('odin.modeling',VENDOR/'odin/modeling')
    for name in ('backproject','transformer_decoder','meta_arch','pixel_decoder','backbone'):
        package('odin.modeling.'+name,VENDOR/'odin/modeling'/name)
    package('odin.data_video',VENDOR/'odin/data_video')
    module_source('odin.modeling.meta_arch.language_encoder',VENDOR/'odin/modeling/meta_arch/language_encoder.py',
                  omit_imports=('transformers',))
    func_name='odin.modeling.pixel_decoder.ops.functions.ms_deform_attn_func'
    package('odin.modeling.pixel_decoder.ops',VENDOR/'odin/modeling/pixel_decoder/ops')
    functions=package('odin.modeling.pixel_decoder.ops.functions',VENDOR/'odin/modeling/pixel_decoder/ops/functions')
    func=module_source(func_name,VENDOR/'odin/modeling/pixel_decoder/ops/functions/ms_deform_attn_func.py',
        replacements=[('raise ModuleNotFoundError(info_string)','MSDA = None')])
    functions.MSDeformAttnFunction=func.MSDeformAttnFunction
    # Upstream attention module already falls back to its grid_sample reference.
    resnet=module_source('odin.modeling.backbone.resnet',VENDOR/'odin/modeling/backbone/resnet.py',
        omit_names=('DeformConv','ModulatedDeformConv'))
    head=importlib.import_module('odin.modeling.meta_arch.odin_head')
    odin_cfg=importlib.import_module('odin.config')
    cfg=config.get_cfg();odin_cfg.add_maskformer2_config(cfg);odin_cfg.add_maskformer2_video_config(cfg)
    # Upstream base.yaml declares these legacy DeepLab fields as unused.
    cfg.MODEL.RESNETS.STEM_TYPE='basic';cfg.MODEL.RESNETS.RES5_MULTI_GRID=[1,1,1]
    config_name='swin_3d.yaml' if backbone_kind=='swin' else '3d.yaml'
    cfg.merge_from_file(str(VENDOR/'configs/scannet_context'/config_name))
    cfg.merge_from_list(['INPUT.IMAGE_SIZE','256','INPUT.VOXELIZE','True','MODEL.CROSS_VIEW_BACKBONE','True',
        'MODEL.PIXEL_DECODER_PANET','True','MODEL.SEM_SEG_HEAD.NUM_CLASSES','20',
        'MODEL.MASK_FORMER.TEST.SEMANTIC_ON','True','USE_GHOST_POINTS','True','USE_SEGMENTS','False',
        'USE_MLP_POSITIONAL_ENCODING','True','MODEL.FREEZE_BACKBONE','False','USE_WANDB','False'])
    assert not cfg.MODEL.OPEN_VOCAB and not any(cfg.MODEL.RESNETS.DEFORM_ON_PER_STAGE)
    backbone_builder=importlib.import_module('odin.modeling.backbone.swin').D2SwinTransformer if backbone_kind=='swin' else resnet.build_resnet_backbone_custom
    return cfg,backbone_builder,head.ODINHead,importlib.import_module('odin.modeling.backproject.backproject'),layers.ShapeSpec
