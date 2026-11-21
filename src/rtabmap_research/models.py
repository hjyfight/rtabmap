"""Author networks, complete checkpoints and actual intermediate tensor traces.

No training, surrogate encoders or random fallback weights. AVSegFormer's
author-provided grid_sample reference kernel is allowed for frozen inference.
"""
import base64
import hashlib
import importlib
import importlib.util
import json
from pathlib import Path
import runpy
import subprocess
import sys
import types
import numpy as np
import torch

ROOT = Path(__file__).resolve().parents[2]


def jpeg(rgb):
    import cv2
    ok, payload = cv2.imencode('.jpg', cv2.cvtColor(np.asarray(rgb, np.uint8), cv2.COLOR_RGB2BGR),
                              [cv2.IMWRITE_JPEG_QUALITY, 82])
    if not ok:
        raise ValueError('JPEG encoding failed')
    return base64.b64encode(payload).decode('ascii')


def palette(n):
    result = np.zeros((n, 3), np.uint8)
    for j in range(n):
        value, i = j, 0
        while value:
            for c in range(3):
                result[j, c] |= ((value >> c) & 1) << (7-i)
            value >>= 3
            i += 1
    return result


def predictions(rgb, labels, names, valid=None, background=0):
    from rtabmap_research.annotations import DISPLAY_LABELS
    colors = palette(len(names))[labels]
    valid = labels != background if valid is None else valid & (labels != background)
    overlay = rgb.copy()
    overlay[valid] = (.45 * rgb[valid] + .55 * colors[valid]).astype(np.uint8)
    colors[~valid] = 0
    counts = np.bincount(labels[valid].reshape(-1), minlength=len(names))
    total = max(1, int(valid.sum()))
    classes = [dict(id=int(i), label=names[i], display_label=DISPLAY_LABELS.get(names[i],names[i]),
                    count=int(counts[i]), fraction=float(counts[i]/total),
                    color=palette(len(names))[i].tolist())
               for i in np.argsort(-counts) if counts[i] > 0][:12]
    return dict(raw=jpeg(rgb), overlay=jpeg(overlay), classes=jpeg(colors)), classes


def annotated_predictions(pictures, rgb, labels, names, valid=None, task='pmf'):
    from rtabmap_research.annotations import annotate
    # Keep the existing semantic color overlay; annotation adds readable names and outlines.
    colors = palette(len(names))
    overlay = rgb.copy()
    mask = labels != 0 if valid is None else valid & (labels != 0)
    overlay[mask] = (.45*rgb[mask]+.55*colors[labels[mask]]).astype(np.uint8)
    image, annotations = annotate(overlay,labels,names,colors,valid,task)
    pictures['annotated_overlay'] = jpeg(image)
    return annotations


def load_state(model, path):
    path = Path(path)
    state = torch.load(path, map_location='cpu', weights_only=True)
    for key in ('state_dict', 'model', 'model_state_dict'):
        if isinstance(state, dict) and key in state and isinstance(state[key], dict):
            state = state[key]
            break
    if not isinstance(state, dict) or not state or not all(torch.is_tensor(v) for v in state.values()):
        raise ValueError('Checkpoint is not a complete tensor state_dict')
    if all(k.startswith('module.') for k in state):
        state = {k[7:]: v for k, v in state.items()}
    model.load_state_dict(state, strict=True)
    model.eval()
    model.requires_grad_(False)
    with path.open('rb') as stream:
        sha = hashlib.file_digest(stream, 'sha256').hexdigest() if hasattr(hashlib, 'file_digest') else None
    if sha is None:
        h = hashlib.sha256()
        with path.open('rb') as stream:
            for block in iter(lambda: stream.read(1024*1024), b''):
                h.update(block)
        sha = h.hexdigest()
    return dict(file=path.name, sha256=sha, strict=True, tensors=len(state), frozen=True)


class Trace:
    def __init__(self):
        self.rows, self.arrays, self.handles = [], {}, []

    def clear(self):
        self.rows, self.arrays = [], {}

    def record(self, name, tensor, operation):
        # Archive the last frame in a temporal batch, including its exact shape.
        x = tensor.detach()[-1:].float().cpu()
        if not torch.isfinite(x).all():
            raise ValueError(f'Nonfinite features: {name}')
        dims = tuple(range(2, x.ndim))
        vector = x.mean(dims).flatten() if dims else x.flatten()
        if x.ndim == 3:
            vector = x.mean(1).flatten()
        self.arrays[name.replace('.', '_')] = x.numpy()
        row = dict(name=name, shape=list(tensor.shape), saved_shape=list(x.shape), operation=operation,
                   l2=float(x.norm()), mean=float(x.mean()), std=float(x.std(unbiased=False)),
                   vector=vector.tolist()[:2048])
        if x.ndim == 4:
            energy = x[0].square().mean(0).sqrt().numpy()
            if energy.shape[1]==1 and energy.shape[0]>1024:
                # Point-index features have no image H/W. Keep the full tensor
                # archived; arrange only their energy preview into a compact grid.
                count=energy.size;width=int(np.ceil(np.sqrt(count)))
                padded=np.full(width*int(np.ceil(count/width)),float(energy.min()),np.float32)
                padded[:count]=energy.ravel();energy=padded.reshape(-1,width)
                row['heatmap_layout']='point_index_grid'
                row['operation']+='；能量图按点序排为网格，非图像投影'
            import cv2
            scaled = ((energy-energy.min()) / max(1e-8, float(np.ptp(energy))) * 255).astype(np.uint8)
            row['heatmap'] = jpeg(cv2.cvtColor(cv2.applyColorMap(scaled, cv2.COLORMAP_TURBO), cv2.COLOR_BGR2RGB))
        self.rows.append(row)

    def hook(self, module, name, operation):
        def receive(_, inputs, out):
            self.record(name, out, operation)
        self.handles.append(module.register_forward_hook(receive))


def package(name, directory):
    obj = types.ModuleType(name)
    obj.__path__ = [str(directory)]
    sys.modules[name] = obj
    return obj


class PMF:
    def __init__(self, weights, device):
        vendor = ROOT / '.tools/pmf'
        # Bypass training-only imports from pc_processor/__init__.py. The actual
        # model modules and all parameter names remain the author's originals.
        package('pc_processor', vendor/'pc_processor')
        package('pc_processor.models', vendor/'pc_processor/models')
        from pc_processor.models.pmf_net import PMFNet
        self.net = PMFNet(imagenet_pretrained=False)
        self.metadata = load_state(self.net, weights)
        self.metadata.update(name='PMF', repository='https://github.com/ICEORY/PMF',
                             classes=20, dataset='SemanticKITTI', backend='torch',
                             commit=subprocess.check_output(['git','-C',str(vendor),'rev-parse','HEAD'], text=True).strip())
        self.net.to(device)
        self.device, self.trace = device, Trace()
        import yaml
        labels = yaml.safe_load((vendor/'pc_processor/dataset/semantic_kitti/semantic-kitti.yaml').read_text())
        self.names = [labels['labels'][labels['learning_map_inv'][i]] for i in range(20)]
        for i in range(1, 5):
            block = getattr(self.net.lidar_stream, f'fusionblock_{i}')
            def before(_, inputs, level=i):
                self.trace.record(f'scale{level}.lidar', inputs[0], 'LiDAR stream')
                self.trace.record(f'scale{level}.rgb', inputs[1], 'RGB stream · matched H/W')
            self.trace.handles.append(block.register_forward_pre_hook(before))
            def cat(_, inputs, level=i):
                self.trace.record(f'scale{level}.concat', inputs[0], 'torch.cat([LiDAR, RGB], dim=1)')
            self.trace.handles.append(block.fuse_conv.register_forward_pre_hook(cat))
            self.trace.hook(block, f'scale{i}.fused', 'Conv3×3 → attention → LiDAR residual')

    @torch.inference_mode()
    def rgb_preview(self, rgb):
        self.trace.clear()
        h,w=rgb.shape[:2]
        image=np.zeros((3,(h+15)//16*16,(w+15)//16*16),np.float32)
        image[:,:h,:w]=rgb.transpose(2,0,1)/255.
        features=self.net.camera_stream_encoder(torch.from_numpy(image[None]).to(self.device))
        camera=self.net.camera_stream_decoder(features)
        if not torch.isfinite(camera).all():raise ValueError('Nonfinite PMF RGB branch output')
        for i,x in enumerate(features):self.trace.record(f'scale{i+1}.rgb',x,'冻结PMF RGB编码器 · LiDAR待标定')
        labels=camera[0,:,:h,:w].argmax(0).cpu().numpy().astype(np.uint8)
        pictures,classes=predictions(rgb,labels,self.names)
        annotations=annotated_predictions(pictures,rgb,labels,self.names,task='pmf_rgb')
        self.trace.arrays.update(rgb_branch_labels=labels,input_rgb=image)
        return dict(mode='rgb_only',images=pictures,classes=classes,annotations=annotations,features=list(self.trace.rows),
                    output_shape=list(camera.shape),projected_points=0,
                    baseline_note='当前仅运行PMF的RGB分支，未执行LiDAR拼接与融合；不等同于独立训练的单模态基线'),labels

    @torch.inference_mode()
    def infer(self, rgb, sparse, image, mask, uv):
        self.trace.clear()
        lidar, camera = self.net(torch.from_numpy(sparse[None]).to(self.device),
                                 torch.from_numpy(image[None]).to(self.device))
        if not torch.isfinite(lidar).all() or not torch.isfinite(camera).all():
            raise ValueError('PMF output contains nonfinite values')
        h, w = rgb.shape[:2]
        fused = lidar[0, :, :h, :w].argmax(0).cpu().numpy().astype(np.uint8)
        rgb_labels = camera[0, :, :h, :w].argmax(0).cpu().numpy().astype(np.uint8)
        pictures, classes = predictions(rgb, fused, self.names, mask[:h, :w])
        annotations = annotated_predictions(pictures,rgb,fused,self.names,mask[:h,:w],task='pmf')
        baseline, _ = predictions(rgb, rgb_labels, self.names)
        pictures['rgb_branch'] = baseline['overlay']
        comparison, _ = predictions(rgb,rgb_labels,self.names,mask[:h,:w])
        annotated_predictions(comparison,rgb,rgb_labels,self.names,mask[:h,:w],task='pmf_rgb')
        pictures['rgb_comparison'] = comparison['annotated_overlay']
        projection = rgb.copy()
        import cv2
        for u, v in uv[::max(1, len(uv)//3000)]:
            cv2.circle(projection, (int(u),int(v)), 1, (66,235,198), -1)
        pictures['projection'] = jpeg(projection)
        self.trace.arrays.update(fused_labels=fused, rgb_branch_labels=rgb_labels, projection_uv=uv,
                                 input_lidar=sparse, input_rgb=image, projection_mask=mask)
        return dict(mode='rgb_lidar_fusion',images=pictures, classes=classes,annotations=annotations,features=list(self.trace.rows),
                    output_shape=list(lidar.shape), projected_points=len(uv),
                    fusion_quality=dict(projection_coverage=float(mask[:h,:w].mean()),
                        rgb_branch_disagreement=float(np.mean(fused[mask[:h,:w]]!=rgb_labels[mask[:h,:w]])),
                        note='同帧、同一投影区域上的分支差异；不是准确率或融合增益'),
                    baseline_note='RGB分支是联合训练模型的分支，不等同于独立单模态基线'), fused


def avs_reference_ops(vendor):
    """Permit import without the optional extension; keep author's fallback."""
    package('ops', vendor/'ops')
    package('ops.functions', vendor/'ops/functions')
    name = 'ops.functions.ms_deform_attn_func'
    path = vendor/'ops/functions/ms_deform_attn_func.py'
    source = path.read_text()
    marker = 'raise ModuleNotFoundError(info_string)'
    if source.count(marker) != 1:
        raise ValueError('AVSegFormer reference compatibility point changed')
    module = types.ModuleType(name)
    module.__file__, module.__package__ = str(path), 'ops.functions'
    sys.modules[name] = module
    exec(compile(source.replace(marker, 'MSDA = None'), str(path), 'exec'), module.__dict__)
    sys.modules['ops.functions'].MSDeformAttnFunction = module.MSDeformAttnFunction
    return 'compiled_cuda' if module.MSDA is not None else 'author_pytorch_grid_sample'


class AVS:
    def __init__(self, weights, device):
        vendor = ROOT/'.tools/avsegformer'
        backend = avs_reference_ops(vendor)
        sys.path.insert(0, str(vendor))
        config = runpy.run_path(str(vendor/'config/avss/AVSegFormer_res50_avss.py'))['model']
        config['backbone']['init_weights_path'] = None
        config['vggish']['freeze_audio_extractor'] = False
        config['vggish']['preprocess_audio_to_log_mel'] = False
        from model import build_model
        self.net = build_model(**config)
        self.metadata = load_state(self.net, weights)
        self.metadata.update(name='AVSegFormer', repository='https://github.com/vvvb-github/AVSegFormer',
                             dataset='AVSBench AVSS', classes=71, backend=backend,
                             commit=subprocess.check_output(['git','-C',str(vendor),'rev-parse','HEAD'], text=True).strip())
        self.net.to(device)
        self.device, self.trace = device, Trace()
        label_path = ROOT/'config/avsbench_label2idx.json'
        if label_path.exists():
            mapping = json.loads(label_path.read_text())
            if set(mapping.values()) != set(range(71)):
                raise ValueError('Invalid official 71-class mapping')
            self.names = [next(k for k,v in mapping.items() if v == i) for i in range(71)]
            self.background = mapping.get('background',0)
            self.metadata['label_mapping']='AVSBench 71-category public AVSS metadata'
            self.metadata['label_mapping_source']='https://github.com/ruohaoguo/ovavss/blob/main/ov_avss/data/datasets/avsbench.py'
        else:
            # Class IDs are valid network outputs. Never attach names from a
            # differently reordered dataset/model while the original is absent.
            self.names = [f'AVSS class #{i}' for i in range(71)]
            self.background = 0
            self.metadata['label_mapping']='class indices only; native names pending'
        self.trace.hook(self.net.vggish, 'audio.embedding', 'VGGish log-Mel → 128')
        self.trace.hook(self.net.audio_proj, 'audio.projected', 'Linear 128 → 256')
        for i, layer in enumerate(self.net.head.in_proj):
            self.trace.hook(layer, f'visual.scale{i+1}', '1×1 Conv + GroupNorm → 256')
        def transformer(_, inputs):
            self.trace.record('visual.concat_tokens', inputs[1], 'cat([28×28,14×14,7×7] tokens, dim=1)')
        self.trace.handles.append(self.net.head.transformer.register_forward_pre_hook(transformer))
        def mixer(_, inputs):
            self.trace.record('visual.before_mixer', inputs[0], 'multi-scale spatial features at 56×56')
        self.trace.handles.append(self.net.head.fusion_block.register_forward_pre_hook(mixer))
        self.trace.hook(self.net.head.fusion_block, 'audiovisual.fused', 'audio query attention → sigmoid channel weights × visual map')
        self.trace.hook(self.net.head.query_generator, 'audiovisual.queries', 'audio-conditioned queries + learned queries')

    @torch.inference_mode()
    def infer(self, rgbs, pcm, rate):
        from PIL import Image
        from model.vggish.vggish_input import waveform_to_examples
        self.trace.clear()
        mel = waveform_to_examples(pcm, rate).detach()
        if mel.shape != (10, 1, 96, 64) or len(rgbs) != 10:
            raise ValueError(f'AVSS requires 10 genuine matched frames/log-Mel examples, got {tuple(mel.shape)}')
        frames = np.stack([np.asarray(Image.fromarray(rgb).resize((224,224),getattr(Image,'Resampling',Image).BILINEAR),
                                      np.float32).transpose(2,0,1)/255 for rgb in rgbs])
        frames = (frames - np.asarray([.485,.456,.406],np.float32)[None,:,None,None]) / np.asarray([.229,.224,.225],np.float32)[None,:,None,None]
        return self.infer_preprocessed(frames,mel.numpy(),rgbs[-1])

    @torch.inference_mode()
    def infer_preprocessed(self,frames,log_mel,rgb):
        """Replay saved genuine normalized inputs without reconstructing missing PCM."""
        self.trace.clear()
        frames=np.asarray(frames,np.float32)
        mel=torch.from_numpy(np.asarray(log_mel,np.float32))
        if frames.shape!=(10,3,224,224) or mel.shape!=(10,1,96,64):
            raise ValueError('AVSS requires ten matched frames and log-Mel examples')
        if not np.isfinite(frames).all() or not torch.isfinite(mel).all():
            raise ValueError('Nonfinite AVSS replay inputs')
        pred, _ = self.net(mel.to(self.device), torch.from_numpy(frames).to(self.device))
        if pred.shape != (10, 71, 224, 224) or not torch.isfinite(pred).all():
            raise ValueError(f'Invalid AVSS output: {tuple(pred.shape)}')
        labels = pred.argmax(1).cpu().numpy().astype(np.uint8)
        import cv2
        display = cv2.resize(labels[-1], (rgb.shape[1],rgb.shape[0]), interpolation=cv2.INTER_NEAREST)
        pictures, classes = predictions(rgb, display, self.names, background=self.background)
        annotations = annotated_predictions(pictures,rgb,display,self.names,task='avs')
        lm = mel[-1,0].numpy()
        lm = ((lm-lm.min())/max(1e-8,float(np.ptp(lm)))*255).astype(np.uint8)
        pictures['spectrum'] = jpeg(cv2.cvtColor(cv2.applyColorMap(lm.T[::-1].copy(),cv2.COLORMAP_TURBO),cv2.COLOR_BGR2RGB))
        self.trace.arrays.update(output_labels=labels, input_log_mel=mel.numpy(), input_frames=frames)
        return dict(images=pictures, classes=classes,annotations=annotations,features=list(self.trace.rows),output_shape=list(pred.shape),
                    baseline_note='未运行独立RGB/音频单模态任务基线；需人工真值评价融合精度'), labels
