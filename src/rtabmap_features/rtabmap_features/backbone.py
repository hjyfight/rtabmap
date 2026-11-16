"""UniBind's public zero-shot PointBind backbone, strictly loaded and frozen."""
import hashlib
import json
from pathlib import Path
import sys
import time
import types
import numpy as np

REVISION = '7b22a7790af38d25ffa5b12d49fbe786dc302e6e'


def _adapt_module(name, filename, replacements):
    source = filename.read_text()
    for old, new in replacements:
        if source.count(old) != 1:
            raise RuntimeError(f'Upstream source changed: {filename}: {old}')
        source = source.replace(old, new)
    module = types.ModuleType(name)
    module.__file__ = str(filename)
    sys.modules[name] = module
    exec(compile(source, str(filename), 'exec'), module.__dict__)
    return module


def load_upstream(source):
    source = Path(source).resolve()
    for name, module in list(sys.modules.items()):
        if name == 'imagebind' or name.startswith(('imagebind.', 'models.i2p_mae.')):
            filename = getattr(module, '__file__', None)
            if filename and not Path(filename).resolve().is_relative_to(source):
                raise RuntimeError('A different ImageBind package is already loaded; use the dedicated runtime')
    sys.path.insert(0, str(source))
    # Only the constructor's numeric drop-path schedule needs CPU allocation;
    # parameters can stay on meta until pretrained checkpoint storage is assigned.
    _adapt_module('imagebind.transformer', source / 'imagebind/transformer.py', [
        ('torch.linspace(0, drop_path_rate, num_blocks)',
         'torch.linspace(0, drop_path_rate, num_blocks, device="cpu")')])
    from imagebind.imagebind_model import imagebind_huge
    point_dir = source / 'models/i2p_mae'
    _adapt_module('rtabmap_features._point_modules', point_dir / 'modules.py', [
        ('from . import misc', 'from rtabmap_features import point_ops as misc'),
        ('from knn_cuda import KNN', 'from rtabmap_features.point_ops import TorchKNN as KNN'),
        ('from utils.logger import *', '# Training-only logging omitted')])
    point_module = _adapt_module('rtabmap_features._point_encoder', point_dir / 'I2P_MAE.py', [
        ('from .checkpoint import get_missing_parameters_message, get_unexpected_parameters_message',
         'from models.i2p_mae.checkpoint import get_missing_parameters_message, get_unexpected_parameters_message'),
        ('from utils.logger import *', '# Training-only logging omitted'),
        ('from .modules import *', 'from rtabmap_features._point_modules import *')])
    return imagebind_huge, point_module.I2P_MAE_BIND


class FrozenEncoder:
    def __init__(self, source, checkpoint, device='cpu', infrared_kind='nir', threads=2, point_count=2048):
        import torch
        from torch import nn
        source, checkpoint = Path(source).resolve(), Path(checkpoint).resolve()
        if infrared_kind not in ('nir', 'thermal'):
            raise ValueError('infrared_kind must be nir or thermal')
        if not 1 <= threads <= 32:
            raise ValueError('CPU thread count must be 1..32')
        if not 512 <= point_count <= 8192:
            raise ValueError('point_count must be 512..8192')
        if not (source / '.source-revision').is_file() or (
                source / '.source-revision').read_text().strip() != REVISION:
            raise RuntimeError('Pinned UniBind source is missing; run scripts/install_fourmodal_features.sh')
        if not checkpoint.is_file():
            raise FileNotFoundError('Pretrained checkpoint missing; run scripts/install_fourmodal_features.sh')
        manifest = json.loads(checkpoint.with_suffix('.manifest.json').read_text())
        digest = hashlib.sha256()
        with checkpoint.open('rb') as stream:
            for block in iter(lambda: stream.read(8 * 1024 * 1024), b''):
                digest.update(block)
        if digest.hexdigest() != manifest['sha256'] or manifest['source_revision'] != REVISION:
            raise RuntimeError('Checkpoint differs from its verified download manifest')
        torch.set_num_threads(threads)
        self.device = torch.device(device)
        if self.device.type == 'cuda' and not torch.cuda.is_available():
            raise RuntimeError('CUDA is unavailable; use --device cpu')
        imagebind_huge, point_encoder_class = load_upstream(source)

        class Backbone(nn.Module):
            def __init__(self):
                super().__init__()
                self.logit_scale = nn.Parameter(torch.ones([]))
                # Allocate the large backbone on meta, then assign pretrained storage.
                with torch.device('meta'):
                    self.bind = imagebind_huge()
                self.point_encoder = point_encoder_class()
                self.pc_projection = nn.Parameter(torch.empty(384, 512))

        model = Backbone()
        state = torch.load(checkpoint, map_location='cpu', weights_only=True, mmap=True)
        model.load_state_dict(state, strict=True, assign=True)
        if any(p.is_meta for p in model.parameters()):
            raise RuntimeError('Checkpoint left uninitialized model parameters')
        del state
        # Drop unused text/depth/IMU weights after validating the entire checkpoint.
        keep = {'vision', 'audio'} | ({'thermal'} if infrared_kind == 'thermal' else set())
        for group in ('modality_preprocessors', 'modality_trunks', 'modality_heads', 'modality_postprocessors'):
            modules = getattr(model.bind, group)
            for name in list(modules.keys()):
                if name not in keep:
                    del modules[name]
        model.requires_grad_(False)
        self.model = model.eval().to(self.device)
        self.infrared_kind = infrared_kind
        self.point_count = point_count
        self.provenance = dict(backend='unibind_zero_shot_pointbind_i2pmae',
                               source_revision=REVISION, checkpoint_sha256=manifest['sha256'],
                               parameters_trainable=sum(p.numel() for p in model.parameters() if p.requires_grad),
                               output_dim=1024,
                               point_ops='torch_cpu_gpu_fps_knn',
                               point_ops_native_cuda_equivalence_validated=False,
                               nir_domain_validated=False,
                               lidar_scene_accuracy_validated=False,
                               inference_device=str(self.device))

    def encode(self, samples, anchor):
        import torch
        from .preprocessing import image_tensor, prepare_points, audio_tensor
        features, preprocessing, timing = {}, {}, {}
        with torch.inference_mode():
            visual_names = [name for name in ('rgb', 'infrared') if name in samples and (
                name != 'infrared' or self.infrared_kind == 'nir')]
            if visual_names:
                started = time.perf_counter()
                tensors = [image_tensor(samples[name].payload, infrared=name == 'infrared')
                           for name in visual_names]
                embeddings = self.model.bind({'vision': torch.stack(tensors).to(self.device)})['vision']
                for i, name in enumerate(visual_names):
                    features[name] = embeddings[i].float().cpu().numpy().copy()
                    preprocessing[name] = dict(encoder='vision', resize_crop=[224, 224],
                                                input_shape=list(samples[name].payload.shape),
                                                nir_repeated_to_rgb=name == 'infrared')
                timing['vision_batch_s'] = time.perf_counter() - started
            if 'infrared' in samples and self.infrared_kind == 'thermal':
                started = time.perf_counter()
                tensor = image_tensor(samples['infrared'].payload, infrared=True, thermal=True)
                features['infrared'] = self.model.bind({'thermal': tensor[None].to(self.device)})[
                    'thermal'][0].float().cpu().numpy().copy()
                preprocessing['infrared'] = dict(encoder='thermal', resize_crop=[224, 224],
                                               input_shape=list(samples['infrared'].payload.shape))
                timing['thermal_s'] = time.perf_counter() - started
            if 'lidar' in samples:
                started = time.perf_counter()
                points, preprocessing['lidar'] = prepare_points(
                    samples['lidar'].payload, count=self.point_count,
                    units=samples['lidar'].attributes.get('units', 'm'))
                embedding = self.model.point_encoder(points.to(self.device)) @ self.model.pc_projection
                embedding = self.model.bind.modality_head_point(embedding)
                embedding = self.model.bind.modality_postprocessor_point(embedding)
                features['lidar'] = embedding[0].float().cpu().numpy().copy()
                timing['lidar_s'] = time.perf_counter() - started
            if 'audio' in samples:
                started = time.perf_counter()
                sample = samples['audio']
                tensor, preprocessing['audio'] = audio_tensor(
                    sample.payload, int(sample.attributes['sample_rate']), anchor - sample.alignment_stamp)
                features['audio'] = self.model.bind({'audio': tensor.to(self.device)})[
                    'audio'][0].float().cpu().numpy().copy()
                timing['audio_s'] = time.perf_counter() - started
        return features, preprocessing, timing
