"""NYUv2 EMSANet RGB-D inference, adapted to the mapper's instance contract.

The input depth must already be registered to the color camera. The official
NYUv2 model consumes millimetres; ROS 32FC1 images arrive in metres. Network
resizing is undone by upstream full-resolution postprocessing before lifting.
"""
from pathlib import Path

import cv2
import numpy as np


EMSANET_POLICY_VERSION = 2
_NYUV2_CHINESE = {
    'blinds': '百叶窗', 'shelves': '置物架', 'clothes': '衣物', 'books': '书籍',
    'refrigerator': '冰箱', 'night stand': '床头柜', 'toilet': '马桶',
    'bathtub': '浴缸', 'otherstructure': '其他结构',
    'otherfurniture': '其他家具', 'otherprop': '其他物品',
}


def emsanet_chinese_label(label):
    from .labels import chinese_label
    return _NYUV2_CHINESE.get(label, chinese_label(label))


def prepare_rgbd(bgr, depth):
    """Copy/validate registered ROS inputs and convert to RGB + float32 mm."""
    if bgr.dtype != np.uint8 or bgr.ndim != 3 or bgr.shape[2] != 3 or not bgr.size:
        raise ValueError('EMSANet requires a nonempty uint8 BGR image')
    if depth is None or depth.ndim != 2 or not depth.size or depth.dtype not in (np.uint16, np.float32):
        raise ValueError('EMSANet requires registered 16UC1 (mm) or 32FC1 (m) depth')
    height, width = bgr.shape[:2]
    dh, dw = depth.shape
    if abs(width / height - dw / dh) > 0.02:
        raise ValueError('RGB and depth must share the registered camera/FOV; resizing is not registration')
    depth_mm = depth.astype(np.float32, copy=True)
    # Sanitize before multiplying to avoid overflow from invalid float depths.
    max_input = 10.0 if depth.dtype == np.float32 else 10000.0
    depth_mm[~np.isfinite(depth_mm) | (depth_mm <= 0) | (depth_mm > max_input)] = 0
    if depth.dtype == np.float32:
        depth_mm *= 1000.0
    if depth_mm.shape != (height, width):
        # Only scale an already registered image, preserving holes and edges.
        depth_mm = cv2.resize(depth_mm, (width, height), interpolation=cv2.INTER_NEAREST)
    return cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB), depth_mm


def panoptic_detections(panoptic_map, instance_ids, metadata, labels, confidence, label_divisor):
    """Decode authoritative packed panoptic IDs and reuse official scores.

In the pinned upstream CPU postprocessor, score computation mutates a tensor
alias and shifts both semantic_idx outputs and metadata by one. Packed IDs
remain intact on CPU and GPU. Using them also excludes stuff pixels that may
still be present in the raw instance map.
    """
    if panoptic_map.ndim != 2 or label_divisor <= 0:
        raise ValueError('Invalid EMSANet panoptic map or label divisor')
    detections = []
    for panoptic_id, instance_id in sorted(instance_ids.items(), key=lambda pair: (pair[1], pair[0])):
        if instance_id <= 0:
            continue
        info = metadata.get(int(instance_id), {})
        class_id = int(panoptic_id) // label_divisor
        if not 0 < class_id < len(labels):
            continue
        name, is_thing = labels[class_id]
        if not is_thing:
            continue
        score = float(info.get('panoptic_score', 0.0))
        if not np.isfinite(score) or not confidence <= score <= 1.0:
            continue
        mask = panoptic_map == panoptic_id
        if not mask.any():
            continue
        detections.append(dict(label=name, confidence=score, mask=mask.astype(np.uint8)))
    return detections


class EmsanetDetector:
    """Official R34-NBt1D NYUv2 checkpoint; no dataset download is required."""
    backend = 'emsanet'

    def __init__(self, model_path, device='cpu', confidence=0.35, image_size=640, threads=2):
        path = Path(model_path).expanduser().resolve()
        if not path.is_file():
            raise FileNotFoundError(f'EMSANet weights not found: {path}. Run bash scripts/install_emsanet.sh')
        if image_size < 128 or image_size % 128:
            raise ValueError('EMSANet image_size must be a positive multiple of 128 (default 640)')
        if not 0 < confidence <= 1 or threads < 1:
            raise ValueError('Invalid EMSANet confidence or CPU thread count')
        try:
            import torch
            from emsanet.args import ArgParserEMSANet
            from emsanet.data import get_dataset
            from emsanet.model import EMSANet
            from emsanet.preprocessing import get_preprocessor
            from nicr_mt_scene_analysis.data import move_batch_to_device, mt_collate
        except ImportError as error:
            raise RuntimeError('EMSANet runtime is missing. Run bash scripts/install_emsanet.sh '
                               'and use scripts/start_semantic_emsanet.sh') from error
        torch.set_num_threads(threads)
        self.torch, self.device = torch, torch.device(device)
        self.confidence = confidence
        self.collate, self.to_device = mt_collate, move_batch_to_device
        args = ArgParserEMSANet().parse_args([
            '--dataset', 'nyuv2', '--input-modalities', 'rgb', 'depth',
            '--tasks', 'semantic', 'instance', 'orientation', 'scene', '--enable-panoptic',
            '--rgb-encoder-backbone', 'resnet34',
            '--depth-encoder-backbone', 'resnet34',
            '--rgb-encoder-backbone-resnet-block', 'nonbottleneck1d',
            '--depth-encoder-backbone-resnet-block', 'nonbottleneck1d',
            '--no-pretrained-backbone',
            '--validation-input-width', str(image_size),
            '--validation-input-height', str(image_size * 3 // 4),
        ], verbose=False)
        dataset = get_dataset(args, args.validation_split)
        self.labels = tuple((label.class_name, bool(label.is_thing))
                            for label in dataset.config.semantic_label_list)
        self.model = EMSANet(args, dataset_config=dataset.config)
        checkpoint = torch.load(str(path), map_location='cpu', weights_only=True)
        state = checkpoint.get('state_dict', checkpoint)
        # This rename is the official legacy-checkpoint migration. Strict load
        # intentionally rejects other datasets/architectures instead of silently
        # leaving randomly initialized heads or copying only some categories.
        state = {key.replace('fused_encoders.', 'encoder.'): value for key, value in state.items()}
        self.model.load_state_dict(state, strict=True)
        self.model.eval().to(self.device)
        self.label_divisor = self.model.decoders['panoptic_helper'].postprocessing.max_instances_per_category
        self.preprocessor = get_preprocessor(args, dataset=dataset, phase='test',
                                             multiscale_downscales=None)

    def detect(self, image, depth):
        rgb, depth_mm = prepare_rgbd(image, depth)
        if not np.any(depth_mm):
            return []
        sample = self.preprocessor({'rgb': rgb, 'depth': depth_mm, 'identifier': 'keyframe'})
        batch = self.to_device(self.collate([sample]), device=self.device)
        # The mapper calls this on a worker thread; inference mode is thread-local.
        with self.torch.inference_mode():
            predictions = self.model(batch, do_postprocessing=True)
        panoptic = predictions['panoptic_segmentation_deeplab_fullres'][0].cpu().numpy()
        if panoptic.shape != image.shape[:2]:
            raise ValueError('EMSANet masks were not restored to the source RGB resolution')
        metadata = predictions['panoptic_segmentation_deeplab_instance_meta'][0]
        instance_ids = predictions['panoptic_segmentation_deeplab_ids'][0]
        return panoptic_detections(panoptic, instance_ids, metadata, self.labels,
                                   self.confidence, self.label_divisor)
