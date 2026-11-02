from pathlib import Path
import numpy as np


class YoloDetector:
    def __init__(self, model_path, device='cpu', confidence=0.5, image_size=640, threads=2,
                 backend='yolov8'):
        if backend not in ('yolov8', 'yoloe_pf'):
            raise ValueError('detector_backend must be yolov8 or yoloe_pf')
        path = Path(model_path).expanduser().resolve()
        if not path.is_file():
            installer = 'install_yoloe.sh' if backend == 'yoloe_pf' else 'install_semantic.sh'
            raise FileNotFoundError(f'Segmentation weights not found: {path}. Run scripts/{installer}.')
        import torch
        torch.set_num_threads(threads)
        if backend == 'yoloe_pf':
            from ultralytics import YOLOE
            self.model = YOLOE(str(path))
            if not hasattr(self.model.model.model[-1], 'lrpc'):
                raise ValueError('YOLOE requires a prompt-free *-seg-pf.pt checkpoint; text/visual models are not supported here')
            # The checkpoint already carries its vocabulary. Do not call set_classes,
            # set_vocab, or load a text encoder: classes=['*'] is only an output filter.
        else:
            from ultralytics import YOLO
            self.model = YOLO(str(path), task='segment')
        if self.model.task != 'segment':
            raise ValueError('Use instance segmentation weights to locate labels with depth')
        self.backend = backend
        self.device, self.confidence, self.image_size = device, confidence, image_size

    def detect(self, image):
        options = dict(agnostic_nms=True, iou=0.5) if self.backend == 'yoloe_pf' else {}
        result = self.model.predict(image, device=self.device, conf=self.confidence,
                                    imgsz=self.image_size, retina_masks=True, verbose=False,
                                    **options)[0]
        if result.masks is None or result.boxes is None:
            return []
        masks = result.masks.data.cpu().numpy()
        classes = result.boxes.cls.cpu().numpy().astype(int)
        scores = result.boxes.conf.cpu().numpy()
        labels = [str(result.names[int(cls)]) for cls in classes]
        if self.backend == 'yoloe_pf':
            from .labels import canonical_label
            labels = [canonical_label(label) for label in labels]
        return [dict(label=label, confidence=float(score),
                     mask=np.asarray(mask > 0.5, dtype=np.uint8))
                for mask, label, score in zip(masks, labels, scores)]
