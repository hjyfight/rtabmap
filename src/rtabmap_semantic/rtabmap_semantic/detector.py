from pathlib import Path
import numpy as np


class YoloDetector:
    def __init__(self, model_path, device='cpu', confidence=0.5, image_size=640, threads=2):
        path = Path(model_path).expanduser().resolve()
        if not path.is_file():
            raise FileNotFoundError(f'YOLO segmentation weights not found: {path}. Run scripts/install_semantic.sh.')
        import torch
        from ultralytics import YOLO
        torch.set_num_threads(threads)
        self.model = YOLO(str(path), task='segment')
        if self.model.task != 'segment':
            raise ValueError('Use segmentation weights such as yolov8n-seg.pt')
        self.device, self.confidence, self.image_size = device, confidence, image_size

    def detect(self, image):
        result = self.model.predict(image, device=self.device, conf=self.confidence,
                                    imgsz=self.image_size, retina_masks=True, verbose=False)[0]
        if result.masks is None or result.boxes is None:
            return []
        masks = result.masks.data.cpu().numpy()
        classes = result.boxes.cls.cpu().numpy().astype(int)
        scores = result.boxes.conf.cpu().numpy()
        return [dict(label=str(result.names[int(cls)]), confidence=float(score),
                     mask=np.asarray(mask > 0.5, dtype=np.uint8))
                for mask, cls, score in zip(masks, classes, scores)]
