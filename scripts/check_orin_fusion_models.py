#!/usr/bin/env python3
"""Real YOLOE/UniBind inference on an example image with synthetic NIR geometry."""
import json
from pathlib import Path
import sys
import time

import cv2
import numpy as np
import ultralytics

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT / 'src/rtabmap_semantic'), str(ROOT / 'src/rtabmap_features')]
from rtabmap_semantic.detector import YoloDetector
from rtabmap_semantic.segmentation import vocabulary, prepare_instances
from rtabmap_semantic.object_fusion import TaskVocabulary, ObjectFusionEngine
from rtabmap_features.backbone import FrozenEncoder


def main():
    image = cv2.imread(str(Path(ultralytics.__file__).parent / 'assets/bus.jpg'))
    if image is None:
        raise RuntimeError('Ultralytics example image missing')
    image = cv2.resize(image, (640, 480))
    nir = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    detector = YoloDetector(ROOT / 'models/yoloe-11s-seg-pf.pt', device='cuda:0',
                            confidence=.45, backend='yoloe_pf')
    labels, _ = vocabulary(detector)
    started = time.monotonic()
    rgb_instances = prepare_instances(detector.detect(image), image.shape, labels)
    nir_instances = prepare_instances(detector.detect(np.repeat(nir[:, :, None], 3, axis=2)), nir.shape, labels)
    if not rgb_instances or not nir_instances:
        raise RuntimeError('Real YOLOE example returned no RGB/NIR detections')
    yolo_seconds = time.monotonic() - started
    encoder = FrozenEncoder(ROOT / '.tools/unibind', ROOT / 'models/unibind/pretrained_weights.pt',
                            device='cuda:0', retain_text=True)
    task = TaskVocabulary(ROOT / 'config/segmentation_fusion_task.json')
    engine = ObjectFusionEngine(encoder, task, ROOT / 'models/unibind/task_prototypes', max_objects=4)
    h, w = nir.shape
    camera = dict(width=w, height=h, k=[640., 0., w / 2., 0., 640., h / 2., 0., 0., 1.],
                  p=[640., 0., w / 2., 0., 0., 640., h / 2., 0., 0., 0., 1., 0.],
                  r=np.eye(3).reshape(-1).tolist(), d=[0.] * 5, distortion_model='plumb_bob')
    bundle = dict(rgb=cv2.cvtColor(image, cv2.COLOR_BGR2RGB), nir=nir,
                  depth_m=np.full((h, w), 2., np.float32), rgb_info=camera, nir_info=camera,
                  transform=np.eye(4), rgb_instances=rgb_instances, nir_instances=nir_instances,
                  native_names=labels)
    result = engine.fuse(bundle)
    assert result['features'].shape[0] > 0
    assert result['features'].shape[1:] == (2, 1024)
    assert np.isfinite(result['features']).all()
    np.testing.assert_allclose(np.linalg.norm(result['features'], axis=-1), 1., atol=1e-5)
    assert result['fused_map'].shape == (h, w)
    report = dict(passed=True, device='cuda:0', real_yoloe=True, real_unibind=True,
                  robot_contacted=False, synthetic_nir_and_geometry=True, accuracy_test=False,
                  rgb_detections=len(rgb_instances), nir_detections=len(nir_instances),
                  rgb_labels=[row['label'] for row in rgb_instances],
                  fused_objects=len(result['objects']), feature_shape=list(result['features'].shape),
                  yolo_rgb_and_nir_seconds=yolo_seconds, fusion_timings_s=result['timings_s'],
                  checkpoint_sha256=encoder.provenance['checkpoint_sha256'])
    output = ROOT / 'logs/orin_perception/fusion_models.json'
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(report, ensure_ascii=False, indent=2) + '\n')
    print(json.dumps(report, ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()
