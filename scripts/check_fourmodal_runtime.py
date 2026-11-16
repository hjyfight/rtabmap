#!/usr/bin/env python3
"""Check CPU imports and backbone tensor shapes; does not claim pretrained inference."""
from pathlib import Path
import sys
import torch
PROJECT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT / 'src/rtabmap_features'))
from rtabmap_features.backbone import load_upstream

torch.set_num_threads(2)
imagebind, point_encoder = load_upstream(PROJECT / '.tools/unibind')
with torch.device('meta'):
    model = imagebind()
points = point_encoder()
assert tuple(model.modality_heads['vision'][2].weight.shape) == (1024, 1280)
assert tuple(model.modality_heads['audio'][2].weight.shape) == (1024, 768)
assert tuple(model.modality_head_point[2].weight.shape) == (1024, 512)
print({'state': 'architecture_runtime_only_no_weights',
       'imagebind_parameters': sum(p.numel() for p in model.parameters()),
       'point_parameters': sum(p.numel() for p in points.parameters())})
