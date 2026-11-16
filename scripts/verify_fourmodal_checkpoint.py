#!/usr/bin/env python3
"""Verify the official backbone structure and record a local download hash."""
import argparse
import hashlib
import json
from pathlib import Path


def main():
    import torch
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--checkpoint', type=Path, required=True)
    p.add_argument('--revision', required=True)
    args = p.parse_args()
    state = torch.load(args.checkpoint, map_location='cpu', weights_only=True, mmap=True)
    expected = {'pc_projection': (384, 512),
                'bind.modality_heads.vision.2.weight': (1024, 1280),
                'bind.modality_heads.audio.2.weight': (1024, 768),
                'bind.modality_head_point.2.weight': (1024, 512)}
    for name, shape in expected.items():
        if name not in state or tuple(state[name].shape) != shape:
            raise RuntimeError(f'Unexpected checkpoint tensor: {name}; expected {shape}')
    digest = hashlib.sha256()
    with args.checkpoint.open('rb') as stream:
        for block in iter(lambda: stream.read(8 * 1024 * 1024), b''):
            digest.update(block)
    info = dict(source='UniBind official Quick Start',
                source_revision=args.revision,
                download_url='https://drive.google.com/file/d/1Dgmj7ajdoT8hYHobJQfgIuB4CbVRoKUn/view',
                bytes=args.checkpoint.stat().st_size, sha256=digest.hexdigest(),
                hash_kind='locally_recorded_not_publisher_signature',
                verified_tensor_shapes={k: list(v) for k, v in expected.items()})
    args.checkpoint.with_suffix('.manifest.json').write_text(json.dumps(info, indent=2) + '\n')
    print(json.dumps(info, indent=2), flush=True)


if __name__ == '__main__':
    main()
