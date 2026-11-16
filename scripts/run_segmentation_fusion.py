#!/usr/bin/env python3
from pathlib import Path
import sys
root=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(root/'src/rtabmap_semantic'),str(root/'src/rtabmap_features')]
from rtabmap_semantic.segmentation_fusion_node import main
if __name__=='__main__':main()
