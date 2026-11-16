#!/usr/bin/env python3
from pathlib import Path
import sys
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'src/rtabmap_semantic'))
from rtabmap_semantic.nir_segmentation_node import main
if __name__=='__main__':main()
