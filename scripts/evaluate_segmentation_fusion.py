#!/usr/bin/env python3
import argparse,json
from pathlib import Path
import sys
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'src/rtabmap_semantic'))
from rtabmap_semantic.fusion_evaluation import evaluate

parser=argparse.ArgumentParser(description='Fit policy on validation scenes; compare fusion against BOTH single modalities on held-out scenes.')
parser.add_argument('--index',type=Path,required=True)
parser.add_argument('--output',type=Path,required=True)
args=parser.parse_args()
report=evaluate(args.index)
args.output.parent.mkdir(parents=True,exist_ok=True)
args.output.write_text(json.dumps(report,ensure_ascii=False,indent=2,allow_nan=False)+'\n')
print(json.dumps(report,ensure_ascii=False,indent=2))
if not report['passed']:sys.exit(2)
