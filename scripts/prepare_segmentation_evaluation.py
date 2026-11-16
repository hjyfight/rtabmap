#!/usr/bin/env python3
import argparse,json
from pathlib import Path
import sys
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'src/rtabmap_semantic'))
from rtabmap_semantic.object_fusion import digest

parser=argparse.ArgumentParser(description='Build a review index; never create ground-truth labels from predictions.')
parser.add_argument('captures',type=Path,nargs='+')
parser.add_argument('--output',type=Path,required=True)
args=parser.parse_args()
samples=[];task=None
for capture in args.captures:
    capture=capture.resolve();current=json.loads((capture/'classes.json').read_text())
    if task and digest(task)!=digest(current):raise ValueError('Capture task vocabularies differ')
    task=current
    manifest=capture/'manifest.jsonl'
    for line in manifest.read_text().splitlines() if manifest.exists() else []:
        row=json.loads(line)
        samples.append(dict(sample_id=row['sample_id'],sample_path=str(capture/row['path']),
            rgb=str(capture/row['path']/'rgb.png'),annotation=str(capture/row['annotation']),
            group_id=row['group_id'],split='unassigned',annotation_source='pending'))
index=dict(schema=1,task_sha256=digest(task),classes=[r['name'] for r in task['classes']],samples=samples,
    instructions='Create reviewed single-channel class-ID PNG labels; 65534/65535 are ignored. Assign different scene groups to validation/test; mark reviewed labels as human_reviewed. Do not split adjacent frames across sets.')
args.output.parent.mkdir(parents=True,exist_ok=True)
args.output.write_text(json.dumps(index,ensure_ascii=False,indent=2)+'\n')
print(json.dumps(dict(index=str(args.output),samples=len(samples),human_labels_created=0)))
