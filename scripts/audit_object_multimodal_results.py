#!/usr/bin/env python3
"""Validate recorded consumer interfaces and count rejected records; no robot/control integration."""
import argparse
import json
from pathlib import Path
import sys
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT/'src/rtabmap_features'))
from rtabmap_features.standard_interface import validate_result,require_control_approval
from rtabmap_features.result_journal import validate_observation

def audit(records):
    accepted=0;rejected=[];targets=0;complete=0;restricted=0;conflicts=0
    for index,record in enumerate(records):
        try:
            if record.get('schema')=='object_multimodal_observation/1':validate_observation(record);record=record['result']
            validate_result(record);accepted+=1
            targets+=len(record['targets']);complete+=sum(t['features']['complete'] for t in record['targets'])
            conflicts+=sum(len(t['uncertainty']['conflicts']) for t in record['targets'])
            try:require_control_approval(record)
            except ValueError:restricted+=1
        except (ValueError,TypeError,KeyError,AttributeError) as error:rejected.append(dict(index=index,reason=str(error)))
    return dict(schema=1,scope='versioned_target_interface_validation_not_standard_or_accuracy_acceptance',
        passed=accepted>0 and not rejected,accepted_results=accepted,rejected_results=len(rejected),rejections=rejected,
        targets=targets,complete_fourmodal_targets=complete,semantic_conflicts=conflicts,
        automatic_control_rejected_results=restricted,hardware_acceptance=False)

def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('results',type=Path,nargs='+')
    parser.add_argument('--output',type=Path,required=True);args=parser.parse_args();records=[]
    for filename in args.results:
        if filename.stat().st_size>64*1024*1024:raise ValueError('Input evidence file exceeds limit')
        if filename.suffix=='.jsonl':records.extend(json.loads(line) for line in filename.read_text().splitlines() if line.strip())
        else:
            value=json.loads(filename.read_text());records.extend(value if isinstance(value,list) else [value])
    report=audit(records);args.output.parent.mkdir(parents=True,exist_ok=True)
    args.output.write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n');print(json.dumps(report,ensure_ascii=False,indent=2))
    return 0 if report['passed'] else 1
if __name__=='__main__':sys.exit(main())
