#!/usr/bin/env python3
"""Summarize captured target-result timing. Unspecified task periods are never judged passed."""
import argparse
import json
from pathlib import Path
import sys
import numpy as np

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'src/rtabmap_features'))
from rtabmap_features.standard_interface import validate_result
from rtabmap_features.result_journal import validate_observation


def cycle_report(records,task_period_s=None):
    if task_period_s is not None and (not np.isfinite(task_period_s) or task_period_s<=0):
        raise ValueError('Task period must be positive')
    if not records:raise ValueError('No measured results')
    boundaries=set();full_publication=True;identities=set()
    for record in records:
        observed=record.get('schema')=='object_multimodal_observation/1'
        if observed:validate_observation(record)
        else:validate_result(record);full_publication=False
        boundaries.add(record['timing']['boundary'])
        result=record['result'] if observed else record
        identity=(result['frame_id'],result['stamp'],result['sequence'])
        if identity in identities:raise ValueError('Duplicate timing results')
        identities.add(identity)
    if len(boundaries)!=1:raise ValueError('Different timing boundaries must be reported separately')
    columns={}
    for key in ('processing_s','core_compute_s','preparation_s','queue_s','receipt_to_result_s','output_interval_s','publication_enqueue_s'):
        values=[r['timing'][key] for r in records if r['timing'].get(key) is not None]
        columns[key]=dict(samples=len(values),median_s=float(np.median(values)),p95_s=float(np.quantile(values,.95)),max_s=float(max(values))) if values else None
    verdict='not_assessed_task_period_unspecified' if task_period_s is None else 'incomplete_processing_boundary' if not full_publication else (
        'measured_processing_within_period' if columns['processing_s']['max_s']<=task_period_s else 'measured_processing_exceeds_period')
    return dict(schema=1,results=len(records),task_period_s=task_period_s,verdict=verdict,timing=columns,
                hardware_accuracy_claim=False,boundary=next(iter(boundaries)),full_publication_boundary=full_publication,
                note='Processing, queue/receipt latency and output interval are separate. A timing sample is not a worst-case real-time guarantee.')


def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('results',type=Path,nargs='+')
    parser.add_argument('--task-period-s',type=float);parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args();records=[]
    for filename in args.results:
        if filename.suffix=='.jsonl':records.extend(json.loads(line) for line in filename.read_text().splitlines() if line.strip())
        else:
            value=json.loads(filename.read_text());records.extend(value if isinstance(value,list) else [value])
    report=cycle_report(records,args.task_period_s);args.output.parent.mkdir(parents=True,exist_ok=True)
    args.output.write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n');print(json.dumps(report,ensure_ascii=False,indent=2))


if __name__=='__main__':main()
