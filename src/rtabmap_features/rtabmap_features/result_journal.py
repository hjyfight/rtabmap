"""Bounded local evidence journal and timing through DDS publication enqueue."""
import hashlib
import json
import time
from pathlib import Path
from .standard_interface import canonical,validate_result


def publication_observation(result, received, processing_started, publish_started, published, previous=None):
    validate_result(result)
    if not received<=processing_started<=publish_started<=published:raise ValueError('Nonmonotonic timing boundaries')
    timing=dict(processing_s=published-processing_started,queue_s=processing_started-received,
        receipt_to_result_s=published-received,publication_enqueue_s=published-publish_started,
        core_compute_s=result['timing']['core_compute_s'],preparation_s=result['timing']['preparation_s'],
        output_interval_s=None if previous is None else published-previous,
        task_period_s=result['timing']['task_period_s'],
        boundary='accepted_preparation_through_association_encoding_tracking_validation_serialization_and_dds_enqueue; excludes_upstream_segmentation_transport_and_subscriber_display',
        clock='local_monotonic',period_verdict='not_assessed_task_period_unspecified')
    record=dict(schema='object_multimodal_observation/1',result=result,timing=timing,recorded_at=time.time())
    record['sha256']=hashlib.sha256(canonical(record)).hexdigest()
    return record


def validate_observation(record):
    if not isinstance(record,dict) or record.get('schema')!='object_multimodal_observation/1':raise ValueError('Unsupported observation schema')
    if record.get('sha256')!=hashlib.sha256(canonical({k:v for k,v in record.items() if k!='sha256'})).hexdigest():
        raise ValueError('Observation integrity failed')
    validate_result(record['result'])
    timing=record['timing']
    for name in ('processing_s','queue_s','receipt_to_result_s','publication_enqueue_s'):
        value=timing.get(name)
        if type(value) not in (int,float) or not 0<=value<float('inf'):raise ValueError('Invalid publication timing')
    if abs(timing['receipt_to_result_s']-timing['processing_s']-timing['queue_s'])>1e-6:
        raise ValueError('Timing boundary arithmetic failed')
    return record


class ResultJournal:
    def __init__(self,path,max_bytes=64*1024*1024,backups=3):
        self.path=Path(path);self.max_bytes=max_bytes;self.backups=backups
    def append(self,record):
        validate_observation(record);self.path.parent.mkdir(parents=True,exist_ok=True)
        data=json.dumps(record,ensure_ascii=False,allow_nan=False,separators=(',',':'))+'\n'
        if self.path.exists() and self.path.stat().st_size+len(data.encode())>self.max_bytes:
            for index in range(self.backups-1,0,-1):
                old=self.path.with_name(self.path.name+f'.{index}')
                if old.exists():old.replace(self.path.with_name(self.path.name+f'.{index+1}'))
            self.path.replace(self.path.with_name(self.path.name+'.1'))
        with self.path.open('a') as output:output.write(data);output.flush()
