"""Bounded views of the actual in-memory inputs used for one inference result."""
import copy

from export_research_process import populate_visuals


def build_input_preview(task, session, result, arrays, maximum=1024):
    context=result.get('context') or {}
    sample={'visuals':{},'tables':{},'features':result.get('features',[])}
    populate_visuals(sample,task,context,arrays,maximum=maximum)
    if task=='pmf' and 'concatenation' in sample['tables']:
        sample['tables']['concatenation']['origin']='本次推理内存中的实际对照分支输入抽样'
    return dict(schema='research_input_preview/1',task=task,session_id=session,
                sequence=result['sequence'],source_stamp=result['source_stamp'],
                source='current_inference_memory',max_points=maximum,
                inputs_used=copy.deepcopy(context.get('inputs_used',[])),
                visuals=sample['visuals'],tables=sample['tables'])
