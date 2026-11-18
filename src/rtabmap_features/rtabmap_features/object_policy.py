"""Validate task policy before allocating model memory."""
import math
from .fusion import MODALITIES


def validate_task(task):
    if not isinstance(task,dict) or task.get('schema')!=1 or not isinstance(task.get('tracking_frame'),str) or not task['tracking_frame']:
        raise ValueError('Invalid target policy schema/frame')
    count=task.get('max_targets')
    if type(count) is not int or not 1<=count<=16:raise ValueError('max_targets must be 1..16')
    names=[c.get('name') for c in task.get('classes',[]) if isinstance(c,dict)]
    if not names or len(names)!=len(set(names)) or any(not isinstance(n,str) or not n.strip() for n in names):
        raise ValueError('Unique nonempty task classes required')
    templates=task.get('templates')
    if not isinstance(templates,list) or not templates:raise ValueError('Text templates required')
    for template in templates:
        if not isinstance(template,str) or template.count('{}')!=1:raise ValueError('Each template must have one class placeholder')
        template.format('target')
    weights=task.get('base_weights')
    if not isinstance(weights,list) or len(weights)!=len(MODALITIES) or any(type(w) not in (int,float) or not math.isfinite(w) or w<0 for w in weights) or sum(weights)<=0:
        raise ValueError('Four nonnegative finite weights with positive sum required')
    for name,lower,upper in [('reassociation_s',0,60),('min_nir_coverage',0,1),('audio_angle_deg',0,90),('audio_ambiguity_deg',0,90)]:
        value=task.get(name)
        if type(value) not in (int,float) or not math.isfinite(value) or not lower<value<=upper:raise ValueError('Invalid policy '+name)
    period=task.get('task_period_s')
    if period is not None and (type(period) not in (int,float) or not math.isfinite(period) or period<=0):raise ValueError('Invalid task period')
    acoustic=task.get('acoustic_source_classes')
    if not isinstance(acoustic,list) or any(not isinstance(n,str) or not n for n in acoustic):raise ValueError('Acoustic source classes required')
    return task
