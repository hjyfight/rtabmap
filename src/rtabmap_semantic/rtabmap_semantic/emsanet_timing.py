"""CPU wall-clock profiling of disjoint EMSANet modules without changing tensors.

Backbone stages and RGB-D fusion stages alternate. Only sibling execution
regions are timed; no whole-encoder hook is summed with its children.
"""
from collections import Counter, defaultdict
import threading
import time


class EmsanetStageTimer:
    def __init__(self, model, device='cpu', clock=time.perf_counter_ns):
        self.device, self.clock = str(device), clock
        self.local = threading.local()
        self.handles = []
        self.sections = []
        self.fusion_name = str(getattr(model.args, 'encoder_fusion', 'unknown'))
        if self.device != 'cpu':
            return  # Do not label asynchronous GPU launch times as execution times.
        encoder = model.encoder
        for group, modules in (
                ('rgb_backbone', encoder.backbone_rgb.stages),
                ('depth_backbone', encoder.backbone_depth.stages),
                ('rgbd_fusion', encoder.fusions),
                ('context', [model.context_module]),
                ('decoders_and_postprocessing', list(model.decoders.values()))):
            for index, module in enumerate(modules):
                self.sections.append((f'{group}/{index}', group, module))
        # Guard against accidentally selecting a parent together with a child.
        selected = [id(module) for _, _, module in self.sections]
        if len(set(selected)) != len(selected) or any(
                any(id(child) in selected for child in list(module.modules())[1:])
                for _, _, module in self.sections):
            raise ValueError('EMSANet timing sections overlap or share a module')
        for name, group, module in self.sections:
            self.handles.append(module.register_forward_pre_hook(self._before(name, group)))
            self.handles.append(module.register_forward_hook(self._after(name), always_call=True))

    def _before(self, name, group):
        def hook(module, args):
            capture = getattr(self.local, 'capture', None)
            if capture is not None:
                if capture['active'] is not None:
                    capture['errors'].append('overlapping module execution')
                capture['active'] = dict(name=name, group=group, start_ns=self.clock())
        return hook

    def _after(self, name):
        def hook(module, args, output):
            capture = getattr(self.local, 'capture', None)
            if capture is None:
                return
            end = self.clock()
            active = capture['active']
            if active is None or active['name'] != name:
                capture['errors'].append('unmatched module end')
            else:
                capture['samples'].append({**active, 'end_ns': end,
                                           'duration_ms': (end-active['start_ns'])/1e6})
            capture['active'] = None
        return hook

    def measure(self, forward):
        if self.device != 'cpu':
            return forward(), dict(schema=1, state='unsupported_device', device=self.device)
        if getattr(self.local, 'capture', None) is not None:
            raise RuntimeError('Nested model timing is not supported')
        capture = dict(active=None, samples=[], errors=[])
        self.local.capture = capture
        started = self.clock()
        try:
            result = forward()
            ended = self.clock()
        finally:
            self.local.capture = None  # Exceptions cannot contaminate the next frame.
        expected = Counter(name for name, _, _ in self.sections)
        observed = Counter(row['name'] for row in capture['samples'])
        if expected != observed or capture['active'] is not None:
            capture['errors'].append('incomplete or repeated module coverage')
        groups = defaultdict(float)
        for row in capture['samples']:
            groups[row['group']] += row['duration_ms']
            if row['start_ns'] < started or row['end_ns'] > ended or row['end_ns'] < row['start_ns']:
                capture['errors'].append('invalid timing boundary')
        total = (ended-started)/1e6
        unassigned = total-sum(groups.values())
        if unassigned < 0:
            capture['errors'].append('module timings exceed full forward')
        values = dict(groups)
        values.update(feature_extraction=groups['rgb_backbone']+groups['depth_backbone'],
                      model_forward=total, model_other=unassigned)
        return result, dict(schema=1, state='valid' if not capture['errors'] else 'invalid',
                            device='cpu', clock='perf_counter_ns', fusion_name=self.fusion_name,
                            groups_ms=values, sections=capture['samples'], errors=capture['errors'])

    def close(self):
        for handle in self.handles:
            handle.remove()
        self.handles.clear()
