"""Measured signal quality and source continuity; never infer hardware health from presence."""
import hashlib
import math
import numpy as np


def signal_quality(modality, payload):
    data = np.asarray(payload)
    if not data.size or (modality!='lidar' and not np.isfinite(data).all()):
        return dict(score=0., grade='invalid', usable=False, reasons=['empty_or_nonfinite'], metrics={})
    reasons, metrics, score = [], {}, 1.
    if modality in ('rgb', 'infrared'):
        if data.dtype != np.uint8 or data.ndim not in (2, 3):
            return dict(score=0., grade='invalid', usable=False, reasons=['invalid_image_format'], metrics={})
        grey = data.astype(float).mean(axis=2) if data.ndim == 3 else data.astype(float)
        metrics = dict(mean=float(grey.mean()), std=float(grey.std()),
                       dark_fraction=float((grey < 5).mean()), saturated_fraction=float((grey > 250).mean()))
        for key, reason in [('dark_fraction', 'underexposure'), ('saturated_fraction', 'overexposure')]:
            if metrics[key] > .35:
                reasons.append(reason); score *= max(.05, 1 - metrics[key])
        if metrics['std'] < 2:
            reasons.append('low_texture_or_obstruction'); score *= .25
        if min(grey.shape) >= 3:
            lap = 4*grey[1:-1, 1:-1]-grey[:-2, 1:-1]-grey[2:, 1:-1]-grey[1:-1, :-2]-grey[1:-1, 2:]
            metrics['laplacian_variance'] = float(lap.var())
            if metrics['laplacian_variance'] < 2:
                reasons.append('low_sharpness'); score *= .7
    elif modality == 'lidar':
        if data.ndim != 2 or data.shape[1] < 3:
            return dict(score=0., grade='invalid', usable=False, reasons=['invalid_point_shape'], metrics={})
        finite=np.isfinite(data[:,:3]).all(axis=1)
        ranges = np.linalg.norm(np.where(np.isfinite(data[:,:3]),data[:,:3],0), axis=1)
        valid = finite & (ranges > .05) & (ranges <= 40)
        count = len(np.unique(data[valid, :3], axis=0))
        metrics = dict(points=len(data), valid_fraction=float(valid.mean()), distinct_points=count,nonfinite_points=int((~finite).sum()))
        if not finite.all():reasons.append('nonfinite_points_filtered')
        score = float(valid.mean()) * min(1., count/512)
        if count < 512:
            reasons.append('insufficient_distinct_points')
        if count==0:return dict(score=0.,grade='invalid',usable=False,reasons=reasons,metrics=metrics)
    elif modality == 'audio':
        if data.ndim not in (1, 2) or np.abs(data).max() > 1.00001:
            return dict(score=0., grade='invalid', usable=False, reasons=['invalid_pcm'], metrics={})
        metrics = dict(rms=float(np.sqrt(np.mean(data.astype(float)**2))),
                       clipped_fraction=float((np.abs(data) >= .999).mean()),
                       channels=1 if data.ndim == 1 else data.shape[1])
        if metrics['rms'] < 1e-4:
            reasons.append('silence'); score *= .1
        if metrics['clipped_fraction'] > .01:
            reasons.append('clipping'); score *= max(.05, 1-metrics['clipped_fraction']*5)
    else:
        raise ValueError('Unknown modality')
    return dict(score=float(np.clip(score, 0, 1)), grade='degraded' if reasons else 'nominal',
                usable=True, reasons=reasons, metrics=metrics)


class SourceMonitor:
    def __init__(self, max_age_s=3., freeze_s=2., max_records=128):
        self.max_age_s, self.freeze_s, self.max_records = max_age_s, freeze_s, max_records
        self.records, self.events = {}, []

    def observe(self, modality, payload, stamp, source_id, frame_id, arrival_s, alignment_stamp=None,
                stamp_quality='acquisition_header', authenticated=False, duration_s=0.):
        if not all(math.isfinite(v) for v in (stamp, arrival_s)) or not source_id or not frame_id:
            raise ValueError('Missing source identity/frame or invalid time')
        aligned = stamp if alignment_stamp is None else alignment_stamp
        if not math.isfinite(aligned):
            raise ValueError('Invalid alignment time')
        quality = signal_quality(modality, payload)
        data = np.ascontiguousarray(payload)
        # Full payload digest distinguishes repeated packets from constant observations.
        fingerprint = hashlib.sha256(data.tobytes()).hexdigest()
        key = (modality, source_id.split(':')[0], frame_id)
        previous = self.records.get(key)
        actions, continuity = [], []
        if previous:
            if stamp < previous['stamp']:
                continuity.append('clock_regression_or_source_restart'); actions.append('reject_and_reset_source')
                quality.update(usable=False, score=0., grade='invalid')
            elif stamp == previous['stamp']:
                continuity.append('duplicate_acquisition_stamp'); actions.append('reject_duplicate')
                quality.update(usable=False, score=0., grade='invalid')
            elif arrival_s-previous['arrival_s'] > self.max_age_s:
                continuity.append('input_gap'); actions.append('resume_with_new_acquisition')
        constant_since = previous['constant_since'] if previous and previous['fingerprint'] == fingerprint else arrival_s
        if arrival_s-constant_since >= self.freeze_s:
            # A static scene may be identical: report suspicion rather than claim a frozen device.
            continuity.append('suspected_frozen_or_constant_signal'); actions.append('request_sensor_check')
            quality['score'] *= .5; quality['grade'] = 'degraded'
        age = arrival_s-aligned-(duration_s if modality=='audio' else 0.)
        if age > self.max_age_s:
            continuity.append('stale_acquisition'); actions.append('reject_stale')
            quality.update(usable=False, score=0., grade='invalid')
        if age < -.1:
            continuity.append('clock_offset_unverified_or_future_stamp'); actions.append('request_clock_check')
        self.records[key] = dict(stamp=stamp, arrival_s=arrival_s, fingerprint=fingerprint, constant_since=constant_since)
        if len(self.records) > self.max_records:
            self.records.pop(next(iter(self.records)))
        record = dict(modality=modality, source_id=source_id, frame_id=frame_id, stamp=stamp,
                      alignment_stamp=aligned, observed_at=arrival_s, payload_sha256=fingerprint,
                      duration_s=duration_s,
                      quality=quality, sensor_health='unverified' if not continuity else 'attention_required',
                      health_basis='signal_and_stream_observation_not_hardware_self_test',
                      stamp_quality=stamp_quality, authenticated=bool(authenticated),
                      continuity=continuity, processing_actions=actions or ['validated_format_and_measured_signal'])
        if continuity or quality['reasons']:
            self.events.append({k:record[k] for k in ('modality','source_id','stamp','continuity','processing_actions')})
            self.events = self.events[-self.max_records:]
        return record
