"""Image-space segmentation and conservative, task-specific fusion contracts. No ROS/map I/O."""
from copy import deepcopy
import hashlib
import json
import math

import numpy as np


def vocabulary(detector):
    names = detector.model.names
    items = names.items() if isinstance(names, dict) else enumerate(names)
    from .labels import canonical_label
    labels = {int(k): canonical_label(str(v)) if detector.backend == 'yoloe_pf' else str(v)
              for k, v in items}
    version = hashlib.sha256(json.dumps(labels, sort_keys=True).encode()).hexdigest()
    return labels, version


def prepare_instances(predictions, shape, labels):
    height, width = shape[:2]
    instances = []
    for i, row in enumerate(predictions, 1):
        mask = np.asarray(row['mask'])
        box = np.asarray(row['bbox_xyxy'], dtype=float)
        score, class_id = float(row['confidence']), int(row['class_id'])
        if mask.shape != (height, width):
            raise ValueError('YOLO mask is not in original image coordinates')
        if box.shape != (4,) or not np.isfinite(box).all() or not (
                0 <= box[0] < box[2] <= width and 0 <= box[1] < box[3] <= height):
            raise ValueError('Invalid original-image detection box')
        if class_id not in labels or not math.isfinite(score) or not 0 <= score <= 1:
            raise ValueError('Invalid YOLO class or confidence')
        ys, xs = np.nonzero(mask)
        if not len(xs):
            continue
        # Crop by mask extent, not bbox: retain every original foreground pixel.
        x, y, right, bottom = int(xs.min()), int(ys.min()), int(xs.max()) + 1, int(ys.max()) + 1
        instances.append(dict(instance_id=i, class_id=class_id, label=labels[class_id],
            confidence=score, classification_score=-1.0, bbox_xyxy=box.tolist(),
            mask_offset=[x, y], mask=np.asarray(mask[y:bottom, x:right] != 0, dtype=np.uint8)))
    return instances


def dense_maps(instances, shape):
    height, width = shape[:2]
    classes = np.full((height, width), -1, dtype=np.int32)
    identities = np.zeros((height, width), dtype=np.int32)
    # Highest original YOLO score wins overlaps; ties use smallest frame-local ID.
    for row in sorted(instances, key=lambda r: (-r['confidence'], r['instance_id'])):
        x, y = row['mask_offset']
        mask = row['mask'].astype(bool)
        h, w = mask.shape
        occupied = identities[y:y+h, x:x+w]
        take = mask & (occupied == 0)
        occupied[take] = row['instance_id']
        classes[y:y+h, x:x+w][take] = row['class_id']
    return classes, identities


def color_for(class_id):
    import colorsys
    return tuple(int(255 * c) for c in colorsys.hsv_to_rgb((class_id * 0.61803398875) % 1, .60, .96))


def render_images(rgb, classes):
    colors = np.full_like(rgb, 26)
    for class_id in np.unique(classes):
        if class_id >= 0:
            colors[classes == class_id] = color_for(int(class_id))
    overlay = rgb.copy()
    foreground = classes >= 0
    overlay[foreground] = np.rint(rgb[foreground] * .58 + colors[foreground] * .42).astype(np.uint8)
    return overlay, colors


def class_label_anchors(classes,min_pixels=64,max_labels=24):
    """Place display labels inside the actual visible class regions."""
    import cv2
    classes=np.asarray(classes)
    if classes.ndim!=2:raise ValueError('Class label anchors require a 2-D class map')
    anchors=[]
    for class_id in np.unique(classes):
        if class_id<0:continue
        count,components,stats,_=cv2.connectedComponentsWithStats(
            (classes==class_id).astype(np.uint8),connectivity=8)
        for component in range(1,count):
            left,top,width,height,pixels=[int(v) for v in stats[component]]
            if pixels<min_pixels:continue
            region=(components[top:top+height,left:left+width]==component).astype(np.uint8)
            distances=cv2.distanceTransform(np.pad(region,1),cv2.DIST_L2,3)[1:-1,1:-1]
            y,x=np.unravel_index(np.argmax(distances),region.shape)
            anchors.append(dict(class_id=int(class_id),x=int(left+x),y=int(top+y),pixels=pixels))
    return sorted(anchors,key=lambda row:(-row['pixels'],row['class_id'],row['y'],row['x']))[:max_labels]


def validate_candidate(candidate, baseline, labels, *, age, timeout, tolerance, min_score):
    """Validate a label-only refinement; provider declarations are provenance, not proof of accuracy."""
    for field in ('session_id', 'sequence', 'stamp_ns', 'frame_id', 'model_sha256', 'vocabulary_version'):
        if candidate.get(field) != baseline[field]:
            return False, 'mismatched_' + field
    if age < 0 or age > timeout:
        return False, 'candidate_expired'
    if not candidate.get('provider_id') or not candidate.get('refinements'):
        return False, 'empty_candidate'
    for key in ('calibration_version', 'score_calibration_version'):
        if candidate.get(key, '').strip().lower() in ('', 'unknown', 'unverified', 'none'):
            return False, 'unverified_' + key
    valid_ids = {r['instance_id'] for r in baseline['instances']}
    seen = set()
    for row in candidate['refinements']:
        identity = row['instance_id']
        if identity not in valid_ids or identity in seen:
            return False, 'invalid_instance_id'
        seen.add(identity)
        if row['class_id'] not in labels:
            return False, 'unknown_class'
        if not math.isfinite(row['score']) or not min_score <= row['score'] <= 1:
            return False, 'low_or_invalid_score'
        valid, weights, stamps = row['valid_mask'], row['weights'], row['source_stamps']
        if len(valid) != 4 or len(weights) != 4 or len(stamps) != 4:
            return False, 'invalid_modality_shape'
        if valid[3]:
            return False, 'audio_object_association_unsupported'
        if sum(bool(x) for x in valid) < 2:
            return False, 'insufficient_modalities'
        if not all(math.isfinite(w) and (0 < w <= 1 if v else w == 0) for v, w in zip(valid, weights)) \
                or not math.isclose(sum(weights), 1, abs_tol=1e-4):
            return False, 'invalid_weights'
        anchor = baseline['stamp_ns'] / 1e9
        if not row['time_alignment_valid'] or any(not math.isfinite(s) or abs(s-anchor) > tolerance
                                                 for v, s in zip(valid, stamps) if v):
            return False, 'unaligned_time'
        if not row['spatial_alignment_valid']:
            return False, 'unaligned_geometry'
    return True, 'valid_candidate'


def select_candidate(candidate, baseline, labels, *, mode, approved, current, **validation):
    valid, reason = validate_candidate(candidate, baseline, labels, **validation)
    if not valid:
        return baseline, reason
    if mode != 'guarded':
        return baseline, 'shadow_candidate' if mode == 'shadow' else 'fusion_disabled'
    if candidate['provider_id'] not in approved:
        return baseline, 'provider_not_approved'
    if not current:
        return baseline, 'superseded_frame'
    selected = deepcopy(baseline)
    changes = {r['instance_id']: r for r in candidate['refinements']}
    for row in selected['instances']:
        change = changes.get(row['instance_id'])
        if change:
            row.update(class_id=change['class_id'], label=labels[change['class_id']],
                       classification_score=change['score'])
    selected.update(result_source='fusion', selection_reason='validated_label_refinement',
                    provider_id=candidate['provider_id'], revision=1)
    return selected, 'fusion_selected'
