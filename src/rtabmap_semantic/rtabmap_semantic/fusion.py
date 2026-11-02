"""Object association, graph reprojection and transactional persistence."""
from dataclasses import dataclass, field, asdict
import json
from pathlib import Path
import sqlite3
import time
import uuid

import numpy as np
from scipy.optimize import linear_sum_assignment
from .geometry import transform_point


@dataclass
class Observation:
    node_id: int
    node_stamp: float
    position: list
    extent: list
    confidence: float
    weight: float
    label: str = ''


@dataclass
class ObjectTrack:
    object_id: int
    label: str
    dynamic: bool = False
    observations: list = field(default_factory=list)
    last_seen: float = 0.0
    distinct_ids: list = field(default_factory=list)
    confirmation_ids: list = field(default_factory=list)
    confirmed: bool = False

    @property
    def confidence(self):
        votes = [o.confidence for o in self.observations if not o.label or o.label == self.label]
        return float(np.mean(votes or [o.confidence for o in self.observations]))


class SemanticState:
    def __init__(self, min_observations=3, association_distance=0.45,
                 dynamic_classes=('person', 'cat', 'dog'), dynamic_ttl=3.0,
                 max_observations=30, confirmation_window=0, dynamic_min_observations=1,
                 unique_objects=False):
        if (confirmation_window == 0 and dynamic_min_observations != 1) or (
                confirmation_window < 0 or dynamic_min_observations < 1) or (
                confirmation_window and confirmation_window < max(min_observations, dynamic_min_observations)):
            raise ValueError('Confirmation window must cover the required observations')
        self.min_observations = min_observations
        self.association_distance = association_distance
        self.dynamic_classes = set(dynamic_classes)
        self.dynamic_ttl = dynamic_ttl
        self.max_observations = max_observations
        self.confirmation_window = confirmation_window
        self.dynamic_min_observations = dynamic_min_observations
        self.unique_objects = unique_objects
        self.map_uuid = str(uuid.uuid4())
        self.next_id = 1
        self.nodes = {}
        self.objects = []
        self.poses = {}
        self.recent_nodes = []

    def update_graph(self, poses):
        self.poses = {int(k): np.asarray(v) for k, v in poses.items() if int(k) > 0}

    def inlier_observations(self, obj):
        points, observations = [], []
        for obs in obj.observations:
            if obs.node_id in self.poses:
                points.append(transform_point(self.poses[obs.node_id], obs.position))
                observations.append(obs)
        if not points:
            return []
        # Outlier rejection operates after graph optimization, not in stale map coordinates.
        points = np.asarray(points)
        distances = np.linalg.norm(points - np.median(points, axis=0), axis=1)
        keep = distances <= max(self.association_distance, float(np.median(distances)) * 2.5)
        return [obs for obs, valid in zip(observations, keep) if valid]

    def position(self, obj):
        observations = self.inlier_observations(obj)
        if not observations:
            return None
        return np.average([transform_point(self.poses[o.node_id], o.position) for o in observations],
                          axis=0, weights=[o.weight for o in observations])

    def extent(self, obj):
        observations = self.inlier_observations(obj)
        if not observations:
            return None
        return np.median([np.abs(self.poses[o.node_id][:3, :3]) @ np.asarray(o.extent)
                          for o in observations], axis=0)

    def same_geometry(self, pa, ea, pb, eb):
        if pa is None or pb is None or ea is None or eb is None:
            return False
        # Small uncertainty padding handles depth noise; containment alone is insufficient
        # (a laptop on a desk is not the same object as the desk).
        ea, eb = np.maximum(ea, 0.02) + 0.04, np.maximum(eb, 0.02) + 0.04
        gate = min(0.20, self.association_distance * 0.5, float(min(np.linalg.norm(ea), np.linalg.norm(eb))) * 0.35)
        if np.linalg.norm(pa - pb) > gate:
            return False
        intersection = float(np.prod(np.maximum(0, np.minimum(pa+ea/2, pb+eb/2) - np.maximum(pa-ea/2, pb-eb/2))))
        union = float(np.prod(ea) + np.prod(eb) - intersection)
        return intersection / max(union, 1e-9) >= 0.45

    def choose_label(self, obj):
        if not self.unique_objects or not obj.observations:
            return
        votes = {}
        for obs in obj.observations:
            name = obs.label or obj.label
            by_node = votes.setdefault(name, {})
            by_node[obs.node_id] = max(by_node.get(obs.node_id, 0.0), obs.confidence)
        scores = {name: sum(values.values()) for name, values in votes.items()}
        winner = max(scores, key=lambda name: (scores[name], name == obj.label, name))
        if winner != obj.label and len(votes[winner]) >= self.min_observations and scores[winner] > scores.get(obj.label, 0.0) * 1.25:
            obj.label = winner

    def reconcile(self):
        """Merge duplicate tracks after loop correction, respecting co-observation evidence."""
        changed = False
        for a in list(self.objects):
            if a not in self.objects or a.dynamic:
                continue
            for b in list(self.objects):
                if b.object_id <= a.object_id or b.dynamic or (not self.unique_objects and b.label != a.label):
                    continue
                if b.object_id in a.distinct_ids or a.object_id in b.distinct_ids:
                    continue
                if len(a.observations) < self.min_observations or len(b.observations) < self.min_observations:
                    continue
                if self.confirmation_window and not (a.confirmed and b.confirmed):
                    continue
                pa, pb = self.position(a), self.position(b)
                if self.unique_objects:
                    if {o.node_id for o in a.observations} & {o.node_id for o in b.observations}:
                        continue
                    if not self.same_geometry(pa, self.extent(a), pb, self.extent(b)):
                        continue
                    for obj in (a, b):
                        for obs in obj.observations:
                            obs.label = obs.label or obj.label
                elif pa is None or pb is None or np.linalg.norm(pa-pb) > self.association_distance * 0.5:
                    continue
                a.observations = sorted(a.observations+b.observations,
                                        key=lambda o: o.node_stamp)[-self.max_observations:]
                a.distinct_ids = sorted(set(a.distinct_ids+b.distinct_ids))
                a.last_seen = max(a.last_seen, b.last_seen)
                self.choose_label(a)
                self.objects.remove(b)
                for other in self.objects:
                    if b.object_id in other.distinct_ids:
                        other.distinct_ids = sorted((set(other.distinct_ids)-{b.object_id}) | {a.object_id})
                changed = True
        return changed

    def check_node(self, node_id, stamp):
        """Return new/seen/reset. Reused IDs with different timestamps start an epoch."""
        if node_id not in self.nodes:
            return 'new'
        if abs(self.nodes[node_id] - stamp) < 1e-6:
            return 'seen'
        self.map_uuid = str(uuid.uuid4())
        self.next_id = 1
        self.nodes.clear()
        self.objects.clear()
        self.recent_nodes.clear()
        return 'reset'

    def add_frame(self, node_id, stamp, detections, now=None):
        now = time.monotonic() if now is None else now
        result = self.check_node(node_id, stamp)
        if result == 'seen':
            return result
        if node_id not in self.poses:
            raise ValueError('Keyframe pose has not arrived yet')
        self.expire(now)
        if self.confirmation_window:
            # Count processed keyframes, not numeric ID gaps or repeated bridge retries.
            self.recent_nodes = (self.recent_nodes + [node_id])[-self.confirmation_window:]
            recent = set(self.recent_nodes)
            for obj in self.objects:
                obj.confirmation_ids = [key for key in obj.confirmation_ids if key in recent]
            # A lone old mistake must not accumulate evidence over an entire mapping run.
            self.objects = [o for o in self.objects if o.confirmed or o.confirmation_ids]
        candidates = [(obj, self.position(obj)) for obj in self.objects]
        candidates = [(obj, pos) for obj, pos in candidates if pos is not None]
        world = [transform_point(self.poses[node_id], d['position']) for d in detections]
        cost = np.full((len(detections), len(candidates)), 1e6)
        for i, detection in enumerate(detections):
            for j, (obj, position) in enumerate(candidates):
                different_label = detection['label'] != obj.label
                if different_label and not self.unique_objects:
                    continue
                distance = float(np.linalg.norm(world[i] - position))
                # Larger visible objects tolerate a modest viewpoint-dependent center shift.
                extent = np.abs(self.poses[node_id][:3, :3]) @ np.asarray(detection['extent'])
                previous_observation = obj.observations[-1]
                previous_pose = self.poses.get(previous_observation.node_id, np.eye(4))
                previous = np.abs(previous_pose[:3, :3]) @ np.asarray(previous_observation.extent)
                if self.unique_objects and different_label:
                    if (detection['label'] in self.dynamic_classes) != obj.dynamic:
                        continue
                    if not self.same_geometry(world[i], extent, position, self.extent(obj)):
                        continue
                gate = self.association_distance + min(0.25, float(np.linalg.norm(extent)) * 0.15)
                if distance > gate:
                    continue
                overlap = np.maximum(0, np.minimum(world[i]+extent/2, position+previous/2)
                                     - np.maximum(world[i]-extent/2, position-previous/2))
                intersection = float(np.prod(overlap))
                union = float(np.prod(extent)+np.prod(previous)-intersection)
                iou = intersection / max(union, 1e-9)
                cost[i, j] = distance / gate + 0.2 * (1-iou)
        matches = {}
        if cost.size:
            rows, cols = linear_sum_assignment(cost)
            matches = {int(i): candidates[int(j)][0] for i, j in zip(rows, cols) if cost[i, j] < 1e5}
        observed = []
        for i, detection in enumerate(detections):
            obj = matches.get(i)
            if obj is None:
                obj = ObjectTrack(self.next_id, detection['label'],
                                  detection['label'] in self.dynamic_classes)
                self.next_id += 1
                self.objects.append(obj)
            observation = Observation(node_id, stamp, detection['position'], detection['extent'],
                                      detection['confidence'],
                                      detection['confidence'] / max(0.02, detection['sigma'])**2,
                                      detection['label'] if self.unique_objects else '')
            # Moving objects use their latest location, never a trail averaged into a map.
            if obj.dynamic:
                obj.observations = [observation]
            else:
                obj.observations.append(observation)
                obj.observations = obj.observations[-self.max_observations:]
            obj.last_seen = now
            self.choose_label(obj)
            if self.confirmation_window:
                obj.confirmation_ids.append(node_id)
                required = self.dynamic_min_observations if obj.dynamic else self.min_observations
                obj.confirmed = obj.confirmed or len(set(obj.confirmation_ids)) >= required
            observed.append(obj)
        for obj in observed:
            obj.distinct_ids = sorted(set(obj.distinct_ids) |
                                     {other.object_id for other in observed if other is not obj})
        self.nodes[node_id] = stamp
        return result

    def expire(self, now=None):
        now = time.monotonic() if now is None else now
        self.objects = [o for o in self.objects if not o.dynamic or now-o.last_seen <= self.dynamic_ttl]

    def visible_objects(self, now=None):
        self.expire(now)
        return [obj for obj in self.objects if self.position(obj) is not None and
                (obj.confirmed if self.confirmation_window else
                 (obj.dynamic or len({o.node_id for o in obj.observations}) >= self.min_observations))]

    def to_dict(self):
        objects = [asdict(o) for o in self.objects if not o.dynamic]
        state = dict(schema=1, map_uuid=self.map_uuid, next_id=self.next_id,
                     nodes=self.nodes, objects=objects)
        if self.unique_objects:
            state['identity_version'] = 1
        else:
            for obj in objects:
                for obs in obj['observations']:
                    obs.pop('label')
        if self.confirmation_window:
            state['recent_nodes'] = self.recent_nodes
        else:
            # Keep the legacy YOLOv8 serialization readable by the original mapper too.
            for obj in objects:
                obj.pop('confirmation_ids')
                obj.pop('confirmed')
        return state

    def restore(self, state):
        if state.get('schema') != 1:
            raise ValueError('Unsupported semantic database schema')
        self.map_uuid = state['map_uuid']
        self.next_id = state['next_id']
        self.nodes = {int(k): v for k, v in state['nodes'].items()}
        self.recent_nodes = state.get('recent_nodes', [])[-self.confirmation_window:] if self.confirmation_window else []
        self.objects = [ObjectTrack(**{**o, 'observations': [Observation(**v) for v in o['observations']]})
                        for o in state['objects']]
        if self.unique_objects:
            for obj in self.objects:
                for obs in obj.observations:
                    obs.label = obs.label or obj.label


class SemanticStore:
    def __init__(self, path, map_key, config):
        self.path = Path(path)
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self.map_key = str(Path(map_key).resolve())
        self.config = json.dumps(config, sort_keys=True)
        self.connection = sqlite3.connect(str(self.path))
        self.connection.execute('PRAGMA journal_mode=WAL')
        self.connection.execute('PRAGMA synchronous=FULL')
        with self.connection:
            self.connection.execute('CREATE TABLE IF NOT EXISTS epochs ('
                'uuid TEXT PRIMARY KEY, map_key TEXT NOT NULL, config TEXT NOT NULL, '
                'state TEXT NOT NULL, updated REAL NOT NULL)')

    def load(self, state):
        row = self.connection.execute('SELECT map_key, config, state FROM epochs '
                                      'ORDER BY updated DESC LIMIT 1').fetchone()
        if row:
            if row[0] != self.map_key:
                raise ValueError('Semantic database belongs to a different RTAB-Map database')
            if row[1] != self.config:
                raise ValueError('Semantic model/config changed; use a new semantic database path')
            payload = json.loads(row[2])
            if state.unique_objects and payload.get('identity_version') != 1:
                backup = self.path.with_name(self.path.stem + '.before_unique_objects' + self.path.suffix)
                if not backup.exists():
                    with sqlite3.connect(str(backup)) as destination:
                        self.connection.backup(destination)
            state.restore(payload)

    def save(self, state):
        payload = json.dumps(state.to_dict(), allow_nan=False)
        with self.connection:
            self.connection.execute('INSERT OR REPLACE INTO epochs VALUES (?, ?, ?, ?, ?)',
                (state.map_uuid, self.map_key, self.config, payload, time.time()))

    def close(self):
        self.connection.close()
