"""YAMNet TFLite sound events, independent of target embeddings and geometry."""
import csv
import hashlib
import math
from pathlib import Path
import time
import zipfile

import numpy as np

MODEL_URL = 'https://storage.googleapis.com/mediapipe-models/audio_classifier/yamnet/float32/1/yamnet.tflite'
MODEL_SHA256 = '4d8b4a53282dc83ef04e3e7dbc4fbc98082e34e44ed798e16c3a0cdd4c584faf'
CLASS_MAP_SHA256 = 'cdf24d193e196d9e95912a2667051ae203e92a2ba09449218ccb40ef787c6df2'
SCHEMA = 'yamnet_audio_events/1'
CLASS_COUNT = 521
SAMPLE_RATE = 16000
PATCH_SAMPLES = 15600
HOP_SAMPLES = 7680

# Preserve all 521 official names; translate common patrol and background events.
ZH_NAMES = {
    'Speech': '人声', 'Child speech, kid speaking': '儿童说话',
    'Conversation': '交谈', 'Narration, monologue': '旁白／独白',
    'Shout': '喊叫', 'Yell': '大声叫喊', 'Screaming': '尖叫', 'Whispering': '低声说话',
    'Laughter': '笑声', 'Cough': '咳嗽', 'Breathing': '呼吸',
    'Walk, footsteps': '行走／脚步', 'Run': '跑步', 'Dog': '狗叫相关声音',
    'Bark': '犬吠', 'Howl': '嚎叫', 'Bow-wow': '汪汪声', 'Growling': '低吼',
    'Cat': '猫叫相关声音', 'Meow': '猫叫', 'Purr': '猫呼噜', 'Caterwaul': '猫嚎叫',
    'Animal': '动物声', 'Domestic animals, pets': '宠物声', 'Bird': '鸟鸣相关声音',
    'Music': '音乐', 'Singing': '歌唱', 'Silence': '安静',
    'Vehicle': '车辆声', 'Motor vehicle (road)': '道路机动车', 'Car': '汽车',
    'Truck': '卡车', 'Bus': '公交车', 'Motorcycle': '摩托车', 'Engine': '发动机',
    'Idling': '发动机怠速', 'Engine starting': '发动机启动',
    'Vehicle horn, car horn, honking': '车辆鸣笛', 'Alarm': '警报声',
    'Siren': '警笛', 'Smoke detector, smoke alarm': '烟雾报警器声',
    'Fire alarm': '火灾报警器声', 'Buzzer': '蜂鸣器', 'Door': '门相关声音',
    'Doorbell': '门铃', 'Knock': '敲击', 'Slam': '关门／猛撞', 'Bang': '砰响',
    'Glass': '玻璃相关声音', 'Shatter': '碎裂', 'Breaking': '破裂',
    'Explosion': '爆炸声', 'Gunshot, gunfire': '枪声', 'Fireworks': '烟花',
    'Firecracker': '爆竹', 'Wind': '风声', 'Rain': '雨声', 'Water': '水声',
    'Noise': '噪声', 'White noise': '白噪声', 'Pink noise': '粉红噪声',
    'Inside, small room': '小房间环境声', 'Inside, large room or hall': '大厅环境声',
    'Outside, urban or manmade': '城市室外环境声', 'Outside, rural or natural': '自然室外环境声',
}


def load_class_map(path):
    with Path(path).open(newline='', encoding='utf-8') as stream:
        rows = list(csv.DictReader(stream))
    if (len(rows) != CLASS_COUNT or
            any(row.get('index') != str(i) or not row.get('mid') or
                not row.get('display_name') for i, row in enumerate(rows)) or
            len({row['mid'] for row in rows}) != CLASS_COUNT):
        raise ValueError('YAMNet requires the ordered official 521-class map')
    return [dict(index=i, mid=row['mid'], label=row['display_name'],
                 label_zh=ZH_NAMES.get(row['display_name'], row['display_name']))
            for i, row in enumerate(rows)]


def prepare_waveform(samples, sample_rate):
    """Validate normalized PCM, mix channels and resample without automatic gain."""
    if type(sample_rate) is not int or not 8000 <= sample_rate <= 192000:
        raise ValueError('Audio sample rate must be an integer in [8000, 192000]')
    data = np.asarray(samples, dtype=np.float32)
    if data.ndim == 1:
        data = data[:, None]
    if (data.ndim != 2 or not 1 <= data.shape[1] <= 64 or len(data) == 0 or
            len(data) > sample_rate * 10 or not np.isfinite(data).all() or
            np.max(np.abs(data)) > 1.00001):
        raise ValueError('Require 0 < duration <= 10 s of finite normalized PCM')
    mono = data.mean(axis=1, dtype=np.float32)
    rms = float(np.sqrt(np.mean(mono.astype(np.float64) ** 2)))
    info = dict(sample_rate_hz=sample_rate, channels=data.shape[1], samples=len(data),
                duration_s=len(data) / sample_rate, rms_dbfs=20 * math.log10(max(rms, 1e-12)),
                peak=float(np.max(np.abs(data))), clipping_fraction=float(np.mean(np.abs(data) >= .999)))
    if sample_rate != SAMPLE_RATE:
        from scipy.signal import resample_poly
        factor = math.gcd(sample_rate, SAMPLE_RATE)
        mono = resample_poly(mono, SAMPLE_RATE // factor, sample_rate // factor)
    # Filter overshoot is clipped to the model's normalized waveform range.
    return np.ascontiguousarray(np.clip(mono, -1, 1), dtype=np.float32), info


class YamnetClassifier:
    """Pinned official model; the TFLite graph includes the log-Mel front end."""
    def __init__(self, model_path, class_map_path, cpu_threads=2, top_k=5):
        from tflite_runtime.interpreter import Interpreter
        if type(cpu_threads) is not int or not 1 <= cpu_threads <= 16:
            raise ValueError('cpu_threads must be in [1, 16]')
        if type(top_k) is not int or not 1 <= top_k <= 20:
            raise ValueError('top_k must be in [1, 20]')
        model_path = Path(model_path)
        self.classes = load_class_map(class_map_path)
        digest = hashlib.sha256(model_path.read_bytes()).hexdigest()
        if digest != MODEL_SHA256:
            raise ValueError('YAMNet model checksum mismatch; run scripts/install_yamnet.sh')
        with zipfile.ZipFile(model_path) as archive:
            labels = archive.read('yamnet_label_list.txt').decode('utf-8').splitlines()
        if labels != [row['label'] for row in self.classes]:
            raise ValueError('Class map order differs from the model metadata')
        self.interpreter = Interpreter(model_path=str(model_path), num_threads=cpu_threads)
        self.interpreter.allocate_tensors()
        inputs, outputs = self.interpreter.get_input_details(), self.interpreter.get_output_details()
        if (len(inputs) != 1 or len(outputs) != 1 or inputs[0]['dtype'] != np.float32 or
                outputs[0]['dtype'] != np.float32 or tuple(inputs[0]['shape']) != (PATCH_SAMPLES,) or
                tuple(outputs[0]['shape']) != (1, CLASS_COUNT)):
            raise ValueError('Unexpected official YAMNet TFLite tensor contract')
        self.input_index, self.output_index = inputs[0]['index'], outputs[0]['index']
        self.top_k = top_k
        self.model = dict(name='YAMNet', runtime='tflite-runtime', sha256=digest,
                          classes=CLASS_COUNT, sample_rate_hz=SAMPLE_RATE,
                          patch_seconds=PATCH_SAMPLES / SAMPLE_RATE, hop_seconds=HOP_SAMPLES / SAMPLE_RATE,
                          score_kind='independent_class_scores', aggregation='mean_over_patches')

    def ranking(self, scores):
        return [dict(self.classes[int(i)], score=float(scores[i]))
                for i in np.argsort(-np.asarray(scores), kind='stable')[:self.top_k]]

    def classify(self, samples, sample_rate):
        started = time.perf_counter()
        waveform, info = prepare_waveform(samples, sample_rate)
        padding = max(0, PATCH_SAMPLES - len(waveform))
        if padding:
            waveform = np.pad(waveform, (0, padding))
        # Anchor complete overlapping patches to the latest sample in a live window.
        starts = list(range(len(waveform) - PATCH_SAMPLES, -1, -HOP_SAMPLES))[::-1]
        scores = []
        for offset in starts:
            self.interpreter.set_tensor(self.input_index, waveform[offset:offset + PATCH_SAMPLES])
            self.interpreter.invoke()
            row = self.interpreter.get_tensor(self.output_index)[0].copy()
            if not np.isfinite(row).all() or np.any(row < 0) or np.any(row > 1):
                raise ValueError('Invalid YAMNet classification scores')
            scores.append(row)
        mean = np.mean(scores, axis=0, dtype=np.float64)
        info.update(model_sample_rate_hz=SAMPLE_RATE, padded_samples=padding,
                    patches=len(scores), analyzed_duration_s=info['duration_s'])
        return dict(model=self.model, scores=mean.tolist(), top_k=self.ranking(mean),
                    audio=info, processing_s=time.perf_counter() - started)


class PatrolEventTracker:
    """Configurable consecutive-window observations; scores are not threat probabilities."""
    def __init__(self, settings, classes):
        if settings.get('schema') != 1:
            raise ValueError('Unsupported YAMNet patrol settings schema')
        self.min_rms_dbfs = float(settings.get('min_rms_dbfs', -60))
        self.reset_gap_s = float(settings.get('reset_gap_s', 3))
        if (not math.isfinite(self.min_rms_dbfs) or not -120 <= self.min_rms_dbfs <= 0 or
                not math.isfinite(self.reset_gap_s) or not 0 < self.reset_gap_s <= 30):
            raise ValueError('Invalid patrol audio level or continuity settings')
        names = {row['label']: row['index'] for row in classes}
        self.rules = []
        rules = settings.get('rules')
        if not isinstance(rules, list) or not rules or len(rules) > 32:
            raise ValueError('Require 1..32 patrol sound rules')
        keys = set()
        for rule in rules:
            key, label = rule.get('key'), rule.get('label_zh')
            threshold, count = rule.get('threshold'), rule.get('confirm_windows')
            labels = rule.get('classes')
            if (not isinstance(key, str) or not key or key in keys or
                    not isinstance(label, str) or not label or type(threshold) not in (int, float) or
                    not math.isfinite(threshold) or not 0 < threshold <= 1 or
                    type(count) is not int or not 1 <= count <= 10 or not isinstance(labels, list) or
                    not labels or any(name not in names for name in labels)):
                raise ValueError('Invalid patrol sound rule: ' + str(key))
            keys.add(key)
            self.rules.append(dict(rule, indices=[names[name] for name in labels]))
        self.classes = classes
        self.last_stamp = None
        self.hits, self.event_ids = {}, {}
        self.sequence = 0

    def reset(self):
        self.last_stamp = None
        self.hits.clear()
        self.event_ids.clear()

    def update(self, scores, stamp, rms_dbfs):
        data = np.asarray(scores, dtype=float)
        if (data.shape != (CLASS_COUNT,) or not np.isfinite(data).all() or
                np.any(data < 0) or np.any(data > 1) or not math.isfinite(stamp) or
                not math.isfinite(rms_dbfs)):
            raise ValueError('Invalid patrol classification input')
        if self.last_stamp is not None and stamp <= self.last_stamp:
            raise ValueError('Patrol observations must have increasing acquisition times')
        if self.last_stamp is not None and stamp - self.last_stamp > self.reset_gap_s:
            self.reset()
        self.last_stamp = stamp
        events = []
        for rule in self.rules:
            index = max(rule['indices'], key=lambda i: data[i])
            score = float(data[index])
            if score < rule['threshold'] or rms_dbfs < self.min_rms_dbfs:
                self.hits.pop(rule['key'], None)
                self.event_ids.pop(rule['key'], None)
                continue
            count = self.hits.get(rule['key'], 0) + 1
            self.hits[rule['key']] = count
            confirmed = count >= rule['confirm_windows']
            onset = confirmed and rule['key'] not in self.event_ids
            if onset:
                self.sequence += 1
                self.event_ids[rule['key']] = f'audio-{self.sequence:06d}'
            events.append(dict(key=rule['key'], label_zh=rule['label_zh'], score=score,
                               threshold=rule['threshold'], status='confirmed' if confirmed else 'candidate',
                               consecutive_windows=count, event_id=self.event_ids.get(rule['key']),
                               onset=onset, source_class=self.classes[index]['label'],
                               observation='sound_event', stamp=stamp))
        return events


def make_snapshot(result, *, sequence, stamp, source, patrol_events, mode='live',
                  published_at=None, input_age_s=None):
    """ROS/browser contract for classification without invented source positions."""
    return dict(schema=SCHEMA, mode=mode, sequence=sequence, stamp=float(stamp),
                published_at=time.time() if published_at is None else published_at,
                source=source, input_age_s=input_age_s, **result, patrol_events=patrol_events,
                source_localization=dict(available=False, target_id=None, position_m=None))


def read_wav(path):
    from scipy.io import wavfile
    rate, samples = wavfile.read(path)
    if np.issubdtype(samples.dtype, np.signedinteger):
        samples = samples.astype(np.float32) / float(2 ** (np.iinfo(samples.dtype).bits - 1))
    elif samples.dtype == np.uint8:
        samples = (samples.astype(np.float32) - 128) / 128
    elif np.issubdtype(samples.dtype, np.floating):
        samples = samples.astype(np.float32)
    else:
        raise ValueError('Unsupported WAV PCM format')
    return int(rate), samples
