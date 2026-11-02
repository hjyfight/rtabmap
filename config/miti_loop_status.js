/* Project-owned native MITI panel. The installer supplies MITI's existing React instance. */
function rtabmapLoopStatusFactory(React) {
  'use strict';
  const h = React.createElement;
  const modes = new Set(['WAITING_FOR_SLAM', 'SEARCHING', 'SUCCESS', 'SLAM_TIMEOUT']);
  const finite = value => typeof value === 'number' && Number.isFinite(value);
  const count = value => Number.isSafeInteger(value) && value >= 0;
  const age = value => value === null || (finite(value) && value >= 0);

  function parseStatus(message) {
    try {
      if (typeof message?.data !== 'string' || message.data.length > 65536) return null;
      const value = JSON.parse(message.data);
      if (!value || value.schema !== 1 || typeof value.monitor_id !== 'string' ||
          !finite(value.published_at) || !modes.has(value.display_state) ||
          typeof value.phase_label !== 'string' || !count(value.nodes) ||
          !value.counts || !['total', 'global', 'proximity'].every(k => count(value.counts[k])) ||
          value.counts.total !== value.counts.global + value.counts.proximity ||
          !age(value.slam_age_seconds) || !age(value.heartbeat_age_seconds) ||
          !finite(value.success_hold_remaining) || value.success_hold_remaining < 0 ||
          !value.rates_hz || typeof value.rates_hz !== 'object') return null;
      if (Object.values(value.rates_hz).some(v => !finite(v) || v < 0)) return null;
      const last = value.last_success;
      if (last !== null && (!last || !['GLOBAL_LOOP', 'PROXIMITY_ICP'].includes(last.type) ||
          !count(last.ref_id) || last.ref_id <= 0 || !count(last.target_id) || last.target_id <= 0 ||
          !finite(last.time) || Math.abs(last.time) > 8640000000000)) return null;
      if (value.display_state === 'SUCCESS' && last === null) return null;
      return value;
    } catch (_) {
      return null;
    }
  }

  function deriveView(snapshot, receivedAt, now, connected, error = '') {
    const elapsed = Math.max(0, (now - receivedAt) / 1000);
    let mode, title, detail;
    if (!connected) {
      mode = 'offline'; title = '等待连接'; detail = '连接 MITI 数据源后自动更新';
    } else if (error) {
      mode = 'stale'; title = '状态数据不可用'; detail = error;
    } else if (!snapshot) {
      mode = 'waiting'; title = '等待回环诊断'; detail = '启动融合或语义建图后自动更新';
    } else if (elapsed >= 3) {
      mode = 'stale'; title = '诊断更新已中断'; detail = '保留最近记录，等待诊断恢复';
    } else if (snapshot.display_state === 'SLAM_TIMEOUT' ||
               (snapshot.slam_age_seconds !== null && snapshot.slam_age_seconds + elapsed >= 7.5)) {
      mode = 'stale'; title = 'SLAM 数据已中断'; detail = '检查传感器与同步链路';
    } else if (snapshot.display_state === 'SUCCESS' && snapshot.success_hold_remaining > elapsed) {
      mode = 'success'; title = '回环成功'; detail = snapshot.phase_label;
    } else if (snapshot.display_state === 'WAITING_FOR_SLAM') {
      mode = 'waiting'; title = '等待建图数据'; detail = snapshot.phase_label;
    } else {
      mode = 'searching'; title = '回环检测中'; detail = snapshot.phase_label;
    }
    return {mode, title, detail, snapshot, elapsed,
            fresh: Boolean(connected && snapshot && !error && elapsed < 3)};
  }

  function subscribe(client, topic, onData, onError, clock = () => performance.now()) {
    let disposed = false;
    const receive = message => {
      if (disposed) return;
      const snapshot = parseStatus(message);
      if (snapshot) onData(snapshot, clock());
      else onError('收到的回环状态格式不兼容');
    };
    client.subscribe(topic, receive);
    return () => { disposed = true; client.unsubscribe(topic, receive); };
  }

  function successKey(snapshot) {
    const last = snapshot?.last_success;
    return last ? [snapshot.monitor_id, last.type, last.ref_id, last.target_id, last.time].join(':') : '';
  }

  function notification(previous, snapshot) {
    if (!snapshot) return {cursor: previous, play: false};
    const key = successKey(snapshot);
    return {cursor: {monitor: snapshot.monitor_id, key},
            play: Boolean(previous && previous.monitor === snapshot.monitor_id && key && previous.key !== key)};
  }

  const css = `
    .rtab-loop {height:100%;min-height:0;min-width:0;overflow:auto;padding:14px 16px;
      color:#e4edf5;background:linear-gradient(135deg,#131f2c,#111923);font:13px/1.45 system-ui,sans-serif;
      font-variant-numeric:tabular-nums;container-type:inline-size;}
    .rtab-loop * {box-sizing:border-box;}
    .rtab-loop-header {display:flex;align-items:center;justify-content:space-between;gap:8px;}
    .rtab-loop-status {display:flex;align-items:center;gap:8px;font-size:17px;font-weight:600;}
    .rtab-loop-dot {width:8px;height:8px;border-radius:50%;background:currentColor;flex:none;}
    .rtab-loop-searching {color:#8dbdff;} .rtab-loop-success {color:#63dab0;}
    .rtab-loop-waiting {color:#edc47f;} .rtab-loop-stale {color:#f19388;} .rtab-loop-offline {color:#a5b4c4;}
    .rtab-loop-detail {color:#a4b4c5;font-size:12px;margin:5px 0 12px;overflow-wrap:anywhere;}
    .rtab-loop-sound {display:flex;align-items:center;gap:5px;white-space:nowrap;color:#a4b4c5;font-size:11px;}
    .rtab-loop-sound input {accent-color:#63dab0;margin:0;}
    .rtab-loop-counts {display:grid;grid-template-columns:1.5fr 1fr 1fr;gap:12px;align-items:center;}
    .rtab-loop-caption {color:#9cafc2;font-size:11px;}
    .rtab-loop-total {font-size:36px;line-height:1.15;letter-spacing:-.04em;font-weight:600;}
    .rtab-loop-count {font-size:23px;line-height:1.2;font-weight:500;}
    .rtab-loop-last {margin:12px 0;padding:10px 0;border-top:1px solid #2a3848;border-bottom:1px solid #2a3848;}
    .rtab-loop-pair {font:600 18px/1.45 ui-monospace,monospace;color:#dceafa;overflow-wrap:anywhere;}
    .rtab-loop-last-meta {color:#a4b4c5;font-size:11px;margin-top:2px;}
    .rtab-loop-empty {color:#8296aa;font-size:12px;margin-top:4px;}
    .rtab-loop-rates {display:grid;grid-template-columns:repeat(4,minmax(0,1fr));gap:8px;}
    .rtab-loop-rate {font-size:16px;margin-top:2px;white-space:nowrap;}
    .rtab-loop-unit {font-size:10px;color:#8296aa;margin-left:3px;}
    .rtab-loop-footer {display:flex;flex-wrap:wrap;justify-content:space-between;gap:5px;
      color:#8296aa;font-size:10px;margin-top:12px;}
    .rtab-loop-muted {opacity:.55;}
    @container (max-width:270px) {.rtab-loop-counts {gap:6px}.rtab-loop-status {font-size:15px}
      .rtab-loop-rates {grid-template-columns:repeat(2,minmax(0,1fr));}}
    @media (pointer:coarse) {.rtab-loop-sound {min-height:44px;}}
  `;

  function View({view, soundEnabled = false, onSoundChange, soundError = ''}) {
    const data = view.snapshot;
    const last = data?.last_success;
    const counts = data?.counts;
    const time = last ? new Intl.DateTimeFormat('zh-CN', {
      timeZone: 'Asia/Shanghai', hour12: false, hour: '2-digit', minute: '2-digit', second: '2-digit'
    }).format(new Date(last.time * 1000)) : '';
    const ratesFresh = view.fresh && data.heartbeat_age_seconds !== null &&
                       data.heartbeat_age_seconds + view.elapsed < 7.5;
    return h('section', {className: 'rtab-loop', 'aria-label': '回环检测状态'},
      h('style', null, css),
      h('div', {className: 'rtab-loop-header'},
        h('div', {className: 'rtab-loop-status rtab-loop-' + view.mode, role: 'status', 'aria-live': 'polite'},
          h('span', {className: 'rtab-loop-dot', 'aria-hidden': true}), view.title),
        h('label', {className: 'rtab-loop-sound'},
          h('input', {type: 'checkbox', checked: soundEnabled, onChange: onSoundChange,
                      'aria-label': '回环成功提示音'}), '提示音')),
      h('p', {className: 'rtab-loop-detail'}, view.detail),
      soundError ? h('p', {className: 'rtab-loop-detail', role: 'alert'}, soundError) : null,
      h('div', {className: 'rtab-loop-counts'},
        h('div', null, h('div', {className: 'rtab-loop-caption'}, '本次监控成功'),
          h('div', {className: 'rtab-loop-total'}, counts?.total ?? '—')),
        h('div', null, h('div', {className: 'rtab-loop-caption'}, '全局回环'),
          h('div', {className: 'rtab-loop-count'}, counts?.global ?? '—')),
        h('div', null, h('div', {className: 'rtab-loop-caption'}, '空间邻近'),
          h('div', {className: 'rtab-loop-count'}, counts?.proximity ?? '—'))),
      h('div', {className: 'rtab-loop-last'},
        h('div', {className: 'rtab-loop-caption'}, '最近成功'),
        last ? h(React.Fragment, null,
          h('div', {className: 'rtab-loop-pair'}, '#' + last.ref_id + ' ↔ #' + last.target_id),
          h('div', {className: 'rtab-loop-last-meta'},
            (last.type === 'GLOBAL_LOOP' ? '全局回环' : '空间邻近回环') + ' · ' + time)) :
          h('div', {className: 'rtab-loop-empty'}, '尚无成功回环记录')),
      h('div', {className: 'rtab-loop-rates' + (ratesFresh ? '' : ' rtab-loop-muted')},
        ...[['lidar', '雷达'], ['rgb', 'RGB'], ['depth', '深度'], ['slam_info', 'SLAM']].map(([key, label]) =>
          h('div', {key}, h('div', {className: 'rtab-loop-caption'}, label),
            h('div', {className: 'rtab-loop-rate'},
              ratesFresh && finite(data.rates_hz[key]) ? data.rates_hz[key].toFixed(1) : '—',
              h('span', {className: 'rtab-loop-unit'}, 'Hz'))))),
      h('div', {className: 'rtab-loop-footer'},
        h('span', null, '当前图节点 ' + (data?.nodes ?? '—')),
        h('span', null, view.fresh ? '实时更新' : data ? '最近状态已保留' : '等待数据')));
  }

  function Component({client, topic = '/miti/loop_status'}) {
    const [sample, setSample] = React.useState(null);
    const [error, setError] = React.useState('');
    const [now, setNow] = React.useState(() => performance.now());
    const [soundEnabled, setSoundEnabled] = React.useState(false);
    const [soundError, setSoundError] = React.useState('');
    const audio = React.useRef(null);
    const lastSoundKey = React.useRef(null);
    React.useEffect(() => {
      if (!client) return;
      setError('');
      try {
        return subscribe(client, topic, (snapshot, receivedAt) => {
          setSample({snapshot, receivedAt}); setError(''); setNow(receivedAt);
        }, setError);
      } catch (_) { setError('无法订阅回环状态'); }
    }, [client, topic]);
    React.useEffect(() => {
      const timer = setInterval(() => setNow(performance.now()), 250);
      return () => clearInterval(timer);
    }, []);
    React.useEffect(() => () => { audio.current?.close().catch(() => {}); }, []);
    const view = deriveView(sample?.snapshot, sample?.receivedAt ?? now, now, Boolean(client), error);
    const key = successKey(sample?.snapshot);
    React.useEffect(() => {
      // A late join, reconnect or page refresh never sounds an old accepted event.
      if (!client) { lastSoundKey.current = null; return; }
      const next = notification(lastSoundKey.current, sample?.snapshot);
      lastSoundKey.current = next.cursor;
      if (!next.play || !soundEnabled || view.mode !== 'success' || !audio.current) return;
      const context = audio.current;
      if (context.state !== 'running') return;
      const oscillator = context.createOscillator(), gain = context.createGain();
      oscillator.frequency.value = 740;
      gain.gain.setValueAtTime(0.07, context.currentTime);
      gain.gain.exponentialRampToValueAtTime(0.001, context.currentTime + 0.18);
      oscillator.connect(gain); gain.connect(context.destination);
      oscillator.start(); oscillator.stop(context.currentTime + 0.18);
      oscillator.onended = () => { oscillator.disconnect(); gain.disconnect(); };
    }, [sample, key, client, soundEnabled, view.mode]);
    async function toggleSound(event) {
      const enabled = event.target.checked;
      setSoundEnabled(enabled); setSoundError('');
      if (!enabled) return;
      lastSoundKey.current = notification(null, sample?.snapshot).cursor;
      try {
        const Audio = window.AudioContext || window.webkitAudioContext;
        if (!audio.current) audio.current = new Audio();
        await audio.current.resume();
      } catch (_) {
        setSoundEnabled(false); setSoundError('浏览器暂时无法播放提示音');
      }
    }
    return h(View, {view, soundEnabled, soundError, onSoundChange: toggleSound});
  }

  return {Component, View, parseStatus, deriveView, subscribe, successKey, notification};
}
