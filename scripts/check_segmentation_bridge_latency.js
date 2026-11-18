/* Read-only timing of image delivery through the actual Foxglove WebSocket bridge. */
const fs = require('node:fs'), path = require('node:path'), vm = require('node:vm');
const root = path.resolve(__dirname, '..');
const vendor = require('node:module').createRequire(path.join(root, '.tools/miti/root/usr/lib/miti/package.json'));
const WS = vendor('next/dist/compiled/ws'), React = vendor('react');
const {renderToStaticMarkup} = vendor('react-dom/server.browser');
const factory = vm.runInNewContext(fs.readFileSync(path.join(root, 'config/miti_segmentation.js'), 'utf8') + ';rtabmapSegmentationFactory');
const panel = factory(React), subscriptions = new Map();
const port = Number(process.argv[2] || 8765), seconds = Number(process.argv[3] || 20);
const output = path.resolve(process.argv[4] || path.join(root, 'logs/segmentation_validation/latency_bridge.json'));
if (!Number.isInteger(port) || port < 1 || port > 65535 || !(seconds > 0 && seconds <= 60)) throw Error('Invalid port/duration');
const topics = new Map([['/miti/segmentation/view', 'rgb'], ['/miti/segmentation/fusion_view', 'fusion']]);
const samples = {rgb: [], fusion: []}, latest = {};
const socket = new WS('ws://127.0.0.1:' + port, ['foxglove.sdk.v1', 'foxglove.websocket.v1']);
let timer, finished = false;
function quantile(values, fraction) {
  if (!values.length) return null;
  const sorted = [...values].sort((a, b) => a - b), index = (sorted.length - 1) * fraction;
  const low = Math.floor(index), high = Math.ceil(index);
  return sorted[low] + (sorted[high] - sorted[low]) * (index - low);
}
function finish(error) {
  if (finished) return;
  finished = true;
  clearTimeout(timer);
  socket.close();
  if (error) { console.error(error.message); process.exitCode = 1; return; }
  const views = {};
  for (const kind of ['rgb', 'fusion']) {
    const rows = samples[kind];
    if (rows.length < 2) return finishMissing(kind);
    const intervals = rows.slice(1).map((row, i) => row.arrival - rows[i].arrival);
    views[kind] = {updates: rows.length, median_update_interval_s: quantile(intervals, .5),
      median_publish_to_websocket_s: quantile(rows.map(r => r.delivery), .5),
      p95_publish_to_websocket_s: quantile(rows.map(r => r.delivery), .95),
      median_source_age_s: quantile(rows.map(r => r.sourceAge).filter(v => Number.isFinite(v)), .5)};
  }
  const report = {passed: true, seconds, transport: 'foxglove_cdr', views,
    native_parser_verified: true, react_render_verified: true, browser_visual_verified: false};
  fs.mkdirSync(path.dirname(output), {recursive: true});
  fs.writeFileSync(output, JSON.stringify(report, null, 2) + '\n');
  console.log(JSON.stringify(report, null, 2));
}
function finishMissing(kind) {
  console.error('Too few live ' + kind + ' bridge frames'); process.exitCode = 1;
}
socket.on('open', () => { timer = setTimeout(() => finish(), seconds * 1000); });
socket.on('error', error => finish(error));
socket.on('message', (data, binary) => {
  if (finished) return;
  try {
    if (!binary) {
      const message = JSON.parse(data.toString());
      if (message.op !== 'advertise') return;
      const added = [];
      for (const channel of message.channels) if (topics.has(channel.topic)) {
        const id = subscriptions.size + 1;
        subscriptions.set(id, topics.get(channel.topic));
        added.push({id, channelId: channel.id});
      }
      if (added.length) socket.send(JSON.stringify({op: 'subscribe', subscriptions: added}));
      return;
    }
    if (data[0] !== 1 || data.length < 21) return;
    const kind = subscriptions.get(data.readUInt32LE(1));
    if (!kind) return;
    const payload = data.subarray(13), little = Boolean(payload[1] & 1);
    const length = little ? payload.readUInt32LE(4) : payload.readUInt32BE(4);
    if (length < 1 || length > payload.length - 8) throw Error('Invalid CDR length');
    const message = {data: payload.subarray(8, 8 + length - 1).toString('utf8')};
    const view = kind === 'rgb' ? panel.parseSnapshot(message) : panel.parseFusion(message);
    if (!view || !Number.isFinite(view.published_at)) throw Error('Native panel rejected bridged ' + kind);
    const key = view.session_id + ':' + view.sequence + ':' + (view.revision || 0);
    if (latest[kind] === key) return;
    latest[kind] = key;
    const delivery = Date.now() / 1000 - view.published_at;
    samples[kind].push({arrival: performance.now() / 1000, delivery,
      sourceAge: Number.isFinite(view.source_age_s) ? view.source_age_s + delivery : null});
    if (samples[kind].length === 1) {
      const html = renderToStaticMarkup(React.createElement(kind === 'rgb' ? panel.View : panel.FusionComparison,
        {sample: {snapshot: view, receivedAt: 0}, now: 0}));
      if (!html.includes('data:image/jpeg;base64,') || !html.includes('画面帧龄')) throw Error('Image/age render failed');
    }
  } catch (error) { finish(error); }
});
