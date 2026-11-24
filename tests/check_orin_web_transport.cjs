/* Read-only LAN validation; does not claim Windows browser visual verification. */
const assert = require('node:assert/strict');
const fs = require('node:fs');
const path = require('node:path');
const {createRequire} = require('node:module');
const root = path.resolve(__dirname, '..');
const vendor = createRequire(path.join(root, '.tools/miti/root/usr/lib/miti/package.json'));
const WS = vendor('next/dist/compiled/ws');
const host = process.argv[2] || '192.168.1.103';
const target = process.argv[3] || path.join(root, 'logs/orin_standalone_validation');
const base = `http://${host}:3001`;
const topics = new Set(['/miti/research/display/semantic_markers',
  '/miti/research/display/clean_map', '/camera/color/image_raw']);

async function snapshots() {
  const result = {};
  for (const task of ['pmf', 'avs', 'rgbnir']) {
    const response = await fetch(`${base}/research/${task}.json`, {signal: AbortSignal.timeout(12000)});
    assert.equal(response.status, 200);
    assert.ok(response.headers.get('date'));
    const data = await response.json();
    assert.equal(data.task, task);
    assert.equal(data.offline_report, false);
    assert.ok(data.session_id);
    assert.ok(data.result?.sequence > 0, `No real ${task} inference result`);
    result[task] = {session_id: data.session_id, published_at: data.published_at,
      producer_age_s: Date.parse(response.headers.get('date')) / 1000 - data.published_at,
      sequence: data.result.sequence, source_stamp: data.result.source_stamp, state: data.state, detail: data.detail};
  }
  return result;
}

function capture() {
  return new Promise((resolve, reject) => {
    const socket = new WS(`ws://${host}:8766`, ['foxglove.sdk.v1', 'foxglove.websocket.v1']);
    const subscriptions = new Map(), samples = {};
    let serverInfo;
    const timer = setTimeout(() => finish(new Error('Missing bridge frames: ' +
      [...topics].filter(topic => !samples[topic]).join(', '))), 45000);
    function finish(error) {
      clearTimeout(timer);
      socket.terminate();
      if (error) reject(error); else resolve({serverInfo, samples, subprotocol: socket.protocol});
    }
    socket.on('error', finish);
    socket.on('message', (data, binary) => {
      try {
        if (!binary) {
          const message = JSON.parse(data.toString());
          if (message.op === 'serverInfo') serverInfo = message;
          if (message.op !== 'advertise') return;
          const added = [];
          for (const channel of message.channels) {
            if (topics.has(channel.topic) && ![...subscriptions.values()].some(c => c.topic === channel.topic)) {
              const id = subscriptions.size + 1;
              subscriptions.set(id, channel);
              added.push({id, channelId: channel.id});
            }
          }
          if (added.length) socket.send(JSON.stringify({op: 'subscribe', subscriptions: added}));
          return;
        }
        if (data[0] !== 1 || data.length <= 13) return;
        const channel = subscriptions.get(data.readUInt32LE(1));
        if (channel && !samples[channel.topic]) samples[channel.topic] = {
          schemaName: channel.schemaName, payload_bytes: data.length - 13,
          cdr: data.subarray(13).toString('base64')};
        if ([...topics].every(topic => samples[topic])) {
          assert.deepEqual(serverInfo?.capabilities, ['connectionGraph']);
          finish();
        }
      } catch (error) { finish(error); }
    });
  });
}

(async () => {
  const response = await fetch(base + '/', {signal: AbortSignal.timeout(12000)});
  assert.equal(response.status, 200);
  const homepage = await response.text();
  assert.ok(homepage.includes('/_next/static/'));
  const manifestResponse = await fetch(base + '/rtabmap-panel-assets.json', {signal: AbortSignal.timeout(12000)});
  assert.equal(manifestResponse.status, 200);
  const manifest = await manifestResponse.json();
  const assets = await Promise.all(manifest.assets.map(async url => {
    const asset = await fetch(new URL(url, base), {signal: AbortSignal.timeout(12000)});
    assert.equal(asset.status, 200);
    return asset.text();
  }));
  const frontend = assets.join('\n');
  assert.ok(frontend.includes('browserClockOffset'), 'Clock conversion missing from served frontend');
  assert.ok(frontend.includes('bridgePort') && frontend.includes('8766'));
  const initial = await snapshots();
  const [bridge] = await Promise.all([capture(), new Promise(resolve => setTimeout(resolve, 15000))]);
  const current = await snapshots();
  for (const task of Object.keys(initial)) {
    assert.equal(current[task].session_id, initial[task].session_id);
    assert.ok(current[task].published_at > initial[task].published_at, `${task} live status stopped advancing`);
    assert.ok(current[task].producer_age_s >= -2 && current[task].producer_age_s < 5, `${task} live status is stale`);
  }
  assert.equal(new Set(Object.values(current).map(task => task.session_id)).size, 1);
  fs.mkdirSync(target, {recursive: true});
  fs.writeFileSync(path.join(target, 'bridge_samples.json'), JSON.stringify(bridge.samples));
  const report = {passed: true, scope: 'web transport and live status; direct sensors are not tested',
    homepage_http_status: response.status, server_date: response.headers.get('date'),
    frontend_clock_conversion: true, url: base + '/', transport: 'HTTP + Foxglove WebSocket from another computer',
    browser_visual_verified: false, windows_browser_verified: false,
    bridge_subprotocol: bridge.subprotocol, bridge_capabilities: bridge.serverInfo.capabilities,
    bridge_topics: Object.fromEntries(Object.entries(bridge.samples).map(([topic, sample]) =>
      [topic, {schemaName: sample.schemaName, payload_bytes: sample.payload_bytes}])),
    inference_advanced: Object.fromEntries(Object.keys(initial).map(task => [task, current[task].sequence > initial[task].sequence])),
    initial, current};
  fs.writeFileSync(path.join(target, 'web_transport.json'), JSON.stringify(report, null, 2) + '\n');
  console.log(JSON.stringify(report));
})().catch(error => {console.error(error); process.exitCode = 1;});
