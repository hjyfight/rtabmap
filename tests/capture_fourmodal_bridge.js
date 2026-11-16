/* Capture real Foxglove CDR frames for the isolated ROS feature integration test. */
const fs = require('node:fs');
const path = require('node:path');
const {createRequire} = require('node:module');
const root = path.resolve(__dirname, '..');
const vendorRequire = createRequire(path.join(root, '.tools/miti/root/usr/lib/miti/package.json'));
const WS = vendorRequire('next/dist/compiled/ws');
const socket = new WS('ws://127.0.0.1:' + process.argv[2], ['foxglove.sdk.v1', 'foxglove.websocket.v1']);
const wanted = new Set(['/miti/fourmodal_features', '/miti/fourmodal_status', '/miti/audio_spectrum/image_raw', ...process.argv.slice(4)]);
const subscriptions = new Map(), frames = {};
const timer = setTimeout(() => {
  console.error('Missing feature bridge frames: ' + [...wanted].filter(t => !frames[t]).join(', '));
  socket.terminate(); process.exit(1);
}, 180000);
socket.on('error', error => { console.error(error.message); process.exit(1); });
socket.on('message', (data, binary) => {
  if (!binary) {
    const message = JSON.parse(data.toString());
    if (message.op !== 'advertise') return;
    const added = [];
    for (const channel of message.channels) {
      if (wanted.has(channel.topic) && ![...subscriptions.values()].includes(channel.topic)) {
        const id = subscriptions.size + 1;
        subscriptions.set(id, channel.topic);
        added.push({id, channelId: channel.id});
      }
    }
    if (added.length) socket.send(JSON.stringify({op: 'subscribe', subscriptions: added}));
  } else if (data[0] === 1 && data.length > 13) {
    const topic = subscriptions.get(data.readUInt32LE(1));
    if (topic) frames[topic] = data.subarray(13).toString('base64');
    if ([...wanted].every(t => frames[t])) {
      fs.writeFileSync(process.argv[3], JSON.stringify(frames));
      clearTimeout(timer); socket.close(); process.exit(0);
    }
  }
});
