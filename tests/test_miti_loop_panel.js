const test = require('node:test');
const assert = require('node:assert/strict');
const fs = require('node:fs');
const path = require('node:path');
const vm = require('node:vm');
const {createRequire} = require('node:module');
const root = path.resolve(__dirname, '..');
const vendorRequire = createRequire(path.join(root, '.tools/miti/root/usr/lib/miti/package.json'));
const React = vendorRequire('react');
const {renderToStaticMarkup} = vendorRequire('react-dom/server.browser');
const source = fs.readFileSync(path.join(root, 'config/miti_loop_status.js'), 'utf8');
const panel = vm.runInNewContext(source + '; rtabmapLoopStatusFactory')(React);

function snapshot(overrides = {}) {
  return {schema: 1, monitor_id: 'test-session', published_at: 1793599200,
    display_state: 'SUCCESS', title: '回环成功', phase: 'CONFIRMED', phase_label: '已接受回环约束',
    counts: {total: 3, global: 2, proximity: 1}, nodes: 42,
    last_success: {type: 'GLOBAL_LOOP', ref_id: 40, target_id: 5, time: 1793599200},
    rates_hz: {lidar: 10, rgb: 6, depth: 6, slam_info: 1},
    slam_age_seconds: 0, heartbeat_age_seconds: 0, success_hold_remaining: 8, ...overrides};
}

test('decodes the actual std_msgs/String envelope and rejects malformed snapshots', () => {
  assert.equal(panel.parseStatus({data: JSON.stringify(snapshot())}).counts.total, 3);
  for (const message of [{}, {data: '{}'}, {data: '{'}, {data: JSON.stringify(snapshot({schema: 2}))},
    {data: JSON.stringify(snapshot({last_success: null}))},
    {data: JSON.stringify(snapshot({counts: {total: 20, global: 2, proximity: 1}}))}]) {
    assert.equal(panel.parseStatus(message), null);
  }
});

test('waiting and missing diagnostics are explicit, without invented zero counts', () => {
  assert.equal(panel.deriveView(null, 0, 0, false).title, '等待连接');
  const view = panel.deriveView(null, 0, 0, true);
  const html = renderToStaticMarkup(React.createElement(panel.View, {view, onSoundChange() {}}));
  assert.match(html, /等待回环诊断/);
  assert.match(html, /尚无成功回环记录/);
  assert.match(html, /—/);
});

test('source loss overrides success and keeps historical counters and accepted pair', () => {
  assert.equal(panel.deriveView(snapshot(), 0, 2000, true).mode, 'success');
  const lost = panel.deriveView(snapshot(), 0, 3000, true);
  assert.equal(lost.mode, 'stale');
  assert.equal(lost.snapshot.counts.total, 3);
  assert.equal(panel.deriveView(snapshot(), 0, 0, false).mode, 'offline');
  const html = renderToStaticMarkup(React.createElement(panel.View, {view: lost, onSoundChange() {}}));
  assert.match(html, /诊断更新已中断/);
  assert.match(html, /#40 ↔ #5/);
  assert.doesNotMatch(html, />10\.0</);
});

test('SLAM timeout and elapsed success hold have the same priority as Qt', () => {
  const stale = panel.deriveView(snapshot({slam_age_seconds: 7}), 0, 600, true);
  assert.equal(stale.title, 'SLAM 数据已中断');
  const expired = panel.deriveView(snapshot({success_hold_remaining: 0.4}), 0, 500, true);
  assert.equal(expired.mode, 'searching');
  const candidate = panel.deriveView(snapshot({display_state: 'SEARCHING',
    phase: 'CANDIDATE_REJECTED', phase_label: '候选被后端拒绝'}), 0, 0, true);
  assert.equal(candidate.mode, 'searching');
});

test('subscribes through MITI bridge client and disposes the exact listener', () => {
  let listener, removed, sample, failure, calls = 0;
  const client = {
    subscribe(topic, callback) { assert.equal(topic, '/miti/loop_status'); listener = callback; },
    unsubscribe(topic, callback) { removed = callback; }
  };
  const dispose = panel.subscribe(client, '/miti/loop_status', (data, received) => {
    sample = data; assert.equal(received, 123); calls++;
  }, error => { failure = error; }, () => 123);
  listener({data: JSON.stringify(snapshot())});
  assert.equal(sample.counts.total, 3);
  listener({data: '{}'});
  assert.match(failure, /格式不兼容/);
  dispose();
  assert.equal(removed, listener);
  listener({data: JSON.stringify(snapshot())});
  assert.equal(calls, 1);
});

test('renders values, time, accessible controls and escaped backend text', () => {
  const view = panel.deriveView(snapshot({phase_label: '<script>bad()</script>'}), 0, 0, true);
  const html = renderToStaticMarkup(React.createElement(panel.View, {view, onSoundChange() {}}));
  for (const label of ['回环成功', '本次监控成功', '全局回环', '空间邻近', '最近成功',
    '#40 ↔ #5', '当前图节点 42', '实时更新', '回环成功提示音', '10.0']) assert.ok(html.includes(label), label);
  assert.match(html, /&lt;script&gt;/);
  assert.doesNotMatch(html, /<script>/);
  assert.notEqual(panel.successKey(snapshot()), panel.successKey(snapshot({monitor_id: 'new-session'})));
});

test('sound skips retained history, duplicates and monitor restarts', () => {
  const first = panel.notification(null, snapshot());
  assert.equal(first.play, false);
  assert.equal(panel.notification(first.cursor, snapshot()).play, false);
  const changed = snapshot({last_success: {type: 'GLOBAL_LOOP', ref_id: 41, target_id: 5, time: 1793599201}});
  assert.equal(panel.notification(first.cursor, changed).play, true);
  assert.equal(panel.notification(first.cursor, snapshot({monitor_id: 'restart'})).play, false);
  assert.equal(panel.notification(null, null).cursor, null);
});
