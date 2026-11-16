const fs = require('node:fs');
const path = require('node:path');
const vm = require('node:vm');
const assert = require('node:assert/strict');
const {createRequire} = require('node:module');
const root = path.resolve(__dirname, '..');
const vendorRequire = createRequire(path.join(root, '.tools/miti/root/usr/lib/miti/package.json'));
const React = vendorRequire('react');
const {renderToStaticMarkup} = vendorRequire('react-dom/server.browser');
const panel = vm.runInNewContext(fs.readFileSync(path.join(root, 'config/miti_fourmodal_features.js'), 'utf8') +
  '; rtabmapFourmodalFactory')(React);
const snapshot = panel.parseSnapshot({data: fs.readFileSync(process.argv[2], 'utf8')});
assert.ok(snapshot, 'Actual bridged snapshot rejected by MITI panel');
const html = renderToStaticMarkup(React.createElement(panel.View, {
  view: {data: snapshot, mode: 'live', title: '四模态特征已更新', detail: '隔离测试'},
  onSelected() {}, onIndex() {}}));
assert.match(html, /LiDAR 点云/); assert.match(html, /声学 PCM/); assert.match(html, /log-Mel/); assert.match(html, /4096/);
assert.equal(snapshot.flow_version, 1);
for (const label of ['传感器输入','编码器输出','归一化特征','拼接输出','加权中间值','四路贡献求和'])
  assert.ok(html.includes(label), label);
for (let i = 0; i < 4; i++)
  assert.equal(panel.inspectVector(snapshot, 'raw' + i).values, snapshot.modalities[i].encoder_features);
const jointHtml = renderToStaticMarkup(React.createElement(panel.View, {
  view:{data:snapshot,mode:'live',title:'特征流',detail:''}, selected:'joint', index:4095}));
assert.ok(jointHtml.includes('维度 4095 = ' + snapshot.joint_features[4095].toFixed(6)));
console.log('Actual bridged features parsed and rendered by the native MITI React panel');
