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
const clip = vm.runInNewContext(fs.readFileSync(path.join(root, 'config/miti_height_clip.js'), 'utf8') +
  '; rtabmapHeightClipFactory')(React);
const identity = [1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1];
const distance = (plane, point) => plane[0] * point[0] + plane[1] * point[1] + plane[2] * point[2] + plane[3];

test('starts disabled and handles negative map heights and invalid saved settings', () => {
  assert.equal(clip.normalize().enabled, false);
  assert.equal(clip.normalize().height, 2);
  assert.equal(clip.normalize({enabled: true, height: -0.5}).height, -0.5);
  assert.equal(clip.normalize({enabled: 'true', height: NaN}).enabled, false);
  assert.equal(clip.normalize({height: Infinity}).height, 2);
});

test('clips by fixed-frame height and includes points exactly at the boundary', () => {
  const plane = clip.makePlane(identity, 2);
  assert.equal(distance(plane, [0, 0, 2]), 0);
  assert.ok(distance(plane, [-8, 9, 2.1]) > 0);
  assert.ok(distance(plane, [6, -4, -0.5]) < 0);
  // A camera point with local z=0 can lie above the cut after its TF is applied.
  // Camera: pitch=90 degrees, translated up 1m; local (-2,0,0) -> fixed (0,0,3).
  assert.ok(distance(plane, [0, 0, 3]) > 0);
});

test('robot-follow translation and rotation do not move the cut relative to the map', () => {
  // Follow transform: scene = (fixed.x+3, -fixed.z-2, fixed.y+7).
  const sceneToFixed = [1, 0, 0, 0, 0, 0, -1, 0, 0, 1, 0, 0, -3, -7, -2, 1];
  const plane = clip.makePlane(sceneToFixed, 2);
  for (const z of [-1, 1, 2, 3, 6]) {
    const scenePoint = [4 + 3, -z - 2, 5 + 7];
    assert.equal(distance(plane, scenePoint), z - 2);
  }
});

function material() {
  return {userData: {}, uniforms: {size: {value: 2}, circle: {value: true}}, needsUpdate: false,
    vertexShader: 'attribute vec4 rgba; varying vec4 vColor; void main() { vColor = rgba; gl_Position = projectionMatrix * modelViewMatrix * vec4(position, 1.0); }',
    fragmentShader: 'varying vec4 vColor; void main() { if (vColor.a <= 0.0) discard; gl_FragColor = vColor; }'};
}

test('changing or disabling the cut only updates uniforms and restores full rendering', () => {
  const shader = material(), originalSize = shader.uniforms.size;
  clip.applyMaterial(shader, {enabled: true, plane: clip.makePlane(identity, 2)});
  const vertex = shader.vertexShader, fragment = shader.fragmentShader;
  assert.match(vertex, /dot\(rtabmapClipPlane, modelMatrix \* vec4\(position, 1\.0\)\)/);
  assert.match(fragment, /rtabmapClipEnabled && rtabmapClipDistance > 0\.0/);
  shader.needsUpdate = false;
  clip.applyMaterial(shader, {enabled: true, plane: clip.makePlane(identity, 1)});
  assert.equal(shader.uniforms.rtabmapClipPlane.value[3], -1);
  assert.equal(shader.needsUpdate, false);
  clip.applyMaterial(shader, {enabled: false, plane: clip.makePlane(identity, 1)});
  assert.equal(shader.uniforms.rtabmapClipEnabled.value, false);
  assert.equal(shader.vertexShader, vertex);
  assert.equal(shader.fragmentShader, fragment);
  assert.equal(shader.uniforms.size, originalSize);
  assert.match(shader.fragmentShader, /gl_FragColor = vColor/);
});

test('separate scene materials can use independent clip settings', () => {
  const a = material(), b = material();
  clip.applyMaterial(a, {enabled: true, plane: clip.makePlane(identity, 1)});
  clip.applyMaterial(b, {enabled: false, plane: clip.makePlane(identity, 4)});
  assert.equal(a.uniforms.rtabmapClipEnabled.value, true);
  assert.equal(b.uniforms.rtabmapClipEnabled.value, false);
  assert.equal(a.uniforms.rtabmapClipPlane.value[3], -1);
  assert.equal(b.uniforms.rtabmapClipPlane.value[3], -4);
});

test('controls expose the switch, slider, numeric height and actual reference frame', () => {
  const props = {settings: {enabled: true, height: 1.25}, frame: 'map', bounds: [-5, 5],
    draft: '1.25', id: 'test', onToggle() {}, onSlide() {}, onDraft() {}, onCommit() {}};
  const html = renderToStaticMarkup(React.createElement(clip.ControlsView, props));
  assert.match(html, /隐藏顶部/);
  assert.match(html, /aria-pressed="true"/);
  assert.match(html, /type="range"/);
  assert.match(html, /type="number"/);
  assert.match(html, /1\.25 m/);
  assert.match(html, /map 坐标系的 Z 高度/);
  const disabled = renderToStaticMarkup(React.createElement(clip.ControlsView, {...props,
    settings: {enabled: false, height: 1.25}}));
  assert.match(disabled, /aria-pressed="false"/);
  assert.doesNotMatch(disabled, /type="range"/);
});
