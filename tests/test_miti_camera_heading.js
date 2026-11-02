const test = require('node:test');
const assert = require('node:assert/strict');
const fs = require('node:fs');
const path = require('node:path');
const vm = require('node:vm');
const root = path.resolve(__dirname, '..');
const factory = vm.runInNewContext(fs.readFileSync(path.join(root, 'config/miti_camera_heading.js'), 'utf8') +
  '; rtabmapCameraHeadingFactory');
const config = {enabled: true, topic: '/icp_odom', opticalFrame: 'camera_depth_optical_frame'};
const layer = {type: 'odometry', topic: '/icp_odom'};
const message = {child_frame_id: 'livox_frame'};

function multiplyQuaternion(a, b) {
  const [x, y, z, w] = a, [X, Y, Z, W] = b;
  return [w*X + x*W + y*Z - z*Y, w*Y - x*Z + y*W + z*X,
    w*Z + x*Y - y*X + z*W, w*W - x*X - y*Y - z*Z];
}

function rotate(q, vector) {
  return multiplyQuaternion(multiplyQuaternion(q, [...vector, 0]), [-q[0], -q[1], -q[2], q[3]]).slice(0, 3);
}

function axisAngle(axis, angle) {
  const scale = Math.sin(angle / 2) / Math.hypot(...axis);
  return [...axis.map(v => v * scale), Math.cos(angle / 2)];
}

class Matrix {
  constructor(q = [0, 0, 0, 1], position = [0, 0, 0]) {
    this.elements = [...rotate(q, [1, 0, 0]), 0, ...rotate(q, [0, 1, 0]), 0,
      ...rotate(q, [0, 0, 1]), 0, ...position, 1];
  }
  clone() {
    const result = new Matrix();
    result.elements = [...this.elements];
    return result;
  }
}

function close(actual, expected) {
  assert.equal(actual.length, expected.length);
  actual.forEach((value, index) => assert.ok(Math.abs(value - expected[index]) < 1e-10,
    `${actual} differs from ${expected} at ${index}`));
}

test('current -90-degree mounting points toward camera forward and retains the odometry origin', () => {
  const qCamera = multiplyQuaternion(axisAngle([0, 0, 1], -Math.PI / 2), [-0.5, 0.5, -0.5, 0.5]);
  const camera = new Matrix(qCamera, [0, -0.12, -0.03]);
  const heading = factory(config, () => camera);
  const original = new Matrix([0, 0, 0, 1], [4, 5, 6]);
  const before = [...original.elements], beforeCamera = [...camera.elements];
  const arrow = heading.transform(original, layer, message, {});
  close(arrow.elements.slice(0, 3), [0, -1, 0]);
  close(arrow.elements.slice(12, 15), [4, 5, 6]);
  assert.deepEqual(original.elements, before);
  assert.deepEqual(camera.elements, beforeCamera);
});

test('camera optical +Z stays correct under robot roll/pitch/yaw and changed mounting angles', () => {
  for (let i = 1; i <= 12; i++) {
    const body = axisAngle([1, 2, -3], i * 0.25);
    const camera = axisAngle([-2, 4, 1], i * 0.4);
    const heading = factory(config, () => new Matrix(camera, [30, -20, 10]));
    const arrow = heading.transform(new Matrix(body, [1, 2, 3]), layer, message, {});
    const combined = multiplyQuaternion(body, camera);
    close(arrow.elements.slice(0, 3), rotate(combined, [0, 0, 1]));
    close(arrow.elements.slice(4, 7), rotate(combined, [-1, 0, 0]));
    close(arrow.elements.slice(8, 11), rotate(combined, [0, -1, 0]));
    close(arrow.elements.slice(12, 15), [1, 2, 3]);
  }
});

test('TF lookup uses odometry child frame, selected optical frame and the supplied timestamp', () => {
  const tree = {}, calls = [];
  const heading = factory({...config, opticalFrame: '/camera_depth_optical_frame'}, (...args) => {
    calls.push(args);
    return new Matrix();
  });
  heading.transform(new Matrix(), layer, {child_frame_id: '/livox_frame'}, tree, 17.25);
  assert.deepEqual(calls, [[tree, 'livox_frame', 'camera_depth_optical_frame', 17.25]]);
});

test('unrelated topics, paths, axes and disabled configuration keep their original pose', () => {
  const original = new Matrix();
  const unexpected = () => { throw new Error('Unexpected TF lookup'); };
  const heading = factory(config, unexpected);
  for (const other of [{...layer, topic: '/other_odom'}, {...layer, type: 'path'},
    {...layer, poseType: 'axis'}, {...layer, type: 'pose'}]) {
    assert.equal(heading.transform(original, other, {}, {}), original);
    assert.equal(heading.status(other), 'Receiving');
  }
  assert.equal(factory({...config, enabled: false}, unexpected).transform(original, layer, {}, {}), original);
});

test('missing TF reports a clear error and recovers when the transform becomes available', () => {
  let available = false;
  const heading = factory(config, () => {
    if (!available) throw new Error('No transform: camera_depth_optical_frame');
    return new Matrix();
  });
  assert.throws(() => heading.transform(new Matrix(), layer, {}, {}), /child_frame_id/);
  assert.throws(() => heading.transform(new Matrix(), layer, message, {}), /相机朝向不可用.*No transform/);
  available = true;
  close(heading.transform(new Matrix(), layer, message, {}).elements.slice(0, 3), [0, 0, 1]);
  assert.match(heading.status(layer), /箭头朝向：camera_depth_optical_frame（光轴）/);
});
