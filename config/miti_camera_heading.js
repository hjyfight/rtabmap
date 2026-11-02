/* Point the odometry arrow along the camera optical axis without changing its position. */
function rtabmapCameraHeadingFactory(config, lookupTransform) {
  'use strict';
  const frameName = name => typeof name === 'string' ? name.trim().replace(/^\/+/, '') : '';
  const topic = frameName(config.topic);
  const opticalFrame = frameName(config.opticalFrame);

  function applies(layer) {
    return config.enabled === true && layer.type === 'odometry' &&
      frameName(layer.topic) === topic && (!layer.poseType || layer.poseType === 'arrow');
  }

  function transform(matrix, layer, message, tree, time = 0) {
    if (!applies(layer)) return matrix;
    const child = frameName(message.child_frame_id);
    if (!child) throw new Error('相机朝向不可用：里程计缺少 child_frame_id');
    let camera;
    try {
      camera = lookupTransform(tree, child, opticalFrame, time);
    } catch (error) {
      // Let the native layer status show the missing TF; never substitute a wrong heading.
      throw new Error('相机朝向不可用：' + (error instanceof Error ? error.message : String(error)));
    }
    const original = matrix.elements, relative = camera.elements;
    const result = matrix.clone(), out = result.elements;
    // Optical axes: +Z forward, +X right, +Y down. MITI's arrow points along +X.
    // New arrow axes are optical (+Z, -X, -Y), then rotated into the fixed frame.
    // Only the rotation changes: the odometry origin and all input TFs stay untouched.
    for (const [destination, source, sign] of [[0, 8, 1], [4, 0, -1], [8, 4, -1]]) {
      for (let row = 0; row < 3; row++) {
        out[destination + row] = sign * (original[row] * relative[source] +
          original[4 + row] * relative[source + 1] + original[8 + row] * relative[source + 2]);
      }
    }
    return result;
  }

  function status(layer) {
    return applies(layer) ? 'Receiving · 箭头朝向：' + opticalFrame + '（光轴）' : 'Receiving';
  }

  return {applies, transform, status};
}
