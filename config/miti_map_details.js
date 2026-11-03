/* Wall toggle and numerical pose, embedded in each native 3D view. */
function rtabmapMapDetailsFactory(React, THREE, HeightClip) {
  'use strict';
  const h = React.createElement, wallTopic = '/miti/walls';
  const finiteVector = (v, n) => Array.isArray(v) && v.length === n && v.every(Number.isFinite);
  function parse(message) {
    try {
      if (typeof message?.data !== 'string' || message.data.length > 20000) return null;
      const data = JSON.parse(message.data), p = data.pose, w = data.walls;
      if (data.schema !== 1 || typeof p?.valid !== 'boolean' || !w ||
          !Number.isInteger(w.count) || w.count < 0 || !['ready', 'waiting', 'error'].includes(w.state)) return null;
      if (p.valid && (typeof p.frame_id !== 'string' || typeof p.tracked_frame !== 'string' ||
          !finiteVector(p.position_m, 3) || !finiteVector(p.rpy_deg, 3) ||
          !finiteVector(p.orientation_xyzw, 4))) return null;
      return data;
    } catch (_) { return null; }
  }
  function toggleWalls(settings) {
    const old = settings.topics?.[wallTopic];
    return {...settings, topics: {...settings.topics, [wallTopic]: {
      ...old, type: 'markers', visible: old?.visible === false
    }}};
  }
  function freshPose(sample, now, connected) {
    return connected && sample && now - sample.received < 2000 && sample.data.pose.valid
      ? sample.data.pose : null;
  }
  const css = `
    .rtab-map-details {position:absolute;bottom:32px;right:10px;z-index:3;width:290px;
      max-width:calc(100% - 20px);border:1px solid #344355;border-radius:7px;
      background:rgba(17,25,35,.94);color:#dce8f3;font:12px/1.45 system-ui,sans-serif;
      font-variant-numeric:tabular-nums;box-shadow:0 3px 14px #0003;}
    .rtab-map-details.rtab-map-inline {position:relative;bottom:auto;right:auto;z-index:auto;
      width:auto;max-width:none;margin:8px;box-shadow:none;}
    .rtab-map-details * {box-sizing:border-box;}
    .rtab-map-bar {display:flex;align-items:center;gap:8px;padding:7px 10px;}
    .rtab-map-bar button {background:transparent;color:inherit;font:inherit;border:0;padding:3px;cursor:pointer;}
    .rtab-map-bar button[aria-pressed=true] {color:#77dfc0;}
    .rtab-map-bar button:last-child {margin-left:auto;color:#a6b9cb;}
    .rtab-pose-body {padding:0 12px 10px;}
    .rtab-pose-title {font-weight:600;font-size:13px;margin:4px 0;}
    .rtab-pose-grid {display:grid;grid-template-columns:repeat(3,1fr);gap:5px 10px;margin:7px 0;}
    .rtab-pose-grid span {color:#91a6b8;font-size:10px;display:block;}
    .rtab-pose-grid b {font:14px ui-monospace,monospace;color:#e7f1fa;}
    .rtab-pose-note {font-size:10px;color:#a6b9cb;overflow-wrap:anywhere;}
    .rtab-pose-wait {color:#edc47f;margin:5px 0;}
    @media(max-width:700px) {.rtab-map-details {width:252px;}}
  `;
  function View({data, pose, connected, stale, expanded, wallsVisible, onToggle, onExpand, inline = false}) {
    const waiting = !connected ? '等待连接' : stale ? '位姿更新已中断' : data?.pose.reason || '等待定位数据';
    return h('section', {className: 'rtab-map-details' + (inline ? ' rtab-map-inline' : ''), 'aria-label': '墙面与机器人位姿',
      onPointerDown: e => e.stopPropagation(), onPointerMove: e => e.stopPropagation(), onPointerUp: e => e.stopPropagation()},
      h('style', null, css),
      h('div', {className: 'rtab-map-bar'},
        h('button', {type: 'button', 'aria-pressed': wallsVisible, onClick: onToggle}, '显示墙面'),
        h('span', {className: 'rtab-pose-note'}, data?.walls.state === 'ready' ? data.walls.count + ' 处' : '等待地图'),
        h('button', {type: 'button', 'aria-expanded': expanded, onClick: onExpand}, expanded ? '收起位姿' : '展开位姿')),
      expanded ? h('div', {className: 'rtab-pose-body'},
        h('div', {className: 'rtab-pose-title'}, '机器人位姿'),
        pose ? h(React.Fragment, null,
          h('div', {className: 'rtab-pose-note'}, '坐标系：' + pose.frame_id + ' · 参考点：' + pose.tracked_frame),
          h('div', {className: 'rtab-pose-grid'},
            ...['X', 'Y', 'Z', 'Roll 横滚', 'Pitch 俯仰', 'Yaw 航向'].map((label, i) =>
              h('div', {key: label}, h('span', null, label + (i < 3 ? ' / m' : ' / °')),
                h('b', null, (i < 3 ? pose.position_m[i] : pose.rpy_deg[i-3]).toFixed(i < 3 ? 3 : 1))))),
          !pose.body_frame ? h('div', {className: 'rtab-pose-note'},
            '当前为定位参考点；机身位姿需要已标定的 base_link 变换。') : null) :
          h('div', {className: 'rtab-pose-wait', role: 'status'}, waiting),
        h('div', {className: 'rtab-pose-note'}, data?.walls.state === 'error' ? '墙面提取失败' :
          '墙面按几何提取，大型竖直柜面也可能被包含。')) : null);
  }
  function Controls({client, settings, onChange, inline = false}) {
    const [sample, setSample] = React.useState(null), [now, setNow] = React.useState(() => performance.now());
    const [expanded, setExpanded] = React.useState(true);
    React.useEffect(() => {
      setSample(null);
      if (!client) return;
      const receive = message => {const data = parse(message); setSample(data ? {data, received: performance.now()} : null);};
      client.subscribe('/miti/map_status', receive);
      return () => client.unsubscribe('/miti/map_status', receive);
    }, [client]);
    React.useEffect(() => {const timer = setInterval(() => setNow(performance.now()), 250); return () => clearInterval(timer);}, []);
    return h(View, {data: sample?.data, pose: freshPose(sample, now, Boolean(client)), connected: Boolean(client), inline,
      stale: Boolean(sample && now-sample.received >= 2000), expanded,
      wallsVisible: settings.topics?.[wallTopic]?.visible !== false,
      onToggle: () => onChange(toggleWalls(settings)), onExpand: () => setExpanded(value => !value)});
  }
  function configureWallMaterial(material, clip) {
    material.userData.rtabmapClip = clip;
    if (!material.userData.rtabmapConfigured) {
      material.userData.rtabmapConfigured = true;
      material.onBeforeCompile = shader => {
        const current = material.userData.rtabmapClip;
        shader.uniforms.rtabmapClipPlane = {value: current.plane};
        shader.uniforms.rtabmapClipEnabled = {value: current.enabled};
        shader.vertexShader = 'uniform vec4 rtabmapClipPlane; varying float rtabmapClipDistance;\n' + shader.vertexShader.replace(
          '#include <begin_vertex>', '#include <begin_vertex>\nrtabmapClipDistance = dot(rtabmapClipPlane, modelMatrix * vec4(position, 1.0));');
        shader.fragmentShader = 'uniform bool rtabmapClipEnabled; varying float rtabmapClipDistance;\n' + shader.fragmentShader.replace(
          'void main() {', 'void main() {\nif (rtabmapClipEnabled && rtabmapClipDistance > 0.0) discard;');
        material.userData.rtabmapShader = shader;
      };
      material.customProgramCacheKey = () => 'rtabmap-wall-clip-v1';
      material.needsUpdate = true;
    }
    if (material.userData.rtabmapShader) {
      material.userData.rtabmapShader.uniforms.rtabmapClipPlane.value = clip.plane;
      material.userData.rtabmapShader.uniforms.rtabmapClipEnabled.value = clip.enabled;
    }
  }
  function WallMesh({marker, opacity}) {
    const clip = React.useContext(HeightClip.Context);
    const geometry = React.useMemo(() => {
      const result = new THREE.BufferGeometry();
      result.setAttribute('position', new THREE.Float32BufferAttribute(marker.points.flatMap(p => [p.x, p.y, p.z]), 3));
      return result;
    }, [marker]);
    const material = React.useMemo(() => new THREE.MeshBasicMaterial({
      transparent: true, depthWrite: false, side: THREE.DoubleSide
    }), []);
    material.color.setRGB(marker.color.r, marker.color.g, marker.color.b);
    material.opacity = opacity;
    configureWallMaterial(material, clip);
    React.useEffect(() => () => geometry.dispose(), [geometry]);
    React.useEffect(() => () => material.dispose(), [material]);
    return h('mesh', {geometry, material, scale: [marker.scale.x, marker.scale.y, marker.scale.z]});
  }
  return {Controls, View, parse, toggleWalls, freshPose, WallMesh, configureWallMaterial};
}

function rtabmapMigrateWallWidget(widget) {
  if (widget.i !== 'rtabmap-scene' || widget.type !== 'urdf-viewer') return widget;
  if (widget.rtabmapWallsVersion === 1 && widget.rtabmapPoseDisplayVersion === 1) return widget;
  const props = widget.props || {}, topics = props.topics || {};
  const updated = {'/miti/walls': {visible: true, type: 'markers'}, ...topics};
  if (widget.rtabmapPoseDisplayVersion !== 1) {
    // Repair the originally shipped layout once. Later user visibility changes persist.
    updated['/robot_pose'] = {poseType: 'axis', axisScale: 0.65, opacity: 1, staleAfter: 2,
      ...topics['/robot_pose'], type: 'pose', visible: true};
  }
  return {...widget, rtabmapWallsVersion: 1, rtabmapPoseDisplayVersion: 1,
    props: {...props, topics: updated}};
}
