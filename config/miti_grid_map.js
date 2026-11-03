/* Floor-relative OccupancyGrid display, paired with the existing project 3D scene. */
function rtabmapGridWidget(id = 'rtabmap-grid-scene') {
  return {
    i: id, type: 'urdf-viewer', title: '二维栅格地图', x: 4, y: 0, w: 4, h: 12,
    rtabmapGridProjectionVersion: 1,
    props: {
      version: 2,
      frame: {fixedFrame: 'map', displayFrame: 'map', followMode: 'fixed', syncTimestamps: false},
      scene: {background: '#808080', renderStats: false, labelScale: 1, rtabmapGrid: true,
        heightClip: {enabled: false, height: 2}},
      view: {perspective: false, distance: 24, thetaOffset: -90, phi: 0.01,
        targetOffset: [0, 0, 0], rtabmapTopDown: true},
      transforms: {visible: false, tfTopics: ['/tf', '/tf_static'], staticTfTopics: ['/tf_static']},
      topics: {
        '/miti/grid_map': {visible: true, type: 'map', mapColorMode: 'custom', minColor: '#ffffff',
          maxColor: '#000000', unknownColor: '#808080', invalidColor: '#ff00ff',
          opacity: 1, drawBehind: true, frameLocked: false},
        '/robot_pose': {visible: true, type: 'pose', poseType: 'arrow', axisScale: 0.5,
          color: '#20c997', opacity: 1, staleAfter: 2},
        '/miti/trajectory': {visible: true, type: 'path', poseType: 'line',
          lineWidth: 0.035, color: '#ed9b21'}
      },
      layers: []
    }
  };
}

function rtabmapMigrateGridSource(widget) {
  if (!widget.props?.scene?.rtabmapGrid || widget.rtabmapGridProjectionVersion === 1) return widget;
  const topics = widget.props.topics || {}, original = topics['/map'];
  return {...widget, rtabmapGridProjectionVersion: 1, props: {...widget.props, topics: {
    ...topics,
    ...(original ? {'/map': {...original, visible: false}} : {}),
    '/miti/grid_map': {...rtabmapGridWidget().props.topics['/miti/grid_map'], ...original,
      ...topics['/miti/grid_map'], type: 'map', visible: true}
  }}};
}

function rtabmapMigrateGridLayout(layout) {
  const updated = layout.widgets.map(rtabmapMigrateGridSource);
  if (updated.some((widget, index) => widget !== layout.widgets[index])) layout = {...layout, widgets: updated};
  const primary = layout.widgets.find(widget => widget.i === 'rtabmap-scene' && widget.type === 'urdf-viewer');
  if (!primary || primary.rtabmapGridLayoutVersion === 1) return layout;
  const leaves = tree => typeof tree === 'string' ? [tree] : tree ? [...leaves(tree.first), ...leaves(tree.second)] : [];
  const order = leaves(layout.mosaic), nextId = order[order.indexOf(primary.i) + 1];
  const next = layout.widgets.find(widget => widget.i === nextId);
  const blank = widget => {
    if (!widget || widget.type !== 'urdf-viewer' || widget.i === primary.i || widget.locked) return false;
    const props = widget.props || {};
    return ['3D Scene', ''].includes(widget.title) && !props.urdfUrl &&
      (!props.topic || props.topic === '/robot_description') &&
      !Object.values(props.topics || {}).some(topic => topic.visible !== false) &&
      !(props.layers || []).some(layer => layer.type !== 'grid' && layer.visible !== false) &&
      !(props.pointCloudTopics || []).length;
  };
  const existing = layout.widgets.find(widget => widget.props?.scene?.rtabmapGrid);
  let widgets = layout.widgets.map(widget => widget === primary ? {...widget, rtabmapGridLayoutVersion: 1} : widget);
  let mosaic = layout.mosaic;
  if (!existing && blank(next)) {
    // Reuse the empty second scene the user placed beside the point-cloud map.
    widgets = widgets.map(widget => widget === next ? {...widget,
      title: '二维栅格地图', rtabmapGridProjectionVersion: 1, props: rtabmapGridWidget(widget.i).props} : widget);
  } else if (!existing) {
    let id = 'rtabmap-grid-scene', suffix = 2;
    while (widgets.some(widget => widget.i === id)) id = 'rtabmap-grid-scene-' + suffix++;
    widgets.push(rtabmapGridWidget(id));
    const insert = tree => tree === primary.i
      ? {direction: 'row', first: primary.i, second: id, splitPercentage: 50}
      : typeof tree === 'object' && tree ? {...tree, first: insert(tree.first), second: insert(tree.second)} : tree;
    mosaic = insert(mosaic);
    if (!order.includes(primary.i)) mosaic = {direction: 'row', first: primary.i, second: id, splitPercentage: 50};
  }
  return {...layout, widgets, mosaic};
}

function rtabmapGridMapFactory(React) {
  'use strict';
  const h = React.createElement;
  function metadata(message) {
    const info = message?.info, p = info?.origin?.position, q = info?.origin?.orientation;
    if (!info || !Number.isInteger(info.width) || !Number.isInteger(info.height) ||
        info.width <= 0 || info.height <= 0 || info.width * info.height > 16777216 ||
        !Number.isFinite(info.resolution) || info.resolution <= 0 ||
        !p || !q || ![p.x, p.y, p.z, q.x, q.y, q.z, q.w].every(Number.isFinite)) return null;
    const norm = Math.hypot(q.x, q.y, q.z, q.w);
    if (norm < 1e-9) return null;
    const [x, y, z, w] = [q.x, q.y, q.z, q.w].map(value => value / norm);
    const width = info.width * info.resolution, height = info.height * info.resolution;
    const points = [[0, 0], [width, 0], [0, height], [width, height]].map(([u, v]) => [
      p.x + (1-2*(y*y+z*z))*u + 2*(x*y-z*w)*v,
      p.y + 2*(x*y+z*w)*u + (1-2*(x*x+z*z))*v,
      p.z + 2*(x*z-y*w)*u + 2*(y*z+x*w)*v]);
    const minimum = [0, 1, 2].map(i => Math.min(...points.map(point => point[i])));
    const maximum = [0, 1, 2].map(i => Math.max(...points.map(point => point[i])));
    return {frame: message.header?.frame_id || '', resolution: info.resolution,
      width, height, columns: info.width, rows: info.height,
      center: minimum.map((value, i) => (value + maximum[i]) / 2),
      span: minimum.map((value, i) => maximum[i] - value)};
  }
  function fitView(view, map, aspect = 1) {
    const fovy = Number.isFinite(view.fovy) && view.fovy > 0 && view.fovy < 180 ? view.fovy : 45;
    const height = Math.max(map.span[1], map.span[0] / Math.max(0.1, aspect), 1) * 1.12;
    return {...view, perspective: false, phi: 0.01, thetaOffset: -90, rtabmapTopDown: true,
      distance: height / (2 * Math.tan(fovy * Math.PI / 360)), targetOffset: map.center};
  }
  function statusMetadata(status) {
    if (!Array.isArray(status?.origin) || status.origin.length !== 3) return null;
    return metadata({header: {frame_id: status.frame_id || 'map'}, info: {
      width: status.width, height: status.height, resolution: status.resolution,
      origin: {position: {x: status.origin[0], y: status.origin[1], z: status.origin[2]},
        orientation: {x: 0, y: 0, z: 0, w: 1}}
    }});
  }
  function View({map, status, connected, onFit, panelRef}) {
    return h('div', {ref: panelRef, className: 'rtab-grid-controls',
      onPointerDown: event => event.stopPropagation(), onPointerUp: event => event.stopPropagation()},
      h('style', null, `
        .rtab-grid-controls {position:absolute;top:10px;left:10px;z-index:3;max-width:calc(100% - 20px);
          padding:8px 10px;border:1px solid #41505d;border-radius:7px;background:rgba(17,25,35,.94);
          color:#dce8f3;font:12px/1.6 system-ui,sans-serif;}
        .rtab-grid-controls button {margin-left:12px;border:1px solid #526777;border-radius:4px;
          padding:2px 8px;background:#253644;color:#dce8f3;font:inherit;cursor:pointer;}
        .rtab-grid-controls button:disabled {opacity:.5;cursor:default;}
        .rtab-grid-legend {display:flex;gap:12px;flex-wrap:wrap;font-size:11px;color:#bbccd9;}
        .rtab-grid-legend i {display:inline-block;width:10px;height:10px;border:1px solid #8b99a5;margin-right:4px;}
      `),
      h('div', null, map ? `${(map.resolution*100).toFixed(1)} cm / 格 · ${map.columns} × ${map.rows}` :
        connected ? '等待栅格地图' : '等待连接',
        h('button', {type: 'button', disabled: !map, onClick: onFit}, '适应地图')),
      h('div', {className: 'rtab-grid-legend'}, ...[
        ['#000000', '障碍'], ['#ffffff', '空闲'], ['#808080', '未知']
      ].map(([color, label]) => h('span', {key: label}, h('i', {style: {background: color}}), label))),
      status ? h('div', {style: {fontSize: 11, color: status.state === 'ready' ? '#bbccd9' : '#edc47f'}},
        status.state === 'ready'
          ? `障碍高度：距地面 ${status.min_obstacle_height.toFixed(2)}–${status.max_obstacle_height.toFixed(2)} m`
          : status.reason || '等待地面识别') : null,
      map ? h('div', {style: {fontSize: 10, color: '#9eb3c4'}},
        `${map.frame} · ${map.width.toFixed(1)} × ${map.height.toFixed(1)} m · 滚轮缩放，右键拖动`) : null,
      map?.received ? h('div', {style: {fontSize: 10, color: '#9eb3c4'}},
        '最近接收 ' + new Date(map.received).toLocaleTimeString()) : null);
  }
  function Controls({client, settings, onChange}) {
    const [map, setMap] = React.useState(null), panelRef = React.useRef(null);
    const [status, setStatus] = React.useState(null);
    const latest = React.useRef({settings, onChange}), fitted = React.useRef(false);
    latest.current = {settings, onChange};
    function fit(value) {
      const {settings: current, onChange: change} = latest.current;
      const bounds = panelRef.current?.parentElement?.getBoundingClientRect();
      if (!bounds || bounds.width <= 0 || bounds.height <= 0) return false;
      change({...current, view: fitView(current.view, value, bounds.width / bounds.height)});
      return true;
    }
    React.useEffect(() => {
      setMap(null);
      setStatus(null);
      if (!client) return;
      const updateMap = status => {
        const value = statusMetadata(status);
        setMap(value ? {...value, received: Date.now()} : null);
        if (value && value.columns === 1 && value.rows === 1) fitted.current = false;
        if (value && value.columns > 1 && value.rows > 1 && value.frame.replace(/^\/+/, '') === latest.current.settings.frame.fixedFrame && !fitted.current) {
          fitted.current = fit(value);
        }
      };
      const receiveStatus = message => {
        try {
          const value = JSON.parse(message.data);
          const valid = value.schema === 1 && Number.isFinite(value.min_obstacle_height) &&
            Number.isFinite(value.max_obstacle_height) && typeof value.state === 'string';
          setStatus(valid ? value : null);
          updateMap(valid ? value : null);
        } catch (_) { setStatus(null); setMap(null); }
      };
      // MITI caches retained TF only, not ordinary topic messages. Subscribing
      // here before the Canvas mounts consumes the latched grid; its later
      // renderer subscription then receives nothing until the next map update.
      // Read metadata from the separate status topic, leaving /miti/grid_map
      // exclusively to the native map renderer.
      client.subscribe('/miti/grid_status', receiveStatus);
      return () => {
        client.unsubscribe('/miti/grid_status', receiveStatus);
      };
    }, [client]);
    return h(View, {map, status, connected: Boolean(client), panelRef, onFit: () => map && fit(map)});
  }
  return {Controls, View, metadata, fitView, statusMetadata};
}
