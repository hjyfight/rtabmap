/* Native 3D panels for RTAB-Map pose-graph constraints and sparse visual words. */
function rtabmapVersionPanelChunk(runtime, version) {
  const original = runtime.u;
  runtime.u = chunkId => {
    const path = original(chunkId);
    return chunkId === 453 ? path + (path.includes('?') ? '&' : '?') +
      'rtabmap_panel=' + encodeURIComponent(version) : path;
  };
}

function rtabmapGraphMapKind(settings) {
  const configured = settings?.scene?.rtabmapGraphMap;
  if (configured === 'topology' || configured === 'features') return configured;
  const topology = settings?.topics?.['/miti/topological_map']?.visible === true ||
    settings?.topics?.['/miti/navigation_map']?.visible === true;
  const features = settings?.topics?.['/miti/sparse_features']?.visible === true;
  return topology !== features ? (topology ? 'topology' : 'features') : null;
}

function rtabmapRepairGraphFeatureWidget(widget) {
  if (widget.type !== 'urdf-viewer') return widget;
  const kind = rtabmapGraphMapKind(widget.props);
  if (!kind) return widget;
  if (widget.props?.scene?.rtabmapGraphMap !== kind) widget = {...widget, props: {...widget.props,
    scene: {...widget.props?.scene, rtabmapGraphMap: kind}}};
  if (kind === 'topology' && widget.rtabmapNavigationVersion !== 1) widget = {...widget,
    rtabmapNavigationVersion: 1,
    title: ['拓扑地图 · 关键帧与回环', '关键帧位姿图'].includes(widget.title)
      ? '导航连通图 · 区域与通道' : widget.title,
    props: rtabmapSetTopologyMode(widget.props, 'navigation')};
  return widget;
}

function rtabmapTopologyMode(settings) {
  return settings?.scene?.rtabmapTopologyMode === 'navigation' ? 'navigation' : 'pose_graph';
}

function rtabmapSetTopologyMode(settings, mode) {
  const navigation = mode === 'navigation';
  return {...settings, scene: {...settings.scene, rtabmapTopologyMode: mode}, topics: {
    ...settings.topics,
    '/miti/navigation_map': {...settings.topics?.['/miti/navigation_map'], type: 'markers', visible: navigation},
    '/miti/topological_map': {...settings.topics?.['/miti/topological_map'], type: 'markers', visible: !navigation},
    '/miti/grid_map': {type: 'map', mapColorMode: 'custom', minColor: '#b8c6c9', maxColor: '#111923',
      unknownColor: '#25313b', opacity: .4, drawBehind: true, frameLocked: false,
      ...settings.topics?.['/miti/grid_map'], visible: navigation}
  }};
}

function rtabmapGraphFeatureWidget(kind, id) {
  const topology = kind === 'topology';
  const widget = {
    i: id || (topology ? 'rtabmap-topology-scene' : 'rtabmap-features-scene'),
    type: 'urdf-viewer', title: topology ? '拓扑地图 · 关键帧与回环' : '稀疏特征地图 · 视觉特征点',
    x: topology ? 0 : 4, y: 8, w: 4, h: 8,
    props: {
      version: 2,
      frame: {fixedFrame: 'map', displayFrame: 'map', followMode: 'fixed', syncTimestamps: false},
      scene: {background: '#111923', renderStats: false, labelScale: 1, rtabmapGraphMap: kind,
        heightClip: {enabled: false, height: 2}},
      view: {perspective: !topology, distance: 12, thetaOffset: topology ? -90 : 45,
        phi: topology ? 0.01 : 60, targetOffset: [0, 0, 0], rtabmapTopDown: topology},
      transforms: {visible: false, tfTopics: ['/tf', '/tf_static'], staticTfTopics: ['/tf_static']},
      topics: {
        [topology ? '/miti/topological_map' : '/miti/sparse_features']: topology
          ? {visible: true, type: 'markers'}
          : {visible: true, type: 'pointcloud', pointSize: 4, maxPoints: 150000,
            colorMode: 'colormap', colorField: 'z', maxRate: 2, staleAfter: 30},
        '/robot_pose': {visible: true, type: 'pose', poseType: 'arrow', axisScale: 0.5,
          color: '#ffffff', opacity: 1, staleAfter: 2}
      },
      layers: []
    }
  };
  if (topology) {
    widget.title = '导航连通图 · 区域与通道';
    widget.rtabmapNavigationVersion = 1;
    widget.props = rtabmapSetTopologyMode(widget.props, 'navigation');
  }
  return widget;
}

function rtabmapMigrateGraphFeatureLayout(layout) {
  // Older saved scenes can retain their data topics while losing the panel tag.
  // Repair these before checking the one-time layout migration marker.
  const repaired = layout.widgets.map(rtabmapRepairGraphFeatureWidget);
  if (repaired.some((widget, index) => widget !== layout.widgets[index])) layout = {...layout, widgets: repaired};
  const primary = layout.widgets.find(w => w.i === 'rtabmap-scene' && w.type === 'urdf-viewer');
  if (!primary || primary.rtabmapGraphFeatureLayoutVersion === 1) return layout;
  let widgets = layout.widgets.map(w => w === primary ? {...w, rtabmapGraphFeatureLayoutVersion: 1} : w);
  const added = [];
  for (const kind of ['topology', 'features']) {
    if (widgets.some(w => w.props?.scene?.rtabmapGraphMap === kind ||
        w.props?.topics?.[kind === 'topology' ? '/miti/topological_map' : '/miti/sparse_features']?.visible === true)) continue;
    let id = rtabmapGraphFeatureWidget(kind).i, suffix = 2;
    while (widgets.some(w => w.i === id)) id = rtabmapGraphFeatureWidget(kind).i + '-' + suffix++;
    widgets.push(rtabmapGraphFeatureWidget(kind, id));
    added.push(id);
  }
  if (!added.length) return {...layout, widgets};
  const newMaps = added.length === 2
    ? {direction: 'row', first: added[0], second: added[1], splitPercentage: 50} : added[0];
  const grid = widgets.find(w => w.props?.scene?.rtabmapGrid);
  const pair = tree => tree && typeof tree === 'object' && grid &&
    ((tree.first === primary.i && tree.second === grid.i) || (tree.second === primary.i && tree.first === grid.i));
  let inserted = false;
  const insertPair = tree => {
    if (pair(tree)) {inserted = true; return {direction: 'column', first: tree, second: newMaps, splitPercentage: 50};}
    return typeof tree === 'object' && tree ? {...tree, first: insertPair(tree.first), second: insertPair(tree.second)} : tree;
  };
  let mosaic = insertPair(layout.mosaic);
  if (!inserted) {
    const insertPrimary = tree => {
      if (tree === primary.i) {inserted = true; return {direction: 'column', first: tree, second: newMaps, splitPercentage: 50};}
      return typeof tree === 'object' && tree ? {...tree, first: insertPrimary(tree.first), second: insertPrimary(tree.second)} : tree;
    };
    mosaic = insertPrimary(layout.mosaic);
  }
  // A grid-only saved layout can omit its mosaic; keep every widget reachable.
  if (!inserted) mosaic = widgets.map(w => w.i).reduce((tree, id) =>
    tree ? {direction: 'row', first: tree, second: id, splitPercentage: 50} : id, null);
  return {...layout, widgets, mosaic};
}

function rtabmapGraphFeatureMapsFactory(React) {
  'use strict';
  const h = React.createElement;
  const vector = value => Array.isArray(value) && value.length === 3 && value.every(Number.isFinite);
  function parse(message) {
    try {
      const data = JSON.parse(message.data);
      if (data.schema !== 1 || !['waiting', 'ready'].includes(data.state) || typeof data.frame_id !== 'string') return null;
      const navigation = data.kind === 'navigation';
      for (const key of navigation ? ['topology'] : ['topology', 'features']) {
        const item = data[key];
        if (!item || (item.bounds && (!vector(item.bounds.center) || !vector(item.bounds.span) || item.bounds.span.some(v => v < 0)))) return null;
      }
      const counts = navigation ? [data.topology.nodes, data.topology.links, data.topology.regions,
        data.topology.corridors, data.topology.doorways, data.topology.components] :
        [data.topology.nodes, data.topology.links, data.features.points];
      if (!counts.every(v => Number.isInteger(v) && v >= 0)) return null;
      return data;
    } catch (_) { return null; }
  }
  function fitView(view, bounds, aspect = 1, topDown = view.rtabmapTopDown) {
    const fovy = Number.isFinite(view.fovy) && view.fovy > 0 && view.fovy < 180 ? view.fovy : 45;
    const span = topDown ? Math.max(bounds.span[1], bounds.span[0] / Math.max(.1, aspect), 1)
      : Math.max(Math.hypot(...bounds.span), 1) / Math.min(1, Math.max(.1, aspect));
    return {...view, perspective: !topDown, phi: topDown ? .01 : 60,
      thetaOffset: topDown ? -90 : 45, rtabmapTopDown: Boolean(topDown),
      distance: span * 1.2 / (2 * Math.tan(fovy * Math.PI / 360)), targetOffset: bounds.center};
  }
  function View({kind, mode, data, connected, stale, onFit, onView, onMode, topDown, panelRef,
                 collapsed = false, onCollapse}) {
    const topology = kind === 'topology', item = data?.[kind === 'topology' ? 'topology' : 'features'];
    const navigation = topology && mode === 'navigation';
    const isCollapsed = topology && collapsed;
    const summary = !connected ? '等待连接' : stale ? '地图显示更新已中断' :
      data?.state !== 'ready' ? data?.reason || '等待地图数据' : topology
        ? navigation ? `${item.regions} 个区域 · ${item.corridors} 条通道 · ${item.doorways} 个候选门口`
          : `${item.nodes} 个关键帧 · ${item.links} 条约束` : `${item.points.toLocaleString()} 个视觉特征点`;
    return h('div', {className: 'rtab-graph-controls', ref: panelRef, 'data-collapsed': isCollapsed,
      onPointerDown: e => e.stopPropagation(), onPointerUp: e => e.stopPropagation()},
      h('style', null, `
        .rtab-graph-controls {position:absolute;top:10px;left:10px;z-index:3;max-width:calc(100% - 20px);
          padding:7px 10px;border:1px solid #41505d;border-radius:7px;background:rgba(17,25,35,.94);
          color:#dce8f3;font:12px/1.6 system-ui,sans-serif;}
        .rtab-graph-controls button {margin-left:8px;border:1px solid #526777;border-radius:4px;
          padding:2px 6px;background:#253644;color:inherit;font:inherit;cursor:pointer;}
        .rtab-graph-controls button:disabled {opacity:.5;cursor:default;}
        .rtab-graph-collapse {float:right;}
        .rtab-graph-controls[data-collapsed="true"] {padding:3px 5px;}
        .rtab-graph-controls[data-collapsed="true"] .rtab-graph-collapse {float:none;margin:0;}
        .rtab-graph-legend {display:flex;gap:10px;flex-wrap:wrap;font-size:10px;color:#bbccd9;}
        .rtab-graph-legend i {display:inline-block;width:12px;height:3px;vertical-align:middle;margin-right:4px;}
      `),
      topology ? h('button', {type: 'button', className: 'rtab-graph-collapse',
        'aria-expanded': !isCollapsed, 'aria-label': isCollapsed ? '展开导航信息' : '隐藏导航信息',
        onClick: onCollapse}, isCollapsed ? '展开导航信息' : '隐藏') : null,
      !isCollapsed ? h(React.Fragment, null,
      h('div', {role: 'status'}, summary,
        h('button', {type: 'button', disabled: !item?.bounds, onClick: onFit}, '适应地图'),
        h('button', {type: 'button', onClick: onView}, topDown ? '三维视角' : '俯视')),
      topology ? h('div', null,
        h('button', {type: 'button', 'aria-pressed': navigation, onClick: () => onMode?.('navigation')}, '导航连通图'),
        h('button', {type: 'button', 'aria-pressed': !navigation, onClick: () => onMode?.('pose_graph')}, '关键帧位姿图')) : null,
      topology ? h('div', {className: 'rtab-graph-legend'}, ...(navigation ? [
        ['#40e6cc', '区域'], ['#59a6ff', '通道'], ['#ffbf33', '候选门口'], ['#4dd973', '空闲路径']
      ] : [
        ['#40e6cc', '关键帧'], ['#59a6ff', '相邻约束'], ['#ff598c', '全局回环'], ['#ffbf33', '邻近回环']
      ]).map(([color, label]) => h('span', {key: label}, h('i', {style: {background: color}}), label))) :
        h('div', {style: {fontSize: 10, color: '#bbccd9'}}, item?.points ?
          `${item.feature_nodes} 个关键帧提供特征 · 颜色表示高度` : item?.reason || '等待 RGB-D 关键帧的三维视觉特征'),
      navigation ? h('div', {style: {fontSize: 10, color: '#bbccd9'}},
        `几何候选 · ${item?.components ?? 0} 个连通分量 · 净空半径 ≥ ${Number(item?.required_clearance_m ?? .45).toFixed(2)} m`,
        data?.robot_region ? ` · 当前位置 ${data.robot_region}` : '') : null,
      navigation && item?.nodes === 0 ? h('div', {style: {fontSize: 11, color: '#edc47f'}},
        '当前仅显示栅格背景，尚未生成导航节点和连线') : null,
      data?.updated_at ? h('div', {style: {fontSize: 10, color: '#9eb3c4'}},
        `${data.frame_id} · 最近地图更新 ${new Date(data.updated_at * 1000).toLocaleTimeString()}`) : null) : null);
  }
  function Controls({client, settings, onChange}) {
    const [sample, setSample] = React.useState(null), [now, setNow] = React.useState(Date.now);
    const panelRef = React.useRef(null), fittedEpoch = React.useRef(null), latest = React.useRef({settings, onChange});
    latest.current = {settings, onChange};
    const kind = rtabmapGraphMapKind(settings);
    const mode = rtabmapTopologyMode(settings);
    function fit(data, topDown) {
      const {settings: current, onChange: change} = latest.current;
      const bounds = data?.[kind === 'topology' ? 'topology' : 'features']?.bounds;
      const rect = panelRef.current?.parentElement?.getBoundingClientRect();
      if (!bounds || !rect?.width || !rect?.height || data.frame_id !== current.frame.fixedFrame) return false;
      change({...current, view: fitView(current.view, bounds, rect.width / rect.height,
        topDown === undefined ? current.view.rtabmapTopDown : topDown)});
      return true;
    }
    React.useEffect(() => {
      setSample(null);
      fittedEpoch.current = null;
      if (!client) return;
      const receive = message => {
        const data = parse(message);
        setSample(data ? {data, received: Date.now()} : null);
        if (data && data.epoch !== fittedEpoch.current && fit(data)) fittedEpoch.current = data.epoch;
      };
      // Separate status input leaves the retained map messages to MITI's native renderers.
      const topic = kind === 'topology' && mode === 'navigation' ? '/miti/navigation_status' : '/miti/graph_feature_status';
      client.subscribe(topic, receive);
      return () => client.unsubscribe(topic, receive);
    }, [client, kind, mode]);
    React.useEffect(() => {const timer = setInterval(() => setNow(Date.now()), 1000); return () => clearInterval(timer);}, []);
    return h(View, {kind, mode, data: sample?.data, connected: Boolean(client),
      stale: Boolean(sample && now - sample.received > 4000), panelRef,
      collapsed: Boolean(settings.scene?.rtabmapGraphControlsCollapsed),
      onCollapse: () => {
        const {settings: current, onChange: change} = latest.current;
        change({...current, scene: {...current.scene,
          rtabmapGraphControlsCollapsed: !current.scene?.rtabmapGraphControlsCollapsed}});
      },
      topDown: settings.view.rtabmapTopDown, onFit: () => fit(sample?.data),
      onMode: value => onChange(rtabmapSetTopologyMode(settings, value)),
      onView: () => {
        const topDown = !settings.view.rtabmapTopDown;
        if (!fit(sample?.data, topDown)) onChange({...settings, view: {...settings.view,
          perspective: !topDown, rtabmapTopDown: topDown, phi: topDown ? .01 : 60, thetaOffset: topDown ? -90 : 45}});
      }});
  }
  return {Controls, View, parse, fitView};
}
