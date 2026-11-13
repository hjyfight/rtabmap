/* The map is retained; current scans replace themselves and clear only proven free history. */
function rtabmapLiveScanVisible(layer, received, now) {
  if (layer?.topic !== '/miti/live_scan') return true;
  const time = received?.getTime?.();
  return Number.isFinite(time) && Number.isFinite(now) && now-time <= 1500;
}

function rtabmapMigrateDynamicCloudWidget(widget) {
  if (widget.i !== 'rtabmap-scene' || widget.type !== 'urdf-viewer' || widget.rtabmapDynamicCloudVersion === 1) return widget;
  const props = widget.props || {}, topics = props.topics || {};
  const original = topics['/cloud_map'] || {type: 'pointcloud', colorMode: 'colormap', colorField: 'z', pointSize: 2};
  const visible = original.visible !== false;
  return {...widget, rtabmapDynamicCloudVersion: 1, props: {...props,
    scene: {...props.scene, rtabmapDynamicCloud: true}, topics: {...topics,
      '/cloud_map': {...original, visible: false},
      '/miti/clean_map': {...original, ...topics['/miti/clean_map'], type: 'pointcloud', visible, maxRate: 2, maxPoints: 300000, staleAfter: 30},
      '/miti/live_scan': {...original, ...topics['/miti/live_scan'], type: 'pointcloud', visible, maxRate: 10, maxPoints: 40000, staleAfter: 1.5}
    }}};
}

function rtabmapDynamicCloudFactory(React) {
  const h = React.createElement;
  function toggle(settings) {
    const enabled = settings.scene?.rtabmapDynamicCloud !== true;
    return {...settings, scene: {...settings.scene, rtabmapDynamicCloud: enabled}, topics: {...settings.topics,
      '/cloud_map': {...settings.topics?.['/cloud_map'], visible: !enabled},
      '/miti/clean_map': {...settings.topics?.['/miti/clean_map'], visible: enabled},
      '/miti/live_scan': {...settings.topics?.['/miti/live_scan'], visible: enabled}}};
  }
  function Controls({client, settings, onChange}) {
    const [sample, setSample] = React.useState(null), [now, setNow] = React.useState(() => performance.now());
    const enabled = settings.scene?.rtabmapDynamicCloud === true;
    const supported = Boolean(settings.topics?.['/miti/clean_map']);
    React.useEffect(() => {
      setSample(null);
      if (!client || !supported) return;
      const receive = message => {
        try {
          const data = JSON.parse(message.data);
          if (data.schema === 1 && Number.isFinite(data.rate_hz)) setSample({data, received: performance.now()});
        } catch (_) { /* Wait for a complete status message. */ }
      };
      client.subscribe('/miti/dynamic_cloud_status', receive);
      return () => client.unsubscribe('/miti/dynamic_cloud_status', receive);
    }, [client, supported]);
    React.useEffect(() => {
      if (!supported) return;
      const timer = setInterval(() => setNow(performance.now()), 500);
      return () => clearInterval(timer);
    }, [supported]);
    if (!supported) return null;
    const data = sample && now-sample.received < 3000 ? sample.data : null;
    return h('div', {style: {display: 'flex', gap: '8px', alignItems: 'center', padding: '0 10px 7px'}},
      h('button', {type: 'button', 'aria-pressed': enabled, onClick: () => onChange(toggle(settings)),
        title: '保留环境地图；新射线连续确认空闲后清除残影。关闭可对照原始累积地图。',
        style: {font: 'inherit', color: enabled ? '#77dfc0' : '#a6b9cb', background: 'transparent', border: 0, padding: 0, cursor: 'pointer'}},
        enabled ? '动态清理：开启' : '动态清理：关闭'),
      h('span', {style: {color: '#a6b9cb', fontSize: '10px'},
        title: data ? (data.reason || '') + ' · 已清理历史点 ' + (data.removed_points || 0) : '等待动态清理状态'},
        !enabled ? '原始地图' : data?.state === 'ready' ? '实时扫描 ' + data.rate_hz.toFixed(1) + ' Hz' : '等待扫描与位姿'));
  }
  return {Controls, toggle};
}
