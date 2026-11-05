/* Add the project's infrared view once without replacing a saved dashboard. */
function rtabmapInfraredWidget() {
  return {
    i: 'rtabmap-infrared', type: 'camera-viewer', title: 'RealSense · 右近红外',
    rtabmapInfraredSourceVersion: 2,
    x: 8, y: 8, w: 4, h: 4,
    props: {topic: '/camera/infra2/image_rect_raw', maxFps: 10,
      colorMode: 'gradient', gradient: ['#000000', '#ffffff'], valueMin: 0, valueMax: 255}
  };
}

/* Switch the existing project infrared panel once; retain other image settings. */
function rtabmapMigrateInfraredTopicWidget(widget) {
  if (widget.i !== 'rtabmap-infrared' || widget.type !== 'camera-viewer' ||
      widget.rtabmapInfraredSourceVersion === 2 ||
      !['/camera/infra1/image_rect_raw', '/camera/infra2/image_rect_raw']
        .includes(widget.props?.topic)) return widget;
  const props = {...widget.props, topic: '/camera/infra2/image_rect_raw'};
  if (props.calibrationTopic === '/camera/infra1/camera_info')
    props.calibrationTopic = '/camera/infra2/camera_info';
  return {...widget, rtabmapInfraredSourceVersion: 2,
    title: widget.title === 'RealSense · 左近红外' ? 'RealSense · 右近红外' : widget.title,
    props};
}

function rtabmapMigrateInfraredLayout(layout) {
  const leaves = tree => typeof tree === 'string' ? [tree] :
    tree ? [...leaves(tree.first), ...leaves(tree.second)] : [];
  const visible = new Set(leaves(layout.mosaic));
  const anchor = ['rtabmap-depth', 'rtabmap-color'].map(id =>
    layout.widgets.find(widget => widget.i === id && widget.type === 'camera-viewer' && visible.has(id)))
    .find(Boolean);
  if (!anchor || anchor.rtabmapInfraredLayoutVersion === 1) return layout;

  // Keep the marker on an existing panel so a later deliberate removal persists.
  const widgets = layout.widgets.map(widget => widget === anchor ?
    {...widget, rtabmapInfraredLayoutVersion: 1} : widget);
  const existing = widgets.some(widget => widget.i === 'rtabmap-infrared' ||
    (widget.type === 'camera-viewer' &&
      ['/camera/infra1/image_rect_raw', '/camera/infra2/image_rect_raw'].includes(widget.props?.topic)));
  if (existing) return {...layout, widgets};

  const insert = tree => {
    if (tree === anchor.i) return {direction: 'column', first: tree,
      second: 'rtabmap-infrared', splitPercentage: 50};
    if (!tree || typeof tree === 'string') return tree;
    const first = insert(tree.first), second = insert(tree.second);
    return first === tree.first && second === tree.second ? tree : {...tree, first, second};
  };
  return {...layout, widgets: [...widgets, rtabmapInfraredWidget()], mosaic: insert(layout.mosaic)};
}
