/* Add the ARC camera once while preserving the user's saved dashboard. */
function rtabmapArcCameraWidget() {
  return {
    i: 'rtabmap-arc-camera', type: 'camera-viewer', title: 'ARC · 相机画面',
    x: 8, y: 12, w: 4, h: 4,
    props: {topic: '/arc_camera/image_raw', maxFps: 15}
  };
}

function rtabmapMigrateArcCameraLayout(layout) {
  const leaves = tree => typeof tree === 'string' ? [tree] :
    tree ? [...leaves(tree.first), ...leaves(tree.second)] : [];
  const visible = new Set(leaves(layout.mosaic));
  const anchor = ['rtabmap-infrared', 'rtabmap-depth', 'rtabmap-color', 'rtabmap-scene'].map(id =>
    layout.widgets.find(widget => widget.i === id && visible.has(id) &&
      widget.type === (id === 'rtabmap-scene' ? 'urdf-viewer' : 'camera-viewer'))).find(Boolean);
  if (!anchor || layout.widgets.some(widget => widget.rtabmapArcCameraLayoutVersion === 1)) return layout;

  // Store the marker on a retained project panel so deliberate ARC removal persists.
  const marker = layout.widgets.find(widget => widget.i === 'rtabmap-scene' &&
    widget.type === 'urdf-viewer') || anchor;
  const widgets = layout.widgets.map(widget => widget === marker ?
    {...widget, rtabmapArcCameraLayoutVersion: 1} : widget);
  if (widgets.some(widget => widget.i === 'rtabmap-arc-camera' ||
      (widget.type === 'camera-viewer' && widget.props?.topic === '/arc_camera/image_raw')))
    return {...layout, widgets};

  const insert = tree => {
    if (tree === anchor.i) return {direction: 'column', first: tree,
      second: 'rtabmap-arc-camera', splitPercentage: 50};
    if (!tree || typeof tree === 'string') return tree;
    const first = insert(tree.first), second = insert(tree.second);
    return first === tree.first && second === tree.second ? tree : {...tree, first, second};
  };
  return {...layout, widgets: [...widgets, rtabmapArcCameraWidget()], mosaic: insert(layout.mosaic)};
}
