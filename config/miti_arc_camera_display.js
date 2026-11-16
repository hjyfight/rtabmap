/* Remove the retired ARC pane from saved layouts; preserve other settings. */
function rtabmapMigrateArcCameraLayout(layout) {
  if (!layout || !Array.isArray(layout.widgets)) return layout;
  const removed = new Set(layout.widgets.filter(widget => widget.i === 'rtabmap-arc-camera' ||
    (widget.type === 'camera-viewer' && widget.props?.topic === '/arc_camera/image_raw')).map(widget => widget.i));
  removed.add('rtabmap-arc-camera'); // Prune dangling legacy leaves too.
  const prune = tree => {
    if (typeof tree === 'string') return removed.has(tree) ? null : tree;
    if (!tree) return tree;
    const first = prune(tree.first), second = prune(tree.second);
    if (!first) return second;
    if (!second) return first;
    return first === tree.first && second === tree.second ? tree : {...tree, first, second};
  };
  const widgets = layout.widgets.filter(widget => !removed.has(widget.i));
  const mosaic = prune(layout.mosaic);
  return widgets.length === layout.widgets.length && mosaic === layout.mosaic ? layout : {...layout, widgets, mosaic};
}
