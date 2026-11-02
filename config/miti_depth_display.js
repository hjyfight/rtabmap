/* Migrate only the project's legacy depth panel once, preserving the rest of the layout. */
function rtabmapMigrateDepthWidget(widget) {
  if (widget.i !== 'rtabmap-depth' || widget.type !== 'camera-viewer' ||
      widget.rtabmapAlignedDepthVersion === 1) return widget;
  if (!['/camera/aligned_depth_to_color/image_raw', '/miti/aligned_depth/image_raw']
      .includes(widget.props?.topic)) return widget;
  // Keep the migration version on the widget: native camera settings normalize props.
  return {...widget, rtabmapAlignedDepthVersion: 1,
    props: {...widget.props, topic: '/miti/aligned_depth/image_raw'}};
}
