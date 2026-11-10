/* Retain one complete OccupancyGrid per active shared subscription. A second
 * renderer must not need robot motion to receive the already displayed map.
 * No images, clouds, or incremental MarkerArrays are retained here. */
function rtabmapRememberGridMessage(topic, subscription, message) {
  if (topic !== '/miti/grid_map') return;
  const info = message?.info, cells = info?.width * info?.height;
  if (!info || !Number.isInteger(info.width) || !Number.isInteger(info.height) ||
      info.width <= 0 || info.height <= 0 || cells > 16777216 ||
      !Number.isFinite(info.resolution) || info.resolution <= 0 ||
      message.data?.length !== cells) {
    delete subscription.rtabmapGridMessage;
    return;
  }
  // Includes 1x1 unknown reset maps, replacing the previous snapshot.
  subscription.rtabmapGridMessage = message;
}

function rtabmapReplayGridMessage(subscription, callback) {
  if (subscription.binding && subscription.rtabmapGridMessage) callback(subscription.rtabmapGridMessage);
}

function rtabmapClearGridMessage(subscription) {
  delete subscription.rtabmapGridMessage;
}
