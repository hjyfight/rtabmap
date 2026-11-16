# All pixels/masks are in this RGB frame, before any display annotation.
rtabmap_semantic_msgs/SegmentationFrame baseline
sensor_msgs/Image rgb
int32[] class_ids
string[] class_names
# Deadline is measured from request receipt. Selector also enforces its own deadline.
float32 deadline_seconds
