std_msgs/Header header
string session_id
uint64 sequence
uint32 revision
uint32 width
uint32 height
string model_sha256
string vocabulary_version
string provenance_json
string result_source
string selection_reason
string provider_id
float32 inference_seconds
rtabmap_semantic_msgs/SegmentationInstance[] instances
# 32SC1: class -1 = unknown; instance 0 = uncovered.
sensor_msgs/Image class_map
sensor_msgs/Image instance_map
