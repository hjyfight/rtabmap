uint32 instance_id
int32 class_id
string label
# Original YOLO instance confidence; refinement does not overwrite it.
float32 confidence
# Calibrated refinement score, -1 when absent.
float32 classification_score
float32[4] bbox_xyxy
uint32[2] mask_offset
sensor_msgs/Image mask
