# The images and calibration belong to this exact RTAB-Map node.
std_msgs/Header header
int32 node_id
int32 session_id
float64 node_stamp
sensor_msgs/Image rgb
sensor_msgs/Image depth
sensor_msgs/CameraInfo camera_info
# Maps optical camera coordinates into the graph node's local frame.
geometry_msgs/Transform camera_to_node
