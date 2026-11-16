# Header identifies the RGB acquisition anchor. Vector order is fixed below.
std_msgs/Header header
uint64 sequence
string model_sha256
string calibration_version
string[4] modality_order
float64[4] source_stamps
float64[4] source_durations
bool[4] valid_mask
float32[4] weights
bool complete
bool fused_valid
# Raw backbone outputs, then normalized rows, all in fixed modality order.
float32[4096] encoder_features
float32[4096] independent_features
float32[4096] joint_features
float32[1024] fused_features
# Source identities, preprocessing, domain limitations and timing.
string metadata_json
