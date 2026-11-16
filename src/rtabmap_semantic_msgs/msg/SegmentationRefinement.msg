uint32 instance_id
int32 class_id
float32 score
# Fixed order: rgb, nir, lidar, audio. Audio object association is not supported yet.
bool[4] valid_mask
float32[4] weights
# Acquisition times expressed in the RGB clock domain, after clock correction.
float64[4] source_stamps
bool spatial_alignment_valid
bool time_alignment_valid
