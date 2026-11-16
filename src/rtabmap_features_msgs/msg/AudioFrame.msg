# Header stamp is the first PCM sample time, not the publication time.
std_msgs/Header header
uint32 sample_rate
uint16 channels
# Interleaved float32 PCM in [-1,1], length divisible by channels.
float32[] samples
string source_id
# Example: hardware_capture, host_read_end_estimate, synthetic_test_clock.
string stamp_quality
