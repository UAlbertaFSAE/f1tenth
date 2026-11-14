# Persistent identity within map_id; unrelated to detector observation IDs.
uint64 id
geometry_msgs/Point position
float64[4] covariance
string state
# Evidence score, not a calibrated detector probability.
float32 existence_score
string color
float32[4] color_evidence
uint32 independent_hits
