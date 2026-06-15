# RFID tag detection on the track.
#
# Tag kinds (paper section 3):
#   1 = CUE         -> first deceleration trigger
#   2 = POSITIONING -> second deceleration trigger
#   3 = DOCKING     -> served by photoeye, included for completeness
uint8 KIND_CUE=1
uint8 KIND_POSITIONING=2
uint8 KIND_DOCKING=3

builtin_interfaces/Time stamp
string robot_id
uint8 kind
string tag_id           # globally unique tag identifier
float64 expected_arclength_m   # pre-mapped tag arclength along the track
float64 rssi            # for future range estimation
