# Docking FSM state, exposed for telemetry/dashboards.
uint8 STATE_IDLE=0
uint8 STATE_CRUISE=1
uint8 STATE_APPROACH_1=2
uint8 STATE_APPROACH_2=3
uint8 STATE_DOCKED=4
uint8 STATE_FAULT=255

builtin_interfaces/Time stamp
string robot_id
uint8 state
float64 lateral_error_mm        # last measured (-1 if unknown)
float64 longitudinal_error_mm   # last measured (-1 if unknown)
