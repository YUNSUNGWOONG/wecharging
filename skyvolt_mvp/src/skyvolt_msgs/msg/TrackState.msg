# Localizer output: pose along the track.
builtin_interfaces/Time stamp
string robot_id
uint32 branch_id            # 0 = main loop; reserved for switches
float64 arclength_m         # position along branch centerline
float64 speed_mps           # signed; positive = forward
float64 arclength_var       # KF variance estimate (m^2)
