# A request to deliver a charging pile to a target support point.
string job_id
string requester_id              # vehicle / parking-spot identifier
uint32 target_branch_id
float64 target_arclength_m
builtin_interfaces/Time submitted_at
builtin_interfaces/Time deadline # 0 = no deadline
uint8 priority                   # 0=low, 255=urgent
