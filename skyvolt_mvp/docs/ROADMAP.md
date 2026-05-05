# Roadmap

## Phase 0 — Digital twin (this repo, complete)
- ROS 2 workspace skeleton
- URDF for robot, track (straight + 0.8 m curve), box-type pile
- Gazebo Garden world `parking_lot.sdf`
- Pure-Python algorithm libraries with unit tests
- Eval harness with regression gates

## Phase 1 — Hardening Localization (next 3-4 weeks)
- [ ] Replace synthetic RFID model with measured noise distribution from a
      benchtop IQT1-FP-R4-V1 (paper's part)
- [ ] Add IMU + wheel-encoder fusion to KF for in-curve estimation
- [ ] Property-based fault injection tests (Hypothesis lib)
- [ ] Replay tooling: bag → offline localizer → same metrics

## Phase 2 — Multi-robot in sim (4-6 weeks)
- [ ] Closed-loop track in Gazebo with 3+ robots
- [ ] Reservation deadlocks: timeout + recovery policy
- [ ] Scheduler stress test: 1000 jobs, 10 robots, deadlock-free target
- [ ] REST API consumed by a tiny operator dashboard (read-only first)

## Phase 3 — Manipulator + plug-in (8 weeks)
- [ ] Telescopic mechanism URDF with the actual lead-screw kinematics
- [ ] Vision module (paper 2: Z-Axis Charging) — RGB-D detection of pile pose
- [ ] Closed-loop visual servoing for plug insertion
- [ ] Force/torque feedback for misalignment detection

## Phase 4 — Operations & safety (parallel, ongoing)
- [ ] Track-tilt + load monitoring (each rail support is a load cell)
- [ ] E-stop chain spec (hardware + ROS-side software fallback)
- [ ] OTA firmware update path
- [ ] Prometheus metrics + Grafana board for fleet health

## Decision log

| Date     | Decision                                             | Why |
|----------|------------------------------------------------------|-----|
| Phase 0  | Use Gazebo (Garden) over Isaac Sim                   | Lower GPU floor, faster headless CI |
| Phase 0  | Pure-Python algorithm core                            | Iteration speed; portability to MCU later |
| Phase 0  | Track-arclength 1D state                              | Collapses planning to 1D for fleet |
| Phase 0  | Greedy scheduler first                                | Establish baseline metrics before MILP |
| Phase 0  | Kinematic sim, not contact physics                    | MVP gates don't need it |
