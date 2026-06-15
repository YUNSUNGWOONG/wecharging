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
- [x] Add IMU + wheel-encoder fusion to KF for in-curve estimation
      → accelerometer as predict() control input; gyro→velocity on curves
        (update_gyro); localizer_node subscribes /imu
- [x] Property-based fault injection tests (Hypothesis lib)
      → src/skyvolt_localization/test/test_*_properties.py (KF + FSM invariants)
- [x] Replay tooling: bag → offline localizer → same metrics
      → skyvolt_eval/replay.py: JSONL trace format, deterministic offline replay
        (bit-identical to live), optional rosbag2 reader. `python -m skyvolt_eval.replay`

## Phase 2 — Multi-robot in sim (4-6 weeks)
- [x] Closed-loop track in Gazebo with 3+ robots
      → scripts/fleet_director.py drives the Gazebo robots with the real
        Scheduler/ReservationTable (assign→cmd_vel→arrive→release→re-assign);
        robots are kinematic (VelocityControl + gravity-off on the rail).
        Run: ./scripts/run_sim.sh fleet  then  python3 scripts/fleet_director.py
- [x] Reservation deadlocks: timeout + recovery policy
      → ReservationTable.reclaim_expired(now, grace) frees presumed-stuck robots;
        fleet_sim re-queues their jobs (stuck-robot run: wedged off / 100% on)
- [x] Scheduler stress test: 1000 jobs, 10 robots, deadlock-free target
      → skyvolt_fleet/fleet_sim.py: closed-loop time-stepped driver + deadlock
        gate (1000 jobs/10 robots PASS). `python -m skyvolt_fleet.fleet_sim`
- [x] REST API consumed by a tiny operator dashboard (read-only first)
      → GET / serves a vanilla-JS dashboard polling /fleet (track view + stats);
        no controls (read-only). Lives in the FastAPI server + api-server/ copy.

## Phase 3 — Manipulator + plug-in (8 weeks)
- [~] Telescopic mechanism URDF with the actual lead-screw kinematics
      → skyvolt_manipulator package: LeadScrew + TelescopicManipulator kinematics
        (rev<->displacement, torque->thrust, stroke limits, insertion planning),
        14 tests. URDF wiring of lead-screw params still pending.
- [~] Vision module (paper 2: Z-Axis Charging) — RGB-D detection of pile pose
      → skyvolt_vision package: Kabsch/Umeyama 6-DoF port pose from feature
        points (+ reflection guard, fit RMSE), feeds reach/lateral to the
        manipulator. 9 tests. Gazebo RGB-D camera wired (publishes image/depth/
        points over the bridge); image feature front-end still pending.
- [~] Closed-loop visual servoing for plug insertion
      → skyvolt_vision/servo.py: align(proportional)->insert->dock controller,
        consumes the estimated port pose, closed-loop tested vs a kinematic
        plant (50 mm offset -> docked, 1.96 mm final align). 9 tests.
- [~] Force/torque feedback for misalignment detection
      → skyvolt_manipulator/force_guard.py: InsertionForceGuard classifies the
        contact wrench (FREE/SEATING/MISALIGNED/OVERLOAD) into CONTINUE/CORRECT/
        ABORT for the servo loop. 8 tests. Gazebo F/T sensor wired (publishes
        wrench; reads ~0 in the kinematic sim — no contact dynamics).
      → Gazebo: charging_port model added to parking_lot world; RGB-D + F/T
        sensors on the robot, bridged to ROS (verified publishing).

## Phase 4 — Operations & safety (parallel, ongoing)
- [~] Track-tilt + load monitoring (each rail support is a load cell)
      → skyvolt_safety package: LoadMonitor classifies rail-support load cells +
        track tilt (NOMINAL/IMBALANCED/OVERLOAD/TILT_FAULT) into OK/HOLD/ESTOP.
        8 tests. Load-cell/inclinometer drivers + ROS node pending.
- [~] E-stop chain spec (hardware + ROS-side software fallback)
      → skyvolt_safety/estop.py: EStopChain aggregates stop sources (hw button,
        software faults, comms-loss watchdog) into a latching trip; reset only
        when all clear. Feeds from LoadMonitor ESTOP. 9 tests. HW GPIO pending.
- [~] OTA firmware update path
      → skyvolt_safety/ota.py: OtaUpdater staged FSM (download→verify→stage→
        activate→active) with checksum verify + health-check rollback; the
        known-good version commits only on success. 9 tests. Transport pending.
- [~] Prometheus metrics + Grafana board for fleet health
      → skyvolt_safety/metrics.py: MetricsRegistry + fleet_health_text() render
        fleet/safety state (utilization, queue, reservations, e-stop, rail load,
        tilt) as Prometheus exposition text. 7 tests. HTTP /metrics + Grafana
        board pending.

## Decision log

| Date     | Decision                                             | Why |
|----------|------------------------------------------------------|-----|
| Phase 0  | Use Gazebo (Garden) over Isaac Sim                   | Lower GPU floor, faster headless CI |
| Phase 0  | Pure-Python algorithm core                            | Iteration speed; portability to MCU later |
| Phase 0  | Track-arclength 1D state                              | Collapses planning to 1D for fleet |
| Phase 0  | Greedy scheduler first                                | Establish baseline metrics before MILP |
| Phase 0  | Kinematic sim, not contact physics                    | MVP gates don't need it |
