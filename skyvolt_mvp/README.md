# SkyvoltRobot MVP

Productization scaffold for the **SkyvoltRobot** rail-mounted EV charging robot
(Hu et al., Procedia CS 250 (2024) 274-281).

This repo is the **Phase 0/1/2 MVP** described in our RE roadmap:

- **Phase 0** — Digital twin in Gazebo (URDF, world, sensor stubs)
- **Phase 1** — Localization & two-stage docking, modular and regression-tested
- **Phase 2** — Fleet scheduler with track-segment reservation (multi-robot)
- **Phase 3** — Vision/Z-axis charging insertion (placeholder, future work)

The deliverable is a ROS 2 (Humble/Iron) workspace plus a Python evaluation
harness so every algorithm change can be measured against the paper's numbers
(`±3 mm` docking, `5 s` telescoping, `1.5 m/s` straight, `1.0 m/s` curve).

## Layout

```
skyvolt_mvp/
├── docs/
│   ├── ARCHITECTURE.md           # design decisions
│   ├── ROADMAP.md                # phases and milestones
│   ├── UNCERTAINTY_INVENTORY.md  # 깨질 수 있는 가정의 living list
│   ├── EXPERIMENT_PROTOCOLS.md   # 100만원 이하 검증 실험 14개
│   └── GO_NO_GO_REPORT.md        # Phase 0 종료 시점 의사결정 템플릿
├── docker/
│   └── Dockerfile          # reproducible env (ROS 2 Humble + Gazebo Garden)
├── src/
│   ├── skyvolt_msgs/       # custom msgs/srvs/actions
│   ├── skyvolt_description/# robot/track/pile xacro
│   ├── skyvolt_sim/        # gazebo worlds + launch
│   ├── skyvolt_localization/# KF + two-stage speed policy (the IP)
│   ├── skyvolt_fleet/      # scheduler + reservation + REST API
│   └── skyvolt_eval/       # regression harness
└── scripts/
    └── run_docking_eval.sh # one-shot regression run
```

## Quick start

### Pure-Python algorithms (no ROS needed)
The localization, scheduler, and evaluation harness are written so that the
**core logic is testable without ROS or Gazebo**. This is intentional: most RE
iterations happen in pure Python.

```bash
cd skyvolt_mvp
python -m pytest src/skyvolt_localization/test
python -m pytest src/skyvolt_fleet/test
python -m src.skyvolt_eval.skyvolt_eval.docking_eval --trials 100
```

### Full ROS 2 + Gazebo
```bash
# inside Docker (recommended) or a Humble + Gazebo Garden host
colcon build
source install/setup.bash
ros2 launch skyvolt_sim single_robot.launch.py
# in another shell
ros2 launch skyvolt_sim fleet.launch.py num_robots:=3
```

## Verification gates

Every PR should keep these green (see `src/skyvolt_eval/`):

| Metric                       | Paper claim | Gate          |
|------------------------------|-------------|---------------|
| Lateral docking deviation    | ±3 mm       | 99th < 4 mm   |
| Longitudinal docking error   | ±10 mm      | 99th < 12 mm  |
| Telescopic cycle time        | 5 s         | mean < 6 s    |
| Single transfer end-to-end   | 11 s        | mean < 13 s   |
| Multi-robot deadlock rate    | n/a (new)   | 0 in 1k jobs  |

## Status

This is the initial scaffold. The IP-critical pieces — `track_localizer.py`,
`speed_policy.py`, `scheduler.py`, `reservation.py` — have working
implementations with unit tests. The Gazebo side is a runnable skeleton:
URDF loads, world spawns, sensor topics publish, but plug-level dynamics
of the manipulator are intentionally a kinematic stub.

See `docs/ROADMAP.md` for what comes next.
