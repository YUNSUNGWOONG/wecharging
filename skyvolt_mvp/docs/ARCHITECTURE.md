# Architecture

## Why these modules

The paper presents a single demo robot. Our MVP target is a **multi-robot
product** that runs unattended in an underground parking lot. The architecture
choices below all flow from that delta.

### Pure-Python core, ROS at the edges

The IP — track localization, two-stage deceleration, fleet scheduling — is
written as plain Python with no ROS dependency. ROS 2 nodes are thin wrappers
that wire topics into these classes. Three reasons:

1. **Iteration speed.** Re-running `pytest` is < 1 s; rebuilding a colcon
   workspace is ~30 s.
2. **Replay testing.** We can drive recorded sensor sequences through the same
   `Localizer` instance offline, with no robot or Gazebo running.
3. **Hardware portability.** The MCU/PLC team can re-implement only the I/O
   layer; the algorithms move with us if we change runtimes.

### Track-arclength state space

A robot suspended from a closed-loop track has 1 effective DoF (arclength `s`)
plus a discrete branch index when junctions exist. Modelling this directly:

- Localizer state is `(s, ds, branch_id)`, not `(x, y, θ)`.
- All sensors (RFID cue/positioning, photoelectric docking) report
  observations against `s`.
- Scheduler routes are sequences of `(branch_id, s_start, s_end, time_window)`
  reservations.

This collapses the planning problem to 1D, which is what makes 100-robot
scheduling tractable later.

### Two-stage deceleration as a state machine, **not** if/else

Paper's logic: `cruise → cue card → decelerate → positioning card →
decelerate → docking card → stop`. We implement this as an explicit FSM
(`SpeedPolicyFSM`) with:

- Named states (`CRUISE`, `APPROACH_1`, `APPROACH_2`, `DOCKED`, `FAULT`)
- Guarded transitions (each transition has a precondition fn)
- An `on_step` that returns commanded velocity given fused localizer estimate

This gives us:
- Property-based tests (state can only progress in valid order)
- Fault injection (drop an RFID, stuck photoelectric — does it recover or
  enter `FAULT` cleanly?)
- A single source of truth that hardware and sim share

### Reservation-based fleet scheduling

To run >1 robot on a shared track without collision, we maintain a
**time-window reservation table** keyed on `(branch_id, segment_id)`. New jobs
are admitted only if the proposed schedule fits the table without overlap.
This is the same technique as airport tarmac scheduling and warehouse AGVs.

The first scheduler is greedy ("nearest-pile-first then earliest-feasible-
window"). It is enough to demo and to gather metrics. We expect to swap it
for an auction or MILP-based one as load grows; the interface lives in
`scheduler.py:Scheduler.assign(jobs, fleet_state)`.

### Sim is kinematic for now

Physics-accurate wheel-rail contact in Gazebo is achievable but expensive to
debug, and it is *not* what gates the MVP. We model the robot as a body
constrained to the track centerline, with arclength velocity updated from
the speed policy. The Gazebo plugin is a small `WorldPosePublisher` driven
by ROS topics. When dynamics matter (e.g. acceleration profile validation
against motor torque), we lift the constraint and switch to a
contact-physics URDF — the localizer/policy code does not change.

## Data flow

```
  ┌─────────────┐    /rfid      ┌──────────────┐
  │ Gazebo      ├──────────────▶│              │
  │ sim plugins │   /photoeye   │  Localizer   │ /track_state
  │             ├──────────────▶│  (KF on s)   ├────────────┐
  │             │   /odom       │              │            │
  │             ├──────────────▶│              │            ▼
  └─────────────┘               └──────────────┘    ┌──────────────┐
       ▲                                            │ SpeedPolicy  │
       │ /cmd_vel                                   │ FSM (per     │
       └────────────────────────────────────────────┤ robot)       │
                                                    └──────────────┘
                                                           ▲
                                                           │ /goal_segment
                                                    ┌──────┴───────┐
                                                    │ FleetManager │
                                                    │ + Scheduler  │◀── REST
                                                    │ + Reservation│
                                                    └──────────────┘
```

## What we deliberately did not build (yet)

- Real wheel-rail contact dynamics
- The Z-axis charging insertion (second paper) — placeholder API only
- BMS / charge-session management — out of scope until plug-in works
- Auth on the REST API — single-tenant assumption for MVP
