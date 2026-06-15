"""Offline replay tooling:  bag / trace  ->  offline localizer  ->  same metrics.

The localizer is deterministic, so a recorded sensor stream replayed offline
reproduces the *exact* state trajectory the live system produced. That makes
replay a faithful regression and debugging tool: record once (in sim, in Gazebo,
or on the real robot), then re-run the localizer anywhere — no ROS, no Gazebo —
and get bit-identical estimates and the same localization metrics.

Pipeline
    bag (.db3/.mcap)  --bag_to_trace (needs ROS)-->  trace.jsonl
    trace.jsonl       --replay (pure Python)----->  estimates + metrics

A "trace" is a ROS-independent JSONL log: one SensorTick per line. This keeps
the replay/metrics path runnable in CI without ROS; only the optional bag reader
touches rosbag2.

CLI
    python -m skyvolt_eval.replay --record sample.jsonl   # synth a trace
    python -m skyvolt_eval.replay --trace  sample.jsonl   # replay -> metrics
    python -m skyvolt_eval.replay --bag run_bag --record out.jsonl  # bag->trace
"""
from __future__ import annotations

import argparse
import json
import sys
from dataclasses import dataclass, asdict, field
from pathlib import Path
from typing import Optional

import numpy as np

# Make the sibling algorithm package importable standalone (mirrors docking_eval).
sys.path.insert(0, str(Path(__file__).resolve().parent.parent.parent / "skyvolt_localization"))

from skyvolt_localization.track_localizer import (  # noqa: E402
    TrackLocalizer, RfidObservation, PhotoeyeObservation, GyroObservation,
)


# ---------------------------------------------------------------------------
@dataclass
class SensorTick:
    """One time step of recorded sensor inputs (+ optional ground truth).

    Every field except dt is optional: a tick carries whatever fired on it.
    Applied to the localizer in a fixed order so replay is deterministic.
    """
    dt: float
    odom: Optional[float] = None                 # encoder velocity (m/s)
    imu_accel: Optional[float] = None            # tangential accel (m/s^2)
    gyro: Optional[dict] = None                  # {yaw_rate, radius, sigma}
    rfid: Optional[dict] = None                  # {s, sigma, kind}
    photoeye: Optional[dict] = None              # {s, sigma}
    gt_s: Optional[float] = None                 # ground-truth arclength (m)
    ref_s_hat: Optional[float] = None            # live estimate, for consistency

    def to_json(self) -> str:
        return json.dumps({k: v for k, v in asdict(self).items() if v is not None})

    @staticmethod
    def from_json(line: str) -> "SensorTick":
        return SensorTick(**json.loads(line))


# ---------------------------------------------------------------------------
# Trace I/O
def write_trace(ticks: list[SensorTick], path: str | Path) -> None:
    Path(path).write_text("\n".join(t.to_json() for t in ticks) + "\n")


def read_trace(path: str | Path) -> list[SensorTick]:
    text = Path(path).read_text()
    return [SensorTick.from_json(ln) for ln in text.splitlines() if ln.strip()]


# ---------------------------------------------------------------------------
# Replay core
_ORDER = "predict, odom, gyro, rfid, photoeye"  # documented canonical order


def _step(loc: TrackLocalizer, t: SensorTick) -> float:
    """Apply one tick to the localizer; return the resulting s estimate."""
    loc.predict(t.dt, accel_mps2=t.imu_accel)
    if t.odom is not None:
        loc.update_odom(t.odom)
    if t.gyro is not None:
        loc.update_gyro(GyroObservation(
            yaw_rate_rps=t.gyro["yaw_rate"],
            curve_radius_m=t.gyro.get("radius"),
            sigma_rps=t.gyro.get("sigma", 0.01)))
    if t.rfid is not None:
        loc.update_rfid(RfidObservation(
            tag_arclength_m=t.rfid["s"],
            sigma_m=t.rfid.get("sigma", 0.025),
            kind=t.rfid.get("kind", "cue")))
    if t.photoeye is not None:
        loc.update_photoeye(PhotoeyeObservation(
            docking_arclength_m=t.photoeye["s"],
            sigma_m=t.photoeye.get("sigma", 0.001), triggered=True))
    return loc.state[0]


def replay(ticks: list[SensorTick],
           initial_s: float = 0.0, initial_ds: float = 0.0) -> list[float]:
    """Run the offline localizer over a trace; return the s estimate per tick."""
    loc = TrackLocalizer(initial_s=initial_s, initial_ds=initial_ds)
    return [_step(loc, t) for t in ticks]


def replay_metrics(ticks: list[SensorTick], **kw) -> dict:
    """Replay and summarize localization error against the recorded ground truth."""
    est = replay(ticks, **kw)
    err_mm = [abs(s - t.gt_s) * 1000.0
              for s, t in zip(est, ticks) if t.gt_s is not None]
    final_err = None
    for s, t in zip(reversed(est), reversed(ticks)):
        if t.gt_s is not None:
            final_err = abs(s - t.gt_s) * 1000.0
            break
    if not err_mm:
        return {"n_ticks": len(ticks), "n_with_gt": 0}
    a = np.asarray(err_mm)
    return {
        "n_ticks": len(ticks),
        "n_with_gt": len(err_mm),
        "loc_err_mm": {
            "rmse": float(np.sqrt(np.mean(a ** 2))),
            "p50": float(np.percentile(a, 50)),
            "p99": float(np.percentile(a, 99)),
            "max": float(a.max()),
        },
        "final_loc_err_mm": final_err,
    }


# ---------------------------------------------------------------------------
# Recording: synthesize a trace from the docking plant (stand-in for a bag).
def simulate_trace(seed: int = 0, with_imu: bool = False,
                   dt: float = 0.02) -> list[SensorTick]:
    """Drive the docking plant and record every tick as a SensorTick, including
    the live localizer estimate (ref_s_hat) so replay can be checked against it."""
    from skyvolt_eval.docking_eval import TrialConfig
    from skyvolt_localization.speed_policy import SpeedPolicyFSM, DockingState

    cfg = TrialConfig(dt_s=dt)
    rng = np.random.default_rng(seed)
    loc = TrackLocalizer(initial_s=0.0, initial_ds=0.0)
    fsm = SpeedPolicyFSM()
    fsm.set_goal(cfg.target_arclength_m, cfg.cue_arclength_m, cfg.pos_arclength_m)

    s_true = v_true = 0.0
    cue_seen = pos_seen = photoeye_done = False
    elapsed = 0.0
    ticks: list[SensorTick] = []

    while elapsed < 30.0:
        on_cue = (not cue_seen
                  and abs(s_true - cfg.cue_arclength_m) < cfg.rfid_cone_radius_m)
        on_pos = (not pos_seen
                  and abs(s_true - cfg.pos_arclength_m) < cfg.rfid_cone_radius_m)
        photo = (not photoeye_done
                 and s_true >= cfg.photoeye_arclength_m
                 + rng.normal(0, cfg.photoeye_jitter_m))

        tick = SensorTick(dt=cfg.dt_s, gt_s=s_true)
        tick.odom = v_true + rng.normal(0, cfg.encoder_noise_sigma)
        if with_imu:
            # Tangential accel from the commanded plant; noisy IMU.
            tick.imu_accel = float((v_true - ticks[-1].odom) / cfg.dt_s) \
                if ticks and ticks[-1].odom is not None else 0.0
            tick.imu_accel += rng.normal(0, 0.05)
        if on_cue:
            tick.rfid = {"s": cfg.cue_arclength_m, "sigma": 0.025, "kind": "cue"}
            cue_seen = True
        if on_pos:
            tick.rfid = {"s": cfg.pos_arclength_m, "sigma": 0.025,
                         "kind": "positioning"}
            pos_seen = True
        if photo:
            tick.photoeye = {"s": cfg.photoeye_arclength_m, "sigma": 0.001}
            photoeye_done = True

        tick.ref_s_hat = _step(loc, tick)   # live estimate, recorded for checks
        cmd_v = fsm.step(s_hat=tick.ref_s_hat, ds_hat=loc.state[1],
                         on_cue=on_cue, on_pos=on_pos, photoeye=photo,
                         dt=cfg.dt_s, on_curve=False)
        ticks.append(tick)

        alpha = min(cfg.dt_s / 0.05, 1.0)
        v_true = (1 - alpha) * v_true + alpha * cmd_v
        s_true += v_true * cfg.dt_s
        elapsed += cfg.dt_s
        if fsm.state in (DockingState.DOCKED, DockingState.FAULT):
            break
    return ticks


# ---------------------------------------------------------------------------
# Optional: ROS bag -> trace (only runs where rosbag2 is available).
def bag_to_trace(bag_path: str, robot_id: str = "r0") -> list[SensorTick]:
    """Read a rosbag2 and flatten /odom, /imu, /rfid, /photoeye into ticks.

    Requires ROS 2 (rosbag2_py + message types). Each odom sample opens a new
    tick; other sensor messages attach to the current tick.
    """
    try:
        import rosbag2_py
        from rclpy.serialization import deserialize_message
        from rosidl_runtime_py.utilities import get_message
    except Exception as e:  # pragma: no cover - exercised only with ROS
        raise RuntimeError(
            "bag_to_trace needs ROS 2 (rosbag2_py). Source /opt/ros/<distro>/"
            "setup.bash, or use simulate_trace() for a ROS-free trace.") from e

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=bag_path, storage_id=""),
        rosbag2_py.ConverterOptions("", ""))
    typemap = {t.name: t.type for t in reader.get_all_topics_and_types()}

    ns = f"/{robot_id}"
    ticks: list[SensorTick] = []
    cur: Optional[SensorTick] = None
    last_t_ns: Optional[int] = None
    while reader.has_next():  # pragma: no cover - exercised only with ROS
        topic, data, t_ns = reader.read_next()
        if topic not in typemap:
            continue
        msg = deserialize_message(data, get_message(typemap[topic]))
        if topic == f"{ns}/odom":
            dt = 0.02 if last_t_ns is None else max((t_ns - last_t_ns) * 1e-9, 1e-4)
            last_t_ns = t_ns
            if cur is not None:
                ticks.append(cur)
            cur = SensorTick(dt=dt, odom=float(msg.twist.twist.linear.x))
        elif cur is None:
            continue
        elif topic == f"{ns}/imu":
            cur.imu_accel = float(msg.linear_acceleration.x)
        elif topic == f"{ns}/rfid":
            kind = {1: "cue", 2: "positioning", 3: "docking"}.get(int(msg.kind), "cue")
            cur.rfid = {"s": float(msg.expected_arclength_m), "kind": kind}
        elif topic == f"{ns}/photoeye" and bool(msg.triggered):
            cur.photoeye = {"s": float(getattr(msg, "expected_arclength_m", 0.0))}
    if cur is not None:
        ticks.append(cur)
    return ticks


# ---------------------------------------------------------------------------
def main(argv=None) -> int:
    p = argparse.ArgumentParser(description="SkyvoltRobot offline replay tool")
    p.add_argument("--trace", type=str, help="replay this trace JSONL and print metrics")
    p.add_argument("--bag", type=str, help="rosbag2 path to convert to a trace")
    p.add_argument("--record", type=str,
                   help="write a trace JSONL here (from --bag, else a synth sim)")
    p.add_argument("--seed", type=int, default=0, help="seed for synth recording")
    p.add_argument("--imu", action="store_true", help="include IMU in synth recording")
    args = p.parse_args(argv)

    if args.bag:
        ticks = bag_to_trace(args.bag)
        if args.record:
            write_trace(ticks, args.record)
            print(f"wrote {len(ticks)} ticks -> {args.record}")
        else:
            print(json.dumps(replay_metrics(ticks), indent=2))
        return 0

    if args.record and not args.trace:
        ticks = simulate_trace(seed=args.seed, with_imu=args.imu)
        write_trace(ticks, args.record)
        print(f"wrote {len(ticks)} ticks -> {args.record}")
        return 0

    if args.trace:
        ticks = read_trace(args.trace)
        print(json.dumps(replay_metrics(ticks), indent=2))
        return 0

    p.error("nothing to do: pass --trace, --record, and/or --bag")
    return 2


if __name__ == "__main__":  # pragma: no cover
    sys.exit(main())
