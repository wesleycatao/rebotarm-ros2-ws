from __future__ import annotations

import argparse
import csv
from dataclasses import dataclass
from datetime import datetime, timezone
import math
from pathlib import Path
import re
import statistics
import time
from typing import Any

import numpy as np
import yaml


PROFILE_VERSION = 2
DEFAULT_STEP_ID = "default"
SEVERITY_OK = "OK"
SEVERITY_WARN = "WARN"
SEVERITY_FAIL = "FAIL"

CSV_FIELDS = (
    "run_id",
    "program",
    "step_id",
    "step_label",
    "payload_name",
    "payload_kg",
    "sample_index",
    "stamp_sec",
    "elapsed_sec",
    "joint_name",
    "position",
    "velocity",
    "torque",
)


@dataclass(frozen=True)
class TorqueLimit:
    joint_name: str
    torque: float
    warn_min: float
    warn_max: float
    fail_min: float
    fail_max: float
    status: str
    phase: float

    def message(self) -> str:
        if self.status == SEVERITY_OK:
            return f"{self.joint_name}: OK"
        if self.status == SEVERITY_WARN:
            return (
                f"{self.joint_name}: WARN torque={self.torque:+.3f} outside "
                f"[{self.warn_min:+.3f}, {self.warn_max:+.3f}]"
            )
        return (
            f"{self.joint_name}: FAIL torque={self.torque:+.3f} outside "
            f"[{self.fail_min:+.3f}, {self.fail_max:+.3f}]"
        )


class TorqueProfile:
    """Per-step WARN/FAIL torque profile learned from supervised executions."""

    def __init__(self, data: dict[str, Any]) -> None:
        version = int(data.get("version", 0))
        if version == 1:
            data = _upgrade_v1_profile(data)
        if int(data.get("version", 0)) != PROFILE_VERSION:
            raise ValueError(f"unsupported torque profile version: {data.get('version')!r}")
        self.data = data
        self.defaults = data.get("defaults", {}) or {}
        self.steps = data.get("steps", {}) or {}

    @classmethod
    def load(cls, path: str | Path) -> "TorqueProfile":
        with open(Path(path).expanduser(), "r", encoding="utf-8") as f:
            return cls(yaml.safe_load(f) or {})

    def step_ids(self) -> list[str]:
        return list(self.steps.keys())

    def has_step(self, step_id: str) -> bool:
        return self._step(step_id) is not None

    def _step(self, step_id: str) -> dict[str, Any] | None:
        if step_id in self.steps:
            return self.steps[step_id]
        if DEFAULT_STEP_ID in self.steps:
            return self.steps[DEFAULT_STEP_ID]
        if self.steps:
            return next(iter(self.steps.values()))
        return None

    def limits_for(self, step_id: str, joint_name: str, phase: float) -> TorqueLimit:
        step = self._step(step_id)
        if step is None:
            raise KeyError("torque profile does not contain steps")
        joints = step.get("joints", {}) or {}
        joint = joints.get(joint_name)
        if not joint:
            raise KeyError(f"joint {joint_name!r} not present in torque profile step")
        phases = [float(v) for v in joint.get("phase", [])]
        warn_min = [float(v) for v in joint.get("warn_min", [])]
        warn_max = [float(v) for v in joint.get("warn_max", [])]
        fail_min = [float(v) for v in joint.get("fail_min", [])]
        fail_max = [float(v) for v in joint.get("fail_max", [])]
        if not phases:
            raise ValueError(f"invalid torque envelope for joint {joint_name!r}")
        phase = max(0.0, min(1.0, float(phase)))
        return TorqueLimit(
            joint_name=joint_name,
            torque=0.0,
            warn_min=_interp(phases, warn_min, phase),
            warn_max=_interp(phases, warn_max, phase),
            fail_min=_interp(phases, fail_min, phase),
            fail_max=_interp(phases, fail_max, phase),
            status=SEVERITY_OK,
            phase=phase,
        )

    def check(
        self,
        step_id: str,
        phase: float,
        joint_names: list[str],
        torques: list[float],
    ) -> tuple[str, list[TorqueLimit]]:
        results: list[TorqueLimit] = []
        severity = SEVERITY_OK
        for name, torque in zip(joint_names, torques):
            try:
                limits = self.limits_for(step_id, name, phase)
            except KeyError:
                continue
            value = float(torque)
            status = SEVERITY_OK
            if value < limits.fail_min or value > limits.fail_max:
                status = SEVERITY_FAIL
                severity = SEVERITY_FAIL
            elif value < limits.warn_min or value > limits.warn_max:
                status = SEVERITY_WARN
                if severity != SEVERITY_FAIL:
                    severity = SEVERITY_WARN
            results.append(
                TorqueLimit(
                    joint_name=name,
                    torque=value,
                    warn_min=limits.warn_min,
                    warn_max=limits.warn_max,
                    fail_min=limits.fail_min,
                    fail_max=limits.fail_max,
                    status=status,
                    phase=limits.phase,
                )
            )
        return severity, results


class TorqueProfileMonitor:
    """Consecutive-sample monitor around a learned WARN/FAIL profile."""

    def __init__(
        self,
        profile: TorqueProfile | None,
        *,
        mode: str = "off",
        violation_samples: int = 3,
        active_step_id: str = DEFAULT_STEP_ID,
        derivative_limit: float = 0.0,
    ) -> None:
        self.profile = profile
        self.mode = normalize_mode(mode)
        self.violation_samples = max(1, int(violation_samples))
        self.active_step_id = active_step_id or DEFAULT_STEP_ID
        self.derivative_limit = max(0.0, float(derivative_limit))
        self._consecutive_fail = 0
        self.last_severity = SEVERITY_OK
        self.last_results: list[TorqueLimit] = []
        self._last_torque_time: float | None = None
        self._last_torque_values: np.ndarray | None = None
        self._derivative_hit = False

    @property
    def enabled(self) -> bool:
        return self.profile is not None and self.mode in ("warn_only", "stop_on_fail")

    def configure(
        self,
        profile: TorqueProfile | None,
        *,
        mode: str,
        violation_samples: int,
        active_step_id: str = "",
        derivative_limit: float = 0.0,
    ) -> None:
        self.profile = profile
        self.mode = normalize_mode(mode)
        self.violation_samples = max(1, int(violation_samples))
        self.derivative_limit = max(0.0, float(derivative_limit))
        if active_step_id:
            self.active_step_id = active_step_id
        self.reset()

    def set_step(self, step_id: str) -> None:
        self.active_step_id = step_id or DEFAULT_STEP_ID
        self.reset()

    def reset(self) -> None:
        self._consecutive_fail = 0
        self.last_severity = SEVERITY_OK
        self.last_results = []
        self._last_torque_time = None
        self._last_torque_values = None
        self._derivative_hit = False

    def _check_derivative(
        self,
        joint_names: list[str],
        torques: list[float],
        now: float,
    ) -> bool:
        """Return True if any joint torque derivative exceeds the limit."""
        if self.derivative_limit <= 0.0:
            return False
        values = np.asarray(torques, dtype=np.float64)
        if (
            self._last_torque_time is None
            or self._last_torque_values is None
            or len(self._last_torque_values) != len(values)
        ):
            self._last_torque_time = now
            self._last_torque_values = values.copy()
            return False

        dt = now - self._last_torque_time
        if dt <= 1e-9:
            return False

        derivative = np.abs(values - self._last_torque_values) / dt
        self._last_torque_time = now
        self._last_torque_values = values.copy()
        return bool(np.any(derivative > self.derivative_limit))

    def check(
        self,
        phase: float,
        joint_names: list[str],
        torques: list[float],
        *,
        step_id: str = "",
    ) -> tuple[bool, str, list[TorqueLimit]]:
        if not self.enabled or self.profile is None:
            return False, SEVERITY_OK, []
        active_step = step_id or self.active_step_id
        severity, results = self.profile.check(active_step, phase, joint_names, torques)

        now = time.monotonic()
        derivative_hit = self._check_derivative(joint_names, torques, now)
        if derivative_hit and severity != SEVERITY_FAIL:
            # Sudden torque spike not captured by the static envelope.
            severity = SEVERITY_WARN
            self._derivative_hit = True

        self.last_severity = severity
        self.last_results = results
        if severity == SEVERITY_FAIL:
            self._consecutive_fail += 1
        else:
            self._consecutive_fail = 0
        should_stop = (
            self.mode == "stop_on_fail"
            and self._consecutive_fail >= self.violation_samples
        )
        return should_stop, severity, results


def normalize_mode(mode: str) -> str:
    value = str(mode or "off").strip().lower()
    aliases = {
        "warn": "warn_only",
        "warning": "warn_only",
        "stop": "stop_on_fail",
        "fail": "stop_on_fail",
    }
    value = aliases.get(value, value)
    if value not in ("off", "warn_only", "stop_on_fail"):
        return "off"
    return value


def safe_step_id(text: str, fallback: str = DEFAULT_STEP_ID) -> str:
    value = re.sub(r"[^A-Za-z0-9_.-]+", "_", (text or "").strip()).strip("_")
    return value or fallback


def _interp(xs: list[float], ys: list[float], x: float) -> float:
    if len(xs) != len(ys) or not xs:
        raise ValueError("invalid interpolation vectors")
    if x <= xs[0]:
        return ys[0]
    if x >= xs[-1]:
        return ys[-1]
    for index in range(1, len(xs)):
        if x <= xs[index]:
            x0 = xs[index - 1]
            x1 = xs[index]
            y0 = ys[index - 1]
            y1 = ys[index]
            ratio = 0.0 if x1 == x0 else (x - x0) / (x1 - x0)
            return y0 + (y1 - y0) * ratio
    return ys[-1]


def record_profile(args: argparse.Namespace) -> None:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import JointState

    output = Path(args.output).expanduser()
    output.parent.mkdir(parents=True, exist_ok=True)
    run_id = args.run_id or datetime.now().strftime("%Y%m%d_%H%M%S")
    step_id = safe_step_id(args.step_id, DEFAULT_STEP_ID)

    class Recorder(Node):
        def __init__(self) -> None:
            super().__init__("torque_profile_recorder")
            self.rows: list[dict[str, Any]] = []
            self.start_mono: float | None = None
            self.sample_index = 0
            self.deadline = time.monotonic() + args.duration if args.duration > 0.0 else None
            self.create_subscription(
                JointState,
                f"/{args.namespace.strip('/')}/joint_states",
                self._joint_state_cb,
                qos_profile_sensor_data,
            )

        def _joint_state_cb(self, msg: JointState) -> None:
            now = time.monotonic()
            if self.start_mono is None:
                self.start_mono = now
            elapsed = now - self.start_mono
            stamp_sec = float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9
            for index, name in enumerate(msg.name):
                if args.arm_only and str(name).startswith("gripper"):
                    continue
                self.rows.append(
                    {
                        "run_id": run_id,
                        "program": args.program,
                        "step_id": step_id,
                        "step_label": args.step_label,
                        "payload_name": args.payload_name,
                        "payload_kg": f"{args.payload_kg:.6g}",
                        "sample_index": self.sample_index,
                        "stamp_sec": f"{stamp_sec:.9f}",
                        "elapsed_sec": f"{elapsed:.9f}",
                        "joint_name": str(name),
                        "position": f"{float(msg.position[index] if index < len(msg.position) else 0.0):.9f}",
                        "velocity": f"{float(msg.velocity[index] if index < len(msg.velocity) else 0.0):.9f}",
                        "torque": f"{float(msg.effort[index] if index < len(msg.effort) else 0.0):.9f}",
                    }
                )
            self.sample_index += 1

    rclpy.init()
    node = Recorder()
    try:
        node.get_logger().info(f"recording torque run step={step_id} to {output}")
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            if node.deadline is not None and time.monotonic() >= node.deadline:
                break
    finally:
        write_csv(output, node.rows)
        node.get_logger().info(f"wrote {len(node.rows)} torque samples to {output}")
        node.destroy_node()
        rclpy.shutdown()


def write_csv(path: str | Path, rows: list[dict[str, Any]]) -> None:
    output = Path(path).expanduser()
    output.parent.mkdir(parents=True, exist_ok=True)
    with open(output, "w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=CSV_FIELDS)
        writer.writeheader()
        writer.writerows(rows)


def build_profile(args: argparse.Namespace) -> None:
    data = build_profile_data(
        [Path(path).expanduser() for path in args.inputs],
        bins=int(args.bins),
        warn_tolerance_percent=float(args.warn_tolerance_percent),
        fail_tolerance_percent=float(args.fail_tolerance_percent),
        min_margin=float(args.min_margin),
        std_factor=float(args.std_factor),
        violation_samples=int(args.violation_samples),
    )
    output = Path(args.output).expanduser()
    output.parent.mkdir(parents=True, exist_ok=True)
    with open(output, "w", encoding="utf-8") as f:
        yaml.safe_dump(data, f, sort_keys=False)
    print(f"Wrote torque profile: {output}")


def _step_params(
    step_id: str,
    defaults: dict[str, Any],
    step_overrides: dict[str, dict[str, Any]] | None,
) -> dict[str, Any]:
    """Return effective parameters for a step, applying overrides if present."""
    params = dict(defaults)
    overrides = (step_overrides or {}).get(step_id, {})
    for key in ("warn_tolerance_percent", "fail_tolerance_percent", "min_margin"):
        if key in overrides:
            params[key] = float(overrides[key])
    return params


def build_profile_data(
    inputs: list[Path],
    *,
    bins: int = 101,
    warn_tolerance_percent: float = 10.0,
    fail_tolerance_percent: float = 20.0,
    min_margin: float = 0.25,
    std_factor: float = 3.0,
    violation_samples: int = 3,
    step_overrides: dict[str, dict[str, Any]] | None = None,
) -> dict[str, Any]:
    rows = []
    for path in inputs:
        rows.extend(_load_rows(path))
    if not rows:
        raise ValueError("no torque samples found")

    bins = max(2, int(bins))
    phases = [index / float(bins - 1) for index in range(bins)]
    grouped: dict[str, dict[str, list[list[float]]]] = {}
    step_labels: dict[str, str] = {}
    programs = set()
    payload_names = set()
    payload_values = []

    durations: dict[tuple[str, str], float] = {}
    for row in rows:
        key = (row["run_id"], row["step_id"])
        durations[key] = max(durations.get(key, 0.0), row["elapsed_sec"])

    for row in rows:
        step_id = safe_step_id(row["step_id"], DEFAULT_STEP_ID)
        duration = max(durations.get((row["run_id"], step_id), 0.0), 1e-9)
        phase = max(0.0, min(1.0, row["elapsed_sec"] / duration))
        bin_index = min(bins - 1, max(0, int(round(phase * (bins - 1)))))
        joint = row["joint_name"]
        grouped.setdefault(step_id, {}).setdefault(joint, [[] for _ in range(bins)])
        grouped[step_id][joint][bin_index].append(row["torque"])
        if row["step_label"]:
            step_labels[step_id] = row["step_label"]
        programs.add(row["program"])
        payload_names.add(row["payload_name"])
        payload_values.append(row["payload_kg"])

    default_params = {
        "warn_tolerance_percent": float(warn_tolerance_percent),
        "fail_tolerance_percent": float(fail_tolerance_percent),
        "min_margin": float(min_margin),
    }
    steps: dict[str, Any] = {}
    for step_id, joints in sorted(grouped.items()):
        params = _step_params(step_id, default_params, step_overrides)
        step_data = {
            "label": step_labels.get(step_id, step_id),
            "warn_tolerance_percent": params["warn_tolerance_percent"],
            "fail_tolerance_percent": params["fail_tolerance_percent"],
            "min_margin": params["min_margin"],
            "joints": {},
        }
        step_warn = params["warn_tolerance_percent"]
        step_fail = params["fail_tolerance_percent"]
        step_margin = params["min_margin"]
        for joint, values_by_bin in sorted(joints.items()):
            filled = _fill_empty_bins(values_by_bin)
            means = [_mean(values) for values in filled]
            mins = [min(values) for values in filled]
            maxs = [max(values) for values in filled]
            stds = [_stdev(values) for values in filled]
            warn_margin = [
                max(step_margin, abs(means[i]) * step_warn / 100.0, stds[i] * std_factor)
                for i in range(bins)
            ]
            fail_margin = [
                max(
                    warn_margin[i],
                    step_margin,
                    abs(means[i]) * step_fail / 100.0,
                    stds[i] * std_factor,
                )
                for i in range(bins)
            ]
            step_data["joints"][joint] = {
                "phase": [_round(v) for v in phases],
                "torque_mean": [_round(v) for v in means],
                "torque_min": [_round(v) for v in mins],
                "torque_max": [_round(v) for v in maxs],
                "warn_min": [_round(means[i] - warn_margin[i]) for i in range(bins)],
                "warn_max": [_round(means[i] + warn_margin[i]) for i in range(bins)],
                "fail_min": [_round(means[i] - fail_margin[i]) for i in range(bins)],
                "fail_max": [_round(means[i] + fail_margin[i]) for i in range(bins)],
            }
        steps[step_id] = step_data

    payload_kg = sum(payload_values) / len(payload_values) if payload_values else 0.0
    return {
        "version": PROFILE_VERSION,
        "generated_at": _utc_now(),
        "program": _single_or_mixed(programs),
        "payload": {
            "name": _single_or_mixed(payload_names),
            "mass_kg": _round(payload_kg),
        },
        "defaults": {
            "warn_tolerance_percent": float(warn_tolerance_percent),
            "fail_tolerance_percent": float(fail_tolerance_percent),
            "min_margin": float(min_margin),
            "violation_samples": int(violation_samples),
        },
        "phase_bins": bins,
        "sources": [str(path) for path in inputs],
        "steps": steps,
    }


def _load_rows(path: Path) -> list[dict[str, Any]]:
    with open(path, "r", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        rows = []
        for row in reader:
            rows.append(
                {
                    "run_id": row.get("run_id", str(path)),
                    "program": row.get("program", ""),
                    "step_id": safe_step_id(row.get("step_id", ""), DEFAULT_STEP_ID),
                    "step_label": row.get("step_label", ""),
                    "payload_name": row.get("payload_name", ""),
                    "payload_kg": float(row.get("payload_kg") or 0.0),
                    "elapsed_sec": float(row.get("elapsed_sec") or 0.0),
                    "joint_name": str(row.get("joint_name") or ""),
                    "torque": float(row.get("torque") or 0.0),
                }
            )
        return rows


def _fill_empty_bins(values_by_bin: list[list[float]]) -> list[list[float]]:
    filled: list[list[float]] = []
    previous = [0.0]
    for index, values in enumerate(values_by_bin):
        if values:
            previous = values
            filled.append(values)
            continue
        next_values = next((v for v in values_by_bin[index + 1 :] if v), previous)
        filled.append([(_mean(previous) + _mean(next_values)) * 0.5])
    return filled


def _upgrade_v1_profile(data: dict[str, Any]) -> dict[str, Any]:
    joints_v1 = data.get("joints", {}) or {}
    joints = {}
    for name, joint in joints_v1.items():
        phase = joint.get("phase", [])
        mean = joint.get("torque_mean", [])
        warn_min = joint.get("limit_min", [])
        warn_max = joint.get("limit_max", [])
        joints[name] = {
            "phase": phase,
            "torque_mean": mean,
            "torque_min": joint.get("torque_min", mean),
            "torque_max": joint.get("torque_max", mean),
            "warn_min": warn_min,
            "warn_max": warn_max,
            "fail_min": warn_min,
            "fail_max": warn_max,
        }
    return {
        "version": PROFILE_VERSION,
        "generated_at": data.get("generated_at", ""),
        "program": data.get("program", ""),
        "payload": data.get("payload", {}),
        "defaults": {
            "warn_tolerance_percent": 0.0,
            "fail_tolerance_percent": 0.0,
            "min_margin": 0.0,
            "violation_samples": (data.get("policy", {}) or {}).get("violation_samples", 3),
        },
        "phase_bins": data.get("phase_bins", 0),
        "sources": data.get("sources", []),
        "steps": {
            DEFAULT_STEP_ID: {
                "label": "Perfil v1 importado",
                "warn_tolerance_percent": 0.0,
                "fail_tolerance_percent": 0.0,
                "min_margin": 0.0,
                "joints": joints,
            }
        },
    }


def _utc_now() -> str:
    return datetime.now(timezone.utc).replace(microsecond=0).isoformat()


def _mean(values: list[float]) -> float:
    return sum(values) / len(values) if values else 0.0


def _stdev(values: list[float]) -> float:
    return statistics.stdev(values) if len(values) > 1 else 0.0


def _round(value: float) -> float:
    if not math.isfinite(value):
        return 0.0
    return round(float(value), 6)


def _single_or_mixed(values: set[str]) -> str:
    cleaned = {value for value in values if value}
    if len(cleaned) == 1:
        return next(iter(cleaned))
    if not cleaned:
        return ""
    return "mixed"


def _float_arg(value: str) -> float:
    try:
        return float(value)
    except ValueError as exc:
        raise argparse.ArgumentTypeError(str(exc)) from exc


def make_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Record supervised torque runs and build reBotArm safety profiles."
    )
    subparsers = parser.add_subparsers(dest="command", required=True)

    record = subparsers.add_parser("record", help="record a supervised run to CSV")
    record.add_argument("--namespace", default="rebotarm")
    record.add_argument("--output", required=True)
    record.add_argument("--duration", type=_float_arg, default=0.0)
    record.add_argument("--run-id", default="")
    record.add_argument("--program", default="")
    record.add_argument("--step-id", default=DEFAULT_STEP_ID)
    record.add_argument("--step-label", default="")
    record.add_argument("--payload-name", default="")
    record.add_argument("--payload-kg", type=_float_arg, default=0.0)
    record.add_argument(
        "--include-gripper",
        action="store_false",
        dest="arm_only",
        help="also record gripper joints from /joint_states",
    )
    record.set_defaults(arm_only=True)
    record.set_defaults(func=record_profile)

    build = subparsers.add_parser("build", help="build a YAML WARN/FAIL profile")
    build.add_argument("inputs", nargs="+")
    build.add_argument("--output", required=True)
    build.add_argument("--bins", type=int, default=101)
    build.add_argument("--warn-tolerance-percent", type=_float_arg, default=10.0)
    build.add_argument("--fail-tolerance-percent", type=_float_arg, default=20.0)
    build.add_argument("--min-margin", type=_float_arg, default=0.25)
    build.add_argument("--std-factor", type=_float_arg, default=3.0)
    build.add_argument("--violation-samples", type=int, default=3)
    build.set_defaults(func=build_profile)
    return parser


def main(argv: list[str] | None = None) -> None:
    parser = make_arg_parser()
    args = parser.parse_args(argv)
    args.func(args)


if __name__ == "__main__":
    main()
