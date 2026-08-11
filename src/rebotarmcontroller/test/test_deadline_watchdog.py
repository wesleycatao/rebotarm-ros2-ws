from __future__ import annotations

import importlib.util
from pathlib import Path
import sys

import pytest


MODULE_PATH = (
    Path(__file__).resolve().parents[1]
    / "third_party"
    / "reBotArm_control_py"
    / "reBotArm_control_py"
    / "timing.py"
)
SPEC = importlib.util.spec_from_file_location("rebotarm_loop_timing", MODULE_PATH)
assert SPEC is not None and SPEC.loader is not None
TIMING = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = TIMING
SPEC.loader.exec_module(TIMING)
LoopTimingMonitor = TIMING.LoopTimingMonitor


def test_repeated_deadline_violations_latch_once():
    faults = []
    monitor = LoopTimingMonitor(
        0.010,
        deadline_sec=0.020,
        violation_limit=2,
        on_fault=faults.append,
    )

    monitor.observe_cycle(1.000, 0.001)
    monitor.observe_cycle(1.021, 0.001)
    monitor.observe_cycle(1.042, 0.001)
    monitor.observe_cycle(1.063, 0.001)

    snapshot = monitor.snapshot()
    assert len(faults) == 1
    assert snapshot.fault_latched is True
    assert snapshot.deadline_misses == 3
    assert snapshot.lost_cycles == 3


def test_long_stall_counts_lost_cycles_and_trips_immediately():
    faults = []
    monitor = LoopTimingMonitor(
        0.002,
        deadline_sec=0.006,
        violation_limit=3,
        on_fault=faults.append,
    )

    monitor.observe_cycle(5.000, 0.0005)
    monitor.observe_cycle(5.020, 0.0005)

    assert len(faults) == 1
    assert faults[0].snapshot.lost_cycles == 9
    assert faults[0].observed_sec == pytest.approx(0.020)


def test_jitter_percentiles_and_maximum_are_bounded_window_statistics():
    monitor = LoopTimingMonitor(0.010, window_size=32)
    starts = [0.000, 0.010, 0.021, 0.033, 0.043]
    for start in starts:
        monitor.observe_cycle(start, 0.001)

    snapshot = monitor.snapshot()
    assert snapshot.sample_count == len(starts)
    assert snapshot.max_jitter_sec == pytest.approx(0.002)
    assert 0.001 <= snapshot.p95_jitter_sec <= 0.002
    assert 0.001 <= snapshot.p99_jitter_sec <= 0.002


def test_external_stall_is_latched_and_runtime_reset_rearms_monitor():
    faults = []
    monitor = LoopTimingMonitor(
        0.010,
        deadline_sec=0.030,
        on_fault=faults.append,
    )

    monitor.report_stall(0.050)
    monitor.report_stall(0.060)
    assert len(faults) == 1

    monitor.reset_runtime()
    monitor.report_stall(0.040)
    assert len(faults) == 2
    assert monitor.snapshot().lost_cycles == 7


def test_rate_reconfiguration_preserves_registered_fault_callback():
    faults = []
    monitor = LoopTimingMonitor(
        0.010,
        deadline_sec=0.015,
        on_fault=faults.append,
    )
    monitor.configure(nominal_period_sec=0.005)

    monitor.observe_cycle(0.000, 0.001)
    monitor.observe_cycle(0.020, 0.001)

    assert len(faults) == 1
