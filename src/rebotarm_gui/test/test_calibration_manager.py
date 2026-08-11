"""Regression tests for versioned work-plane calibration profiles."""

from __future__ import annotations

from datetime import datetime, timedelta, timezone
from pathlib import Path
import json
import sys

import pytest


WORKSPACE = Path(__file__).resolve().parents[3]
SRC_PKG = WORKSPACE / "src" / "rebotarm_gui"
if str(SRC_PKG) not in sys.path:
    sys.path.insert(0, str(SRC_PKG))

from rebotarm_gui.calibration import (  # noqa: E402
    CalibrationPoint,
    CalibrationProfile,
    CalibrationStore,
    STATUS_APPROVED,
)


def _profile(*, bad_sample: bool = False) -> CalibrationProfile:
    pixels = [
        (80.0, 60.0),
        (560.0, 60.0),
        (560.0, 420.0),
        (80.0, 420.0),
        (320.0, 240.0),
        (320.0, 90.0),
    ]

    def project(u: float, v: float) -> tuple[float, float]:
        return 0.50 - 0.0005 * v, 0.25 - 0.0008 * u

    points = []
    for index, (u, v) in enumerate(pixels):
        x, y = project(u, v)
        if bad_sample and index == len(pixels) - 1:
            x += 0.08
            y -= 0.05
        points.append(CalibrationPoint(u=u, v=v, x=x, y=y))
    return CalibrationProfile(
        name="camera mesa",
        camera_frame="camera_link",
        target_frame="base_link",
        image_width=640,
        image_height=480,
        tcp_frame="gripper_tcp",
        tool_id="parallel_gripper",
        points=points,
    )


def test_fit_reports_metric_and_rejects_one_outlier() -> None:
    profile = _profile(bad_sample=True)

    profile.fit()

    assert len(profile.homography) == 9
    assert profile.metrics.sample_count == 6
    assert profile.metrics.inlier_count == 5
    assert profile.metrics.rms_error_m < 1e-8
    assert profile.metrics.max_error_m < 1e-8
    assert profile.metrics.pixel_coverage > 0.4
    assert profile.metrics.inliers.count(False) == 1
    assert profile.approval_readiness()[0]


def test_four_points_fit_but_cannot_be_approved() -> None:
    profile = _profile()
    profile.points = profile.points[:4]

    profile.fit()
    ready, reason = profile.approval_readiness()

    assert not ready
    assert "cinco" in reason or "5" in reason


def test_approved_profile_is_atomic_active_and_integrity_checked(tmp_path) -> None:
    store = CalibrationStore(tmp_path)
    profile = _profile()
    profile.fit()
    digest = profile.approve("supervisor")

    path = store.save(profile, activate=True)
    active = store.active()

    assert path.is_file()
    assert (path.stat().st_mode & 0o077) == 0
    assert active is not None
    assert active.status == STATUS_APPROVED
    assert active.integrity_hash == digest
    assert active.production_status(
        camera_frame="camera_link",
        image_width=640,
        image_height=480,
        target_frame="base_link",
        tcp_frame="gripper_tcp",
        tool_id="parallel_gripper",
    )[0]

    payload = json.loads(path.read_text(encoding="utf-8"))
    payload["table_z_m"] = 0.123
    path.write_text(json.dumps(payload), encoding="utf-8")

    assert store.active() is None
    assert "integridade" in store.last_error or "mudou" in store.last_error


def test_production_gate_checks_resolution_frame_tool_and_expiration() -> None:
    profile = _profile()
    profile.fit()
    profile.approve("supervisor")

    valid, _ = profile.production_status(
        camera_frame="camera_link",
        image_width=640,
        image_height=480,
        target_frame="base_link",
        tcp_frame="gripper_tcp",
        tool_id="parallel_gripper",
    )
    assert valid
    assert not profile.production_status(image_width=1920)[0]
    assert not profile.production_status(camera_frame="new_camera")[0]
    assert not profile.production_status(tool_id="vacuum")[0]

    profile.expires_at_utc = (
        datetime.now(timezone.utc) - timedelta(seconds=1)
    ).isoformat().replace("+00:00", "Z")
    # Re-hash to isolate expiration from tamper detection.
    profile.integrity_hash = profile.compute_integrity_hash()
    assert not profile.production_status()[0]
    assert "vencido" in profile.production_status()[1]


def test_degenerate_or_duplicate_samples_fail_closed() -> None:
    profile = _profile()
    profile.points[1] = CalibrationPoint(
        u=profile.points[0].u,
        v=profile.points[0].v,
        x=profile.points[1].x,
        y=profile.points[1].y,
    )

    with pytest.raises(ValueError, match="duplicados"):
        profile.fit()

