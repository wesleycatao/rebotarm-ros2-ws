"""Regression tests for recipes, production RBAC and durable alarms."""

from __future__ import annotations

import inspect
import os
import sys
import tempfile
from pathlib import Path
from types import SimpleNamespace

WORKSPACE = Path(__file__).resolve().parents[3]
for package in (
    WORKSPACE / "src" / "rebotarm_gui",
    WORKSPACE / "src" / "rebotarmcontroller",
):
    if str(package) not in sys.path:
        sys.path.insert(0, str(package))

from rebotarm_gui.motion import MoveStep, Program  # noqa: E402
from rebotarm_gui.motion.recovery import RecoveryCheckpoint, RecoveryStore  # noqa: E402
from rebotarm_gui.security import (  # noqa: E402
    AuthorizationContext,
    CAP_EXECUTE_RELEASED,
    CAP_MANUAL_MOTION,
    CredentialStore,
    ROLE_OPERATOR,
    ROLE_SUPERVISOR,
    Identity,
    signed_lease,
)
from rebotarm_gui.widgets.alarm import (  # noqa: E402
    Alarm,
    AlarmStore,
    STATE_ACKNOWLEDGED,
)
from rebotarm_gui.ros_bridge import RosBridge, _BridgeNode  # noqa: E402
from rebotarmcontroller.authorization import AuthorizationGuard  # noqa: E402


def test_released_recipe_is_integrity_checked_and_revisioned():
    recipe = Program(name="pick", steps=[MoveStep(label="approach")])
    digest = recipe.release("supervisor")

    assert len(digest) == 64
    assert recipe.verify_release() == (True, "")
    restored = Program.from_json(recipe.to_json())
    assert restored.verify_release() == (True, "")

    restored.steps[0].duration += 0.1
    valid, reason = restored.verify_release()
    assert not valid
    assert "alterado" in reason

    restored.start_new_revision()
    assert restored.revision == 2
    assert restored.release_hash == ""


def test_credentials_and_production_capability_gate(monkeypatch):
    with tempfile.TemporaryDirectory() as directory:
        monkeypatch.setenv(
            "REBOTARM_AUTH_SECRET_PATH", str(Path(directory) / "authorization.key")
        )
        store = CredentialStore(Path(directory) / "auth.json")
        store.set_user("alice", ROLE_SUPERVISOR, "123456")
        identity = store.authenticate("alice", "123456")
        assert identity.authenticated
        assert identity.role == ROLE_SUPERVISOR
        assert not store.authenticate("alice", "bad-pin").authenticated

    context = AuthorizationContext()
    context.set_identity(Identity("operator", ROLE_OPERATOR, True))
    context.set_production_mode(True)
    assert context.permits(CAP_EXECUTE_RELEASED)
    assert not context.permits(CAP_MANUAL_MOTION)

    enforced = AuthorizationContext(enforce_rbac=True)
    enforced.set_identity(Identity("operator", ROLE_OPERATOR, True))
    assert enforced.permits(CAP_MANUAL_MOTION)
    assert not enforced.permits("raw_control")


def test_signed_controller_lease_activates_fail_closed_guard(monkeypatch, tmp_path):
    secret = tmp_path / "authorization.key"
    secret.write_bytes(os.urandom(32))
    monkeypatch.setenv("REBOTARM_AUTH_SECRET_PATH", str(secret))
    identity = Identity("alice", ROLE_SUPERVISOR, True)
    guard = AuthorizationGuard(required=False, secret_path=secret)

    lease = signed_lease(identity, production_mode=True)
    valid, _detail = guard.update(lease)

    assert valid
    assert guard.required
    assert guard.permits(CAP_EXECUTE_RELEASED)
    assert not guard.permits(CAP_MANUAL_MOTION)
    replayed, _detail = guard.update(lease)
    assert not replayed


def test_alarm_store_replays_latest_lifecycle_state(tmp_path):
    store = AlarmStore(tmp_path / "alarms.jsonl")
    alarm = Alarm(message="heartbeat perdido", code="HB")
    assert store.append(alarm, "raised")
    alarm.state = STATE_ACKNOWLEDGED
    alarm.acknowledged = True
    assert store.append(alarm, "acknowledged")

    loaded = store.load()

    assert len(loaded) == 1
    assert loaded[0].alarm_id == alarm.alarm_id
    assert loaded[0].state == STATE_ACKNOWLEDGED
    assert store.active() == []


def test_moveit_result_code_is_understood_by_action_parser():
    wrapped = SimpleNamespace(
        status=4,  # GoalStatus.STATUS_SUCCEEDED
        result=SimpleNamespace(
            error_code=SimpleNamespace(val=1, message="SUCCESS")
        ),
    )

    success, message, _pose = RosBridge._parse_action_result("move_group", wrapped)

    assert success
    assert message == "SUCCESS"


def test_moveit_goal_defaults_match_robot_semantic_model():
    parameters = inspect.signature(
        _BridgeNode.send_collision_planned_pose
    ).parameters

    assert parameters["base_frame"].default == "base_link"
    assert parameters["end_link"].default == "gripper_tcp"
    assert parameters["group_name"].default == "arm"


def test_recovery_checkpoint_is_persisted_and_cleared(tmp_path):
    store = RecoveryStore(tmp_path / "checkpoint.json")
    checkpoint = RecoveryCheckpoint(
        program_path="/tmp/recipe.json",
        program_hash="a" * 64,
        root_step_index=2,
        opcode_index=7,
        step_id="move-3",
        reason="heartbeat perdido",
        run_id="run-1",
    )

    store.save(checkpoint)
    assert store.load() == checkpoint
    assert (store.path.stat().st_mode & 0o077) == 0
    store.clear()
    assert store.load() is None
