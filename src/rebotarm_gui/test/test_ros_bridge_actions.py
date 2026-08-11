"""Unit coverage for action readiness, deadlines and global cancellation."""

from __future__ import annotations

import hashlib
import json
import sys
import unittest
from pathlib import Path
from types import SimpleNamespace

from action_msgs.msg import GoalStatus

WORKSPACE = Path(__file__).resolve().parents[3]
SRC_PKG = WORKSPACE / "src" / "rebotarm_gui"
if str(SRC_PKG) not in sys.path:
    sys.path.insert(0, str(SRC_PKG))

from rebotarm_gui.ros_bridge import (  # noqa: E402
    RosBridge,
    cell_scene_reconciliation,
    serialize_planning_scene,
    transform_planning_scene_payload,
)
from rebotarm_gui.motion.cell_scene import (  # noqa: E402
    CellObstacle,
    CellScene,
)


class _ImmediateFuture:
    def __init__(self, result=None) -> None:
        self._result = result

    def add_done_callback(self, callback) -> None:
        callback(self)

    def result(self):
        return self._result or SimpleNamespace(goals_canceling=[object()])


class _FakeGoalHandle:
    def __init__(self) -> None:
        self.cancel_calls = 0

    def cancel_goal_async(self):
        self.cancel_calls += 1
        return _ImmediateFuture()


class TestRosBridgeActions(unittest.TestCase):
    def test_cell_scene_reconciliation_only_removes_owned_stale_ids(
        self,
    ) -> None:
        desired, stale = cell_scene_reconciliation(
            {
                "rebotarm_gui__old_fixture",
                "rebotarm_gui__keep_fixture",
                "external_fixture",
            },
            [
                {"object_id": "keep_fixture", "enabled": True},
                {"object_id": "disabled_fixture", "enabled": False},
            ],
        )

        self.assertEqual(desired, {"rebotarm_gui__keep_fixture"})
        self.assertEqual(stale, ("rebotarm_gui__old_fixture",))

    def test_apply_cell_scene_reconciles_before_applying(self) -> None:
        events = []
        observed_scene = SimpleNamespace(
            world=SimpleNamespace(
                collision_objects=[
                    SimpleNamespace(id="rebotarm_gui__old_fixture"),
                    SimpleNamespace(id="external_fixture"),
                ]
            )
        )

        class _FakeNode:
            def __init__(self) -> None:
                self.observed_ids = set()

            def call_get_planning_scene(self):
                events.append("query")
                return _ImmediateFuture(
                    SimpleNamespace(scene=observed_scene)
                )

            def observe_cell_scene_world(self, scene):
                events.append("observe")
                self.observed_ids = {
                    item.id
                    for item in scene.world.collision_objects
                    if item.id.startswith("rebotarm_gui__")
                }

            def call_apply_cell_scene(self, **kwargs):
                events.append("apply")
                _desired, stale = cell_scene_reconciliation(
                    self.observed_ids,
                    kwargs["obstacles"],
                )
                return (
                    _ImmediateFuture(
                        SimpleNamespace(success=True, message="")
                    ),
                    len(stale),
                )

        bridge = RosBridge()
        bridge._node = _FakeNode()
        results = []
        bridge.service_result.connect(
            lambda *args: results.append(args)
        )
        scene = CellScene(
            obstacles=(
                CellObstacle(
                    object_id="new_fixture",
                    name="Nova proteção",
                ),
            )
        )

        self.assertTrue(bridge.apply_cell_scene(scene))
        self.assertEqual(events, ["query", "observe", "apply"])
        self.assertTrue(results[-1][1])
        self.assertIn("1 objeto(s) obsoleto(s)", results[-1][2])

    def test_apply_cell_scene_fails_closed_without_scene_query(self) -> None:
        bridge = RosBridge()
        bridge._node = SimpleNamespace(
            call_get_planning_scene=lambda: None,
        )
        results = []
        bridge.service_result.connect(
            lambda *args: results.append(args)
        )

        self.assertFalse(bridge.apply_cell_scene(CellScene()))
        self.assertFalse(results[-1][1])
        self.assertIn("objetos obsoletos", results[-1][2])

    def test_planning_scene_serializer_separates_shapes_from_ros_messages(
        self,
    ) -> None:
        pose = SimpleNamespace(
            position=SimpleNamespace(x=0.4, y=0.1, z=0.2),
            orientation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0),
        )
        collision_object = SimpleNamespace(
            id="external_table",
            header=SimpleNamespace(frame_id="base_link"),
            pose=SimpleNamespace(
                position=SimpleNamespace(x=1.0, y=0.0, z=0.0),
                orientation=SimpleNamespace(
                    x=0.0, y=0.0, z=0.0, w=1.0
                ),
            ),
            primitives=[SimpleNamespace(type=1, dimensions=[0.8, 0.6, 0.1])],
            primitive_poses=[pose],
            meshes=[],
            mesh_poses=[],
        )
        scene = SimpleNamespace(
            name="cell",
            world=SimpleNamespace(collision_objects=[collision_object]),
            robot_state=SimpleNamespace(attached_collision_objects=[]),
        )
        payload = serialize_planning_scene(scene)
        self.assertEqual(payload["name"], "cell")
        self.assertEqual(len(payload["geometries"]), 1)
        self.assertEqual(payload["geometries"][0]["shape"], "box")
        self.assertEqual(
            payload["geometries"][0]["dimensions"],
            [0.8, 0.6, 0.1],
        )
        self.assertAlmostEqual(payload["geometries"][0]["pose"][0], 1.4)

    def test_planning_scene_geometry_is_resolved_into_display_frame(
        self,
    ) -> None:
        payload = {
            "name": "cell",
            "geometries": [
                {
                    "object_id": "camera_fixture",
                    "frame_id": "camera_link",
                    "shape": "box",
                    "pose": [1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0],
                    "dimensions": [0.1, 0.1, 0.1],
                },
                {
                    "object_id": "unknown_fixture",
                    "frame_id": "unknown_link",
                    "shape": "sphere",
                    "pose": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0],
                    "dimensions": [0.1],
                },
            ],
        }
        # base_link <- camera_link: translation X=1 and +90 degrees on Z.
        result = transform_planning_scene_payload(
            payload,
            "base_link",
            {
                "camera_link": (
                    1.0,
                    0.0,
                    0.0,
                    0.0,
                    0.0,
                    2**-0.5,
                    2**-0.5,
                )
            },
        )

        resolved, unresolved = result["geometries"]
        self.assertEqual(resolved["frame_id"], "base_link")
        self.assertEqual(resolved["source_frame_id"], "camera_link")
        self.assertEqual(resolved["transform_status"], "resolved")
        self.assertAlmostEqual(resolved["pose"][0], 1.0, places=6)
        self.assertAlmostEqual(resolved["pose"][1], 1.0, places=6)
        self.assertEqual(unresolved["frame_id"], "unknown_link")
        self.assertEqual(unresolved["transform_status"], "unresolved")
        self.assertEqual(result["tf_resolved_geometry_count"], 1)
        self.assertEqual(result["tf_unresolved_geometry_count"], 1)

    def test_planning_scene_serializes_planes_and_octomap_audit_data(
        self,
    ) -> None:
        identity = SimpleNamespace(
            position=SimpleNamespace(x=0.0, y=0.0, z=0.2),
            orientation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0),
        )
        collision_object = SimpleNamespace(
            id="floor",
            header=SimpleNamespace(frame_id="/base_link"),
            pose=None,
            primitives=[],
            primitive_poses=[],
            meshes=[],
            mesh_poses=[],
            planes=[SimpleNamespace(coef=[0.0, 0.0, 2.0, 0.0])],
            plane_poses=[identity],
        )
        raw_data = [0, -1, 2, -128, 127]
        world = SimpleNamespace(
            collision_objects=[collision_object],
            octomap=SimpleNamespace(
                header=SimpleNamespace(frame_id="/map"),
                origin=identity,
                octomap=SimpleNamespace(
                    id="OcTree",
                    resolution=0.025,
                    binary=True,
                    data=raw_data,
                ),
            ),
        )
        scene = SimpleNamespace(
            name="mapped_cell",
            world=world,
            robot_state=SimpleNamespace(attached_collision_objects=[]),
        )

        payload = serialize_planning_scene(scene)

        plane = payload["geometries"][0]
        self.assertEqual(plane["shape"], "plane")
        self.assertEqual(plane["frame_id"], "base_link")
        self.assertEqual(
            plane["plane_coefficients"],
            [0.0, 0.0, 1.0, 0.0],
        )
        self.assertEqual(plane["dimensions"], [2.0])
        octomap = payload["octomap"]
        self.assertTrue(octomap["present"])
        self.assertEqual(octomap["frame_id"], "map")
        self.assertEqual(octomap["data_bytes"], len(raw_data))
        self.assertEqual(
            octomap["content_sha256"],
            hashlib.sha256(bytes([0, 255, 2, 128, 127])).hexdigest(),
        )
        self.assertNotIn("data", octomap)

    def test_attached_geometry_uses_link_as_frame_when_header_is_empty(
        self,
    ) -> None:
        pose = SimpleNamespace(
            position=SimpleNamespace(x=0.0, y=0.0, z=0.05),
            orientation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0),
        )
        collision_object = SimpleNamespace(
            id="held_part",
            header=SimpleNamespace(frame_id=""),
            pose=None,
            primitives=[SimpleNamespace(type=1, dimensions=[0.1, 0.1, 0.1])],
            primitive_poses=[pose],
            meshes=[],
            mesh_poses=[],
            planes=[],
        )
        scene = SimpleNamespace(
            name="attached",
            world=SimpleNamespace(collision_objects=[]),
            robot_state=SimpleNamespace(
                attached_collision_objects=[
                    SimpleNamespace(
                        link_name="gripper_link",
                        object=collision_object,
                    )
                ]
            ),
        )

        payload = serialize_planning_scene(scene)

        self.assertEqual(
            payload["geometries"][0]["frame_id"], "gripper_link"
        )
        self.assertEqual(
            payload["geometries"][0]["attached_to"], "gripper_link"
        )

    def test_cached_scene_tf_refresh_is_rate_limited_and_compact(self) -> None:
        bridge = RosBridge()
        raw_payload = {
            "name": "cell",
            "geometries": [
                {
                    "object_id": "held_part",
                    "frame_id": "gripper_link",
                    "shape": "mesh",
                    "pose": [0.0, 0.0, 0.1, 0.0, 0.0, 0.0, 1.0],
                    "vertices": [[0.0, 0.0, 0.0]] * 100,
                    "triangles": [],
                    "attached_to": "gripper_link",
                }
            ],
        }
        calls = []

        class _FakeNode:
            def resolve_planning_scene_frames(
                self,
                payload,
                target_frame,
                *,
                timeout_s,
            ):
                calls.append((payload, target_frame, timeout_s))
                return transform_planning_scene_payload(
                    payload,
                    target_frame,
                    {
                        "gripper_link": (
                            0.5,
                            0.0,
                            0.0,
                            0.0,
                            0.0,
                            0.0,
                            1.0,
                        )
                    },
                )

        bridge._node = _FakeNode()
        enabled, generation = bridge._cache_planning_scene_snapshot(
            raw_payload,
            "base_link",
        )
        bridge._planning_scene_last_tf_refresh = 10.0
        received = []
        bridge.planning_scene_result.connect(
            lambda success, payload, message: received.append(
                (success, payload, message)
            )
        )

        self.assertTrue(enabled)
        self.assertFalse(
            bridge._refresh_cached_planning_scene_transforms(10.19)
        )
        self.assertTrue(
            bridge._refresh_cached_planning_scene_transforms(10.21)
        )
        self.assertEqual(len(calls), 1)
        self.assertEqual(calls[0][1:], ("base_link", 0.0))
        update = received[0][1]
        self.assertNotIn("geometries", update)
        self.assertNotIn("vertices", update["transform_updates"][0])
        self.assertEqual(update["tf_scene_generation"], generation)
        self.assertEqual(update["tf_refresh_sequence"], 1)
        self.assertAlmostEqual(
            update["transform_updates"][0]["pose"][0],
            0.5,
        )

    def test_plan_preview_fails_closed_when_planner_service_is_absent(
        self,
    ) -> None:
        bridge = RosBridge()
        received = []
        bridge.motion_plan_preview_result.connect(
            lambda *args: received.append(args)
        )
        accepted = bridge.request_motion_plan_preview(
            "preview-1",
            [[{"x": 0.1, "y": 0.0, "z": 0.2}]],
        )
        self.assertFalse(accepted)
        self.assertEqual(received[0][0], "preview-1")
        self.assertFalse(received[0][1])
        self.assertIn("indisponível", received[0][3])

    def test_planned_state_sampling_is_bounded_and_keeps_endpoints(self) -> None:
        points = [
            {"positions": [float(index)]}
            for index in range(300)
        ]
        paths = [
            {
                "segments": [
                    {"joint_names": ["joint1"], "points": points}
                ]
            }
        ]
        samples = RosBridge._sample_planned_states(paths, limit=120)
        self.assertEqual(len(samples), 120)
        self.assertEqual(samples[0][1], [0.0])
        self.assertEqual(samples[-1][1], [299.0])

    def test_plan_preview_chains_segments_and_validates_full_robot_states(
        self,
    ) -> None:
        bridge = RosBridge()
        starts = []
        plan_index = 0

        def plan_future(
            _pose,
            _speed,
            *,
            start_joint_names,
            start_positions,
        ):
            nonlocal plan_index
            starts.append((list(start_joint_names), list(start_positions)))
            plan_index += 1
            point = SimpleNamespace(
                positions=[float(plan_index)],
                velocities=[0.1],
                accelerations=[0.2],
                time_from_start=SimpleNamespace(sec=1, nanosec=0),
            )
            response = SimpleNamespace(
                motion_plan_response=SimpleNamespace(
                    error_code=SimpleNamespace(val=1),
                    trajectory=SimpleNamespace(
                        joint_trajectory=SimpleNamespace(
                            joint_names=["joint1"],
                            points=[point],
                        )
                    ),
                    planning_time=0.01,
                    planner_id="test",
                )
            )
            return _ImmediateFuture(response)

        bridge._node = SimpleNamespace(
            cli_get_motion_plan=object(),
            cli_get_state_validity=object(),
            call_motion_plan=plan_future,
            call_state_validity=lambda *_args, **_kwargs: _ImmediateFuture(
                SimpleNamespace(valid=True, contacts=[])
            ),
        )
        received = []
        bridge.motion_plan_preview_result.connect(
            lambda *args: received.append(args)
        )
        accepted = bridge.request_motion_plan_preview(
            "preview-chain",
            [[
                {"x": 0.1, "y": 0.0, "z": 0.2},
                {"x": 0.2, "y": 0.0, "z": 0.2},
            ]],
        )
        self.assertTrue(accepted)
        self.assertEqual(starts[0], ([], []))
        self.assertEqual(starts[1], (["joint1"], [1.0]))
        self.assertEqual(len(received), 1)
        self.assertTrue(received[0][1])
        validation = received[0][2]["state_validation"]
        self.assertTrue(validation["verified"])
        self.assertEqual(validation["sample_count"], 2)
        self.assertFalse(
            received[0][2]["full_robot_clearance"]["available"]
        )

    def test_full_robot_clearance_blocks_below_required_margin(self) -> None:
        bridge = RosBridge()
        bridge._plan_preview_requests["preview-clearance"] = {
            "result_paths": [],
            "sampled_states": [
                (["joint1"], [0.1]),
                (["joint1"], [0.2]),
            ],
            "state_validation": {
                "available": True,
                "verified": True,
                "sample_count": 2,
                "invalid_count": 0,
                "contacts": [],
            },
            "validation_message": "estados válidos",
            "required_clearance_m": 0.05,
        }
        response = SimpleNamespace(
            success=True,
            message="ok",
            available=[True, True],
            distance_m=[0.08, 0.03],
            collision=[False, False],
            body_1=["link1", "link2"],
            body_2=["table", "wall"],
            source=["world", "world"],
            nearest_point_1=[
                SimpleNamespace(x=0.0, y=0.0, z=0.0),
                SimpleNamespace(x=0.1, y=0.2, z=0.3),
            ],
            nearest_point_2=[
                SimpleNamespace(x=0.0, y=0.08, z=0.0),
                SimpleNamespace(x=0.1, y=0.23, z=0.3),
            ],
        )
        received = []
        bridge.motion_plan_preview_result.connect(
            lambda *args: received.append(args)
        )

        bridge._on_clearance_done(
            "preview-clearance", _ImmediateFuture(response)
        )

        self.assertEqual(len(received), 1)
        self.assertFalse(received[0][1])
        clearance = received[0][2]["full_robot_clearance"]
        self.assertTrue(clearance["verified"])
        self.assertFalse(clearance["safe"])
        self.assertAlmostEqual(clearance["minimum_distance_m"], 0.03)
        self.assertEqual(clearance["body_1"], "link2")
        self.assertEqual(clearance["body_2"], "wall")
        self.assertEqual(clearance["limiting_sample"], 1)

    def test_full_robot_clearance_approves_complete_safe_measurement(
        self,
    ) -> None:
        bridge = RosBridge()
        bridge._plan_preview_requests["preview-safe-clearance"] = {
            "result_paths": [],
            "sampled_states": [(["joint1"], [0.1])],
            "state_validation": {
                "available": True,
                "verified": True,
                "sample_count": 1,
                "invalid_count": 0,
                "contacts": [],
            },
            "validation_message": "estado válido",
            "required_clearance_m": 0.04,
        }
        point = SimpleNamespace(x=0.0, y=0.0, z=0.0)
        response = SimpleNamespace(
            success=True,
            message="ok",
            available=[True],
            distance_m=[0.07],
            collision=[False],
            body_1=["link1"],
            body_2=["fixture"],
            source=["world"],
            nearest_point_1=[point],
            nearest_point_2=[SimpleNamespace(x=0.0, y=0.07, z=0.0)],
        )
        received = []
        bridge.motion_plan_preview_result.connect(
            lambda *args: received.append(args)
        )

        bridge._on_clearance_done(
            "preview-safe-clearance", _ImmediateFuture(response)
        )

        self.assertTrue(received[0][1])
        clearance = received[0][2]["full_robot_clearance"]
        self.assertTrue(clearance["safe"])
        self.assertAlmostEqual(clearance["minimum_distance_m"], 0.07)

    def test_global_speed_override_scales_all_time_based_motion_goals(self) -> None:
        bridge = RosBridge()
        sent: dict[str, float] = {}
        timeouts: list[float] = []
        bridge._node = SimpleNamespace(
            send_move_to_pose=(
                lambda _pose, duration, _feedback: sent.__setitem__("pose", duration)
                or object()
            ),
            send_follow_joint_trajectory=(
                lambda _names, _positions, duration: sent.__setitem__("joints", duration)
                or object()
            ),
        )
        bridge._send_action = (
            lambda _label, _future, *, result_timeout_s, tag=None: timeouts.append(
                result_timeout_s
            )
            or True
        )

        self.assertEqual(bridge.set_speed_override(0.25), 0.25)
        self.assertTrue(bridge.move_to_pose(object(), 2.0))
        self.assertTrue(bridge.follow_joint_trajectory(["joint1"], [0.5], 1.0))

        self.assertAlmostEqual(sent["pose"], 8.0)
        self.assertAlmostEqual(sent["joints"], 4.0)
        self.assertAlmostEqual(timeouts[0], 80.0)
        self.assertAlmostEqual(timeouts[1], 50.0)

    def test_global_speed_override_is_clamped_to_slowdown_only_range(self) -> None:
        bridge = RosBridge()
        self.assertEqual(bridge.set_speed_override(2.0), 1.0)
        self.assertEqual(bridge.set_speed_override(0.0), 0.05)

    def test_dynamic_controller_receives_override_and_keeps_nominal_goal_time(self) -> None:
        bridge = RosBridge()
        commands: list[dict] = []
        sent: dict[str, float] = {}
        bridge._node = SimpleNamespace(
            publish_speed_override=lambda raw: commands.append(json.loads(raw)),
            send_move_to_pose=(
                lambda _pose, duration, _feedback: sent.__setitem__(
                    "duration", duration
                )
                or object()
            ),
        )
        bridge._send_action = lambda *_args, **_kwargs: True
        bridge._dynamic_speed_override_supported = True

        self.assertEqual(bridge.set_speed_override(0.4), 0.4)
        self.assertTrue(bridge.move_to_pose(object(), 2.0))

        self.assertEqual(commands[-1]["value"], 0.4)
        self.assertEqual(commands[-1]["version"], 1)
        self.assertGreaterEqual(commands[-1]["sequence"], 1)
        self.assertAlmostEqual(sent["duration"], 2.0)
        self.assertGreaterEqual(bridge.motion_result_timeout(2.0), 45.0)

    def test_readiness_requires_fresh_heartbeat_and_action_server(self) -> None:
        bridge = RosBridge()
        status = SimpleNamespace(enabled=True, state_machine="READY")
        bridge._node = SimpleNamespace(
            arm_status=status,
            action_server_ready=lambda name: name == "move_to_pose",
        )

        bridge._arm_alive_state = False
        ok, reason = bridge.motion_readiness()
        self.assertFalse(ok)
        self.assertIn("heartbeat", reason)

        bridge._arm_alive_state = True
        ok, reason = bridge.motion_readiness()
        self.assertTrue(ok, reason)

        ok, reason = bridge.motion_readiness("follow_joint_trajectory")
        self.assertFalse(ok)
        self.assertIn("indisponível", reason)

    def test_cancel_active_actions_covers_pending_and_accepted_goals(self) -> None:
        bridge = RosBridge()
        handle = _FakeGoalHandle()
        bridge._action_requests = {
            1: {
                "label": "move_to_pose",
                "tag": None,
                "handle": handle,
                "accept_timer": None,
                "result_timer": None,
            },
            2: {
                "label": "follow_joint_trajectory",
                "tag": None,
                "handle": None,
                "accept_timer": None,
                "result_timer": None,
            },
            3: {
                "label": "gripper_command",
                "tag": None,
                "handle": None,
                "accept_timer": None,
                "result_timer": None,
            },
        }

        count = bridge.cancel_active_actions()

        self.assertEqual(count, 3)
        self.assertEqual(handle.cancel_calls, 1)
        self.assertEqual(bridge._action_requests, {})

    def test_gripper_result_does_not_default_to_success(self) -> None:
        wrapped = SimpleNamespace(
            status=GoalStatus.STATUS_SUCCEEDED,
            result=SimpleNamespace(reached_goal=False, stalled=True),
        )
        success, message, _pose = RosBridge._parse_action_result(
            "gripper_command", wrapped
        )
        self.assertFalse(success)
        self.assertIn("bloqueado", message)

    def test_acceptance_timeout_emits_failure_and_forgets_request(self) -> None:
        bridge = RosBridge()
        results = []
        bridge.action_result.connect(lambda name, ok, msg: results.append((name, ok, msg)))
        bridge._action_requests = {
            1: {
                "label": "move_to_pose",
                "tag": None,
                "handle": None,
                "accept_timer": None,
                "result_timer": None,
            }
        }

        bridge._on_action_accept_timeout(1)

        self.assertEqual(bridge._action_requests, {})
        self.assertEqual(results[0][0:2], ("move_to_pose", False))
        self.assertIn("aceitação", results[0][2])

    def test_result_timeout_requests_cancel_and_emits_failure(self) -> None:
        bridge = RosBridge()
        results = []
        handle = _FakeGoalHandle()
        bridge.action_result.connect(lambda name, ok, msg: results.append((name, ok, msg)))
        bridge._action_requests = {
            1: {
                "label": "follow_joint_trajectory",
                "tag": None,
                "handle": handle,
                "accept_timer": None,
                "result_timer": None,
            }
        }

        bridge._on_action_result_timeout(1)

        self.assertEqual(handle.cancel_calls, 1)
        self.assertEqual(results[0][0:2], ("follow_joint_trajectory", False))
        self.assertIn("resultado", results[0][2])

    def test_non_success_goal_status_is_reported_as_failure(self) -> None:
        wrapped = SimpleNamespace(
            status=GoalStatus.STATUS_ABORTED,
            result=SimpleNamespace(success=True, message=""),
        )
        success, message, _pose = RosBridge._parse_action_result(
            "move_to_pose", wrapped
        )
        self.assertFalse(success)
        self.assertIn("abortado", message)


if __name__ == "__main__":
    unittest.main()
