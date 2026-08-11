"""Tests for cell-scene persistence, signatures and TCP clearances."""

from __future__ import annotations

import math
import sys
from pathlib import Path

WORKSPACE = Path(__file__).resolve().parents[3]
SRC_PKG = WORKSPACE / "src" / "rebotarm_gui"
if str(SRC_PKG) not in sys.path:
    sys.path.insert(0, str(SRC_PKG))

from rebotarm_gui.motion.cell_scene import (  # noqa: E402
    KEEP_OUT,
    CellObstacle,
    CellScene,
    CellSceneStore,
    MonitoredGeometry,
    analyze_program_tcp_clearance,
    analyze_tcp_clearance,
    load_cell_scene,
    monitored_scene_signature,
    monitored_world_vertices,
    save_cell_scene,
)
from rebotarm_gui.motion.program import MoveStep, Program, pose  # noqa: E402


def _scene() -> CellScene:
    return CellScene(
        minimum_clearance_m=0.05,
        tcp_radius_m=0.03,
        obstacles=(
            CellObstacle(
                object_id="fixture_1",
                name="Dispositivo",
                kind=KEEP_OUT,
                center=(0.4, 0.0, 0.2),
                size=(0.2, 0.2, 0.2),
                margin_m=0.01,
            ),
        ),
    ).validated()


def test_scene_signature_ignores_revision_metadata() -> None:
    first = _scene()
    second = CellScene(
        frame_id=first.frame_id,
        minimum_clearance_m=first.minimum_clearance_m,
        tcp_radius_m=first.tcp_radius_m,
        obstacles=first.obstacles,
        revision=99,
        updated_at="2099-01-01T00:00:00+00:00",
    ).validated()
    assert first.signature() == second.signature()


def test_scene_round_trip(tmp_path: Path) -> None:
    target = tmp_path / "cell_scene.json"
    save_cell_scene(_scene(), target)
    loaded = load_cell_scene(target)
    assert loaded.signature() == _scene().signature()
    assert loaded.obstacles[0].kind == KEEP_OUT
    assert target.stat().st_mode & 0o077 == 0


def test_crossing_segment_is_rejected() -> None:
    report = analyze_tcp_clearance(
        ((0.1, 0.0, 0.2), (0.7, 0.0, 0.2)),
        _scene(),
    )
    assert report.verified
    assert not report.safe
    assert report.minimum_clearance_m < 0.0
    assert report.limiting_obstacle == "Dispositivo"


def test_route_above_box_has_expected_positive_margin() -> None:
    report = analyze_tcp_clearance(
        ((0.1, 0.0, 0.5), (0.7, 0.0, 0.5)),
        _scene(),
    )
    assert report.safe
    # Box top including its own margin is z=.31; TCP envelope radius is .03.
    assert math.isclose(report.minimum_clearance_m, 0.16, abs_tol=0.003)


def test_program_analysis_checks_cartesian_targets() -> None:
    program = Program(
        steps=[
            MoveStep(pose=pose(0.1, 0.0, 0.2)),
            MoveStep(pose=pose(0.7, 0.0, 0.2)),
        ]
    )
    report = analyze_program_tcp_clearance(program, _scene())
    assert not report.safe
    assert report.point_count == 2
    assert report.violations[0].path_index == 0


def test_store_invalidates_moveit_ack_only_when_geometry_changes(
    tmp_path: Path,
) -> None:
    store = CellSceneStore(tmp_path / "scene.json")
    scene = _scene()
    store.replace(scene)
    store.mark_applied(scene.signature(), True, "cena reconciliada")
    assert store.is_applied
    assert store.last_apply_message == "cena reconciliada"

    same_geometry = CellScene(
        frame_id=scene.frame_id,
        minimum_clearance_m=scene.minimum_clearance_m,
        tcp_radius_m=scene.tcp_radius_m,
        obstacles=scene.obstacles,
        revision=scene.revision + 1,
    )
    store.replace(same_geometry)
    assert store.is_applied

    changed = CellScene(
        frame_id=scene.frame_id,
        minimum_clearance_m=0.08,
        tcp_radius_m=scene.tcp_radius_m,
        obstacles=scene.obstacles,
        revision=scene.revision + 2,
    )
    store.replace(changed)
    assert not store.is_applied
    assert store.last_apply_message == ""


def test_store_preserves_invalid_file_error_for_fail_closed_gate(
    tmp_path: Path,
) -> None:
    target = tmp_path / "scene.json"
    target.write_text("{invalid", encoding="utf-8")
    store = CellSceneStore(target)
    assert store.load_error
    assert not store.scene.configured


def test_monitored_geometry_is_normalized_and_transformed() -> None:
    geometry = MonitoredGeometry(
        object_id="external_fixture",
        frame_id="base_link",
        shape="box",
        pose=(1.0, 2.0, 3.0, 0.0, 0.0, 0.0, 2.0),
        dimensions=(2.0, 4.0, 6.0),
    ).validated()
    vertices = monitored_world_vertices(geometry)
    assert len(vertices) == 8
    assert {round(point[0], 6) for point in vertices} == {0.0, 2.0}
    assert {round(point[1], 6) for point in vertices} == {0.0, 4.0}
    assert {round(point[2], 6) for point in vertices} == {0.0, 6.0}


def test_monitored_plane_is_clipped_to_finite_display_quad() -> None:
    plane = MonitoredGeometry(
        object_id="floor",
        frame_id="base_link",
        shape="plane",
        pose=(0.0, 0.0, 0.2, 0.0, 0.0, 0.0, 1.0),
        dimensions=(2.0,),
        plane_coefficients=(0.0, 0.0, 2.0, 0.0),
    ).validated()

    vertices = monitored_world_vertices(plane)

    assert len(vertices) == 4
    assert all(abs(point[2] - 0.2) < 1e-9 for point in vertices)
    assert max(point[0] for point in vertices) == 1.0
    assert min(point[0] for point in vertices) == -1.0
    assert max(point[1] for point in vertices) == 1.0
    assert min(point[1] for point in vertices) == -1.0


def test_monitored_signature_uses_source_pose_not_live_tf_pose() -> None:
    first = MonitoredGeometry(
        object_id="attached_part",
        frame_id="base_link",
        shape="box",
        pose=(0.1, 0.0, 0.2, 0.0, 0.0, 0.0, 1.0),
        dimensions=(0.1, 0.1, 0.1),
        attached_to="gripper_link",
        source_frame_id="gripper_link",
        source_pose=(0.0, 0.0, 0.05, 0.0, 0.0, 0.0, 1.0),
        transform_status="resolved",
    ).validated()
    moved = MonitoredGeometry(
        object_id=first.object_id,
        frame_id=first.frame_id,
        shape=first.shape,
        pose=(0.4, 0.2, 0.3, 0.0, 0.0, 0.0, 1.0),
        dimensions=first.dimensions,
        attached_to=first.attached_to,
        source_frame_id=first.source_frame_id,
        source_pose=first.source_pose,
        transform_status="resolved",
    ).validated()

    assert monitored_scene_signature((first,)) == monitored_scene_signature(
        (moved,)
    )


def test_store_separates_gui_managed_and_external_moveit_objects(
    tmp_path: Path,
) -> None:
    store = CellSceneStore(tmp_path / "scene.json")
    payload = {
        "name": "monitored",
        "geometries": [
            {
                "object_id": "external_fixture",
                "frame_id": "base_link",
                "shape": "sphere",
                "pose": [0.3, 0.0, 0.2, 0.0, 0.0, 0.0, 1.0],
                "dimensions": [0.1],
            },
            {
                "object_id": "rebotarm_gui__fixture_1",
                "frame_id": "base_link",
                "shape": "box",
                "pose": [0.4, 0.0, 0.2, 0.0, 0.0, 0.0, 1.0],
                "dimensions": [0.2, 0.2, 0.2],
            },
        ],
    }
    store.accept_monitored_result(True, payload, "")
    assert store.monitored_ready
    assert len(store.monitored_geometries) == 2
    assert [item.object_id for item in store.external_geometries] == [
        "external_fixture"
    ]
    assert store.monitored_signature == monitored_scene_signature(
        store.monitored_geometries
    )


def test_failed_monitored_refresh_preserves_last_known_snapshot(
    tmp_path: Path,
) -> None:
    store = CellSceneStore(tmp_path / "scene.json")
    payload = {
        "geometries": [
            {
                "object_id": "wall",
                "frame_id": "base_link",
                "shape": "box",
                "pose": [0.5, 0.0, 0.5, 0.0, 0.0, 0.0, 1.0],
                "dimensions": [0.1, 1.0, 1.0],
            }
        ]
    }
    store.accept_monitored_result(True, payload)
    signature = store.monitored_signature
    store.accept_monitored_result(False, {}, "timeout")
    assert not store.monitored_ready
    assert store.monitored_error == "timeout"
    assert store.monitored_signature == signature
    assert len(store.external_geometries) == 1


def test_compact_tf_update_moves_geometry_without_rebuilding_mesh(
    tmp_path: Path,
) -> None:
    store = CellSceneStore(tmp_path / "scene.json")
    vertices = [[float(index), 0.0, 0.0] for index in range(200)]
    store.accept_monitored_result(
        True,
        {
            "tf_dynamic_refresh_enabled": True,
            "tf_refresh_rate_hz": 5.0,
            "tf_scene_generation": 7,
            "geometries": [
                {
                    "object_id": "held_mesh",
                    "frame_id": "base_link",
                    "shape": "mesh",
                    "pose": [0.1, 0.0, 0.2, 0.0, 0.0, 0.0, 1.0],
                    "vertices": vertices,
                    "triangles": [],
                    "attached_to": "gripper_link",
                    "source_frame_id": "gripper_link",
                    "source_pose": [
                        0.0,
                        0.0,
                        0.1,
                        0.0,
                        0.0,
                        0.0,
                        1.0,
                    ],
                    "transform_status": "resolved",
                }
            ],
        },
    )
    original = store.monitored_geometries[0]
    signature = store.monitored_signature

    store.accept_monitored_result(
        True,
        {
            "tf_dynamic_update": True,
            "tf_dynamic_refresh_enabled": True,
            "tf_refresh_rate_hz": 5.0,
            "tf_refresh_sequence": 1,
            "tf_scene_generation": 7,
            "transform_updates": [
                {
                    "geometry_index": 0,
                    "object_id": "held_mesh",
                    "attached_to": "gripper_link",
                    "source_frame_id": "gripper_link",
                    "frame_id": "base_link",
                    "pose": [0.4, 0.0, 0.2, 0.0, 0.0, 0.0, 1.0],
                    "transform_status": "resolved",
                }
            ],
        },
    )

    updated = store.monitored_geometries[0]
    assert updated.pose[0] == 0.4
    assert updated.vertices is original.vertices
    assert store.monitored_signature == signature
    assert store.monitored_tf_refresh_sequence == 1
    assert store.monitored_error == ""


def test_tf_update_from_old_scene_generation_is_ignored(tmp_path: Path) -> None:
    store = CellSceneStore(tmp_path / "scene.json")
    store.accept_monitored_result(
        True,
        {
            "tf_scene_generation": 3,
            "geometries": [
                {
                    "object_id": "part",
                    "frame_id": "base_link",
                    "shape": "sphere",
                    "pose": [0.1, 0.0, 0.2, 0.0, 0.0, 0.0, 1.0],
                    "dimensions": [0.1],
                }
            ],
        },
    )
    store.accept_monitored_result(
        True,
        {
            "tf_dynamic_update": True,
            "tf_scene_generation": 2,
            "transform_updates": [],
        },
    )

    assert store.monitored_geometries[0].pose[0] == 0.1
    assert store.monitored_error == ""


def test_octomap_metadata_participates_in_monitored_scene_signature(
    tmp_path: Path,
) -> None:
    store = CellSceneStore(tmp_path / "scene.json")
    base_payload = {
        "geometries": [],
        "octomap": {
            "present": True,
            "frame_id": "map",
            "pose": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0],
            "tree_id": "OcTree",
            "resolution_m": 0.05,
            "binary": True,
            "data_bytes": 3,
            "content_sha256": "a" * 64,
        },
    }
    store.accept_monitored_result(True, base_payload)
    first_signature = store.monitored_signature

    changed = dict(base_payload)
    changed["octomap"] = {
        **base_payload["octomap"],
        "content_sha256": "b" * 64,
    }
    store.accept_monitored_result(True, changed)

    assert store.monitored_octomap["present"]
    assert store.monitored_signature != first_signature
