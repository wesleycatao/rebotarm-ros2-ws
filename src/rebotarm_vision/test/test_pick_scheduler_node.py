"""Unit tests for configurable pick filtering and route ordering."""

from __future__ import annotations

from types import SimpleNamespace
import unittest

from rebotarm_vision.pick_scheduler_node import PickSchedulerNode


def _object(
    x: float,
    y: float,
    *,
    orientation: str = "horizontal",
    score: float = 0.8,
    clearances: tuple[float, float, float, float] = (20.0, 20.0, 20.0, 20.0),
):
    left, right, up, down = clearances
    return SimpleNamespace(
        orientation=orientation,
        score=score,
        pose=SimpleNamespace(position=SimpleNamespace(x=x, y=y)),
        clearance_left_px=left,
        clearance_right_px=right,
        clearance_up_px=up,
        clearance_down_px=down,
    )


class SchedulerHarness(PickSchedulerNode):
    def __init__(self, **params) -> None:
        self._params = {
            "strategy": "score",
            "allowed_orientations": ["horizontal", "vertical", "ambiguous"],
            "min_clearance_px": 0.0,
            "min_clearance_horizontal_px": 0.0,
            "min_clearance_vertical_px": 0.0,
            "route_origin_xy": [0.0, 0.0],
            **params,
        }

    def get_parameter(self, name: str):
        return SimpleNamespace(value=self._params[name])


class TestPickScheduler(unittest.TestCase):
    def test_clearance_depends_on_orientation(self) -> None:
        scheduler = SchedulerHarness(
            min_clearance_horizontal_px=10.0,
            min_clearance_vertical_px=50.0,
        )
        horizontal = _object(0, 0, clearances=(9, 30, 100, 100))
        vertical = _object(0, 0, orientation="vertical", clearances=(9, 30, 40, 60))
        self.assertFalse(scheduler._clearance_allowed(horizontal))
        self.assertTrue(scheduler._clearance_allowed(vertical))

    def test_orientation_filter(self) -> None:
        scheduler = SchedulerHarness(allowed_orientations=["vertical"])
        self.assertFalse(scheduler._orientation_allowed(_object(0, 0)))
        self.assertTrue(scheduler._orientation_allowed(_object(0, 0, orientation="vertical")))

    def test_nearest_neighbor_route_is_greedy(self) -> None:
        scheduler = SchedulerHarness(strategy="nearest_neighbor", route_origin_xy=[0.0, 0.0])
        far = _object(10.0, 0.0)
        near = _object(1.0, 0.0)
        middle = _object(3.0, 0.0)
        self.assertEqual(scheduler._sort_queue([far, near, middle]), [near, middle, far])


if __name__ == "__main__":
    unittest.main()
