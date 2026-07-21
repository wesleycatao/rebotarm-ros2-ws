"""Unit tests for rebotarm_vision.geometry helpers."""

from __future__ import annotations

import math
import unittest

from rebotarm_vision.geometry import (
    local_to_image_point,
    normalize_angle,
    polygon_area,
    rotated_bbox_axes,
    shrink_polygon,
)


class TestNormalizeAngle(unittest.TestCase):
    def test_zero(self):
        self.assertAlmostEqual(normalize_angle(0.0), 0.0)

    def test_wrap_positive(self):
        self.assertAlmostEqual(normalize_angle(3.0 * math.pi), math.pi, places=5)

    def test_wrap_negative(self):
        self.assertAlmostEqual(normalize_angle(-3.0 * math.pi), math.pi, places=5)


class TestRotatedBBoxAxes(unittest.TestCase):
    def test_zero_angle(self):
        long_axis, short_axis = rotated_bbox_axes(0.0)
        self.assertAlmostEqual(long_axis[0], 1.0)
        self.assertAlmostEqual(long_axis[1], 0.0)
        self.assertAlmostEqual(short_axis[0], 0.0)
        self.assertAlmostEqual(short_axis[1], 1.0)


class TestLocalToImagePoint(unittest.TestCase):
    def test_center(self):
        pt = local_to_image_point(
            (100.0, 100.0),
            (1.0, 0.0),
            (0.0, 1.0),
            50.0,
            20.0,
            (0.0, 0.0),
        )
        self.assertAlmostEqual(pt[0], 100.0)
        self.assertAlmostEqual(pt[1], 100.0)

    def test_corner(self):
        pt = local_to_image_point(
            (100.0, 100.0),
            (1.0, 0.0),
            (0.0, 1.0),
            50.0,
            20.0,
            (1.0, 1.0),
        )
        self.assertAlmostEqual(pt[0], 150.0)
        self.assertAlmostEqual(pt[1], 120.0)


class TestPolygonArea(unittest.TestCase):
    def test_rectangle(self):
        pts = [(0, 0), (10, 0), (10, 5), (0, 5)]
        self.assertAlmostEqual(polygon_area(pts), 50.0)

    def test_triangle(self):
        pts = [(0, 0), (10, 0), (0, 5)]
        self.assertAlmostEqual(polygon_area(pts), 25.0)


class TestShrinkPolygon(unittest.TestCase):
    def test_rectangle(self):
        pts = [(-1, -1), (1, -1), (1, 1), (-1, 1)]
        shrunk = shrink_polygon(pts, 0.5, 0.5, (0, 0), (1, 0), (0, 1))
        self.assertEqual(len(shrunk), 4)
        self.assertAlmostEqual(shrunk[0][0], -0.5)
        self.assertAlmostEqual(shrunk[0][1], -0.5)


if __name__ == "__main__":
    unittest.main()
