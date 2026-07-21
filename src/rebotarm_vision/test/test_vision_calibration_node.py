"""Regression tests for AI calibration scoring and local validation metrics."""

from __future__ import annotations

import unittest

from geometry_msgs.msg import Point
from rebotarm_msgs.msg import SegmentedObject, SegmentedObjectArray

from rebotarm_vision.vision_calibration_node import (
    VisionCalibrationNode,
    profile_quality_key,
)
from rebotarm_vision.vision_profile import (
    MergeSettings,
    ProfileMetrics,
    ReferenceObject,
    RotatedBBox,
    Scene,
    VisionProfile,
)


class _MetricHarness:
    _approx_rotated_iou = VisionCalibrationNode._approx_rotated_iou
    _object_iou = staticmethod(VisionCalibrationNode._object_iou)


def _points(cx: float, cy: float, width: float, height: float) -> list[Point]:
    return [
        Point(x=cx - width / 2, y=cy - height / 2),
        Point(x=cx + width / 2, y=cy - height / 2),
        Point(x=cx + width / 2, y=cy + height / 2),
        Point(x=cx - width / 2, y=cy + height / 2),
    ]


def _object(
    cx: float,
    cy: float,
    *,
    width: float = 80.0,
    height: float = 30.0,
    orientation: str = "horizontal",
) -> SegmentedObject:
    obj = SegmentedObject()
    obj.center_pixel = Point(x=cx, y=cy)
    obj.bounding_box_pixels = _points(cx, cy, width, height)
    obj.orientation = orientation
    obj.pickable = True
    return obj


def _reference(
    cx: float,
    cy: float,
    *,
    orientation: str = "horizontal",
) -> ReferenceObject:
    return ReferenceObject(
        bbox_rotated=RotatedBBox(
            cx=cx / 1000.0,
            cy=cy / 500.0,
            w=80.0 / 1000.0,
            h=30.0 / 500.0,
        ),
        orientation=orientation,
    )


def _metrics(profile: VisionProfile, objects: list[SegmentedObject]) -> ProfileMetrics:
    message = SegmentedObjectArray(objects=objects)
    return VisionCalibrationNode._compute_metrics(_MetricHarness(), profile, message)


class TestProfileQualityKey(unittest.TestCase):
    def test_fallback_penalizes_duplicates_not_only_count(self):
        duplicate_profile = VisionProfile(
            scene=Scene(expected_objects=25),
            metrics=ProfileMetrics(count_error=0, duplicate_count=2),
        )
        clean_profile = VisionProfile(
            scene=Scene(expected_objects=25),
            metrics=ProfileMetrics(count_error=1, duplicate_count=0),
        )
        self.assertLess(profile_quality_key(clean_profile), profile_quality_key(duplicate_profile))


class TestComputeMetrics(unittest.TestCase):
    def test_partial_reference_set_uses_reference_count_for_coverage(self):
        profile = VisionProfile(
            image_width=1000,
            image_height=500,
            scene=Scene(expected_objects=25),
            reference_objects=[_reference(200, 100), _reference(700, 350)],
        )
        metrics = _metrics(profile, [_object(200, 100), _object(700, 350)])
        self.assertEqual(metrics.coverage, 1.0)
        self.assertEqual(metrics.mean_iou, 1.0)

    def test_one_detection_cannot_cover_two_references(self):
        profile = VisionProfile(
            image_width=1000,
            image_height=500,
            scene=Scene(expected_objects=1),
            reference_objects=[_reference(200, 100), _reference(205, 100)],
        )
        metrics = _metrics(profile, [_object(200, 100)])
        self.assertEqual(metrics.coverage, 0.5)

    def test_close_non_overlapping_objects_are_not_duplicates(self):
        profile = VisionProfile(
            image_width=1000,
            image_height=500,
            scene=Scene(expected_objects=2),
            merge=MergeSettings(
                center_distance_threshold=0.10,
                rotated_iou_threshold=0.45,
            ),
        )
        metrics = _metrics(
            profile,
            [_object(100, 100, width=30), _object(150, 100, width=30)],
        )
        self.assertEqual(metrics.duplicate_count, 0)

    def test_overlapping_objects_are_duplicates(self):
        profile = VisionProfile(
            image_width=1000,
            image_height=500,
            scene=Scene(expected_objects=2),
            merge=MergeSettings(
                center_distance_threshold=0.10,
                rotated_iou_threshold=0.45,
            ),
        )
        metrics = _metrics(profile, [_object(100, 100), _object(105, 100)])
        self.assertEqual(metrics.duplicate_count, 1)

    def test_ambiguous_reference_does_not_reduce_orientation_accuracy(self):
        profile = VisionProfile(
            image_width=1000,
            image_height=500,
            scene=Scene(expected_objects=1),
            reference_objects=[_reference(200, 100, orientation="ambiguous")],
        )
        metrics = _metrics(profile, [_object(200, 100, orientation="vertical")])
        self.assertEqual(metrics.orientation_accuracy, 1.0)

    def test_ambiguous_non_pickable_object_is_a_grasp_violation(self):
        profile = VisionProfile(
            image_width=1000,
            image_height=500,
            scene=Scene(expected_objects=1),
        )
        obj = _object(200, 100, orientation="ambiguous")
        obj.pickable = False
        metrics = _metrics(profile, [obj])
        self.assertEqual(metrics.grasp_zone_violation_count, 1)


if __name__ == "__main__":
    unittest.main()
