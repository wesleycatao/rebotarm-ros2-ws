"""Smoke tests for the CalibrateVision service interface."""

from __future__ import annotations

import unittest

from rebotarm_msgs.srv import CalibrateVision
from sensor_msgs.msg import Image


class TestCalibrateVisionService(unittest.TestCase):
    def test_request_has_expected_fields(self):
        req = CalibrateVision.Request()
        req.prompt = "detect bottles"
        req.api_key = "test-key"
        req.max_iterations = 5
        req.max_time_s = 120.0
        req.image = Image()
        req.image.width = 640
        req.image.height = 480
        self.assertEqual(req.prompt, "detect bottles")
        self.assertEqual(req.api_key, "test-key")

    def test_response_has_expected_fields(self):
        res = CalibrateVision.Response()
        res.success = True
        res.message = "ok"
        res.profile_json = "{}"
        res.report = "done"
        self.assertTrue(res.success)


if __name__ == "__main__":
    unittest.main()
