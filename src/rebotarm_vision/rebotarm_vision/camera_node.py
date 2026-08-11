from __future__ import annotations

import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Header

from .image_tools import bgr_to_image_msg


class WebcamNode(Node):
    def __init__(self) -> None:
        super().__init__("rebotarm_webcam")
        self.declare_parameter("camera_index", 0)
        self.declare_parameter("frame_id", "camera_link")
        self.declare_parameter("width", 640)
        self.declare_parameter("height", 480)
        self.declare_parameter("fps", 15.0)
        self.declare_parameter("camera_matrix", [0.0] * 9)
        self.declare_parameter("distortion_coefficients", [0.0] * 5)
        self.declare_parameter("distortion_model", "plumb_bob")

        self._frame_id = str(self.get_parameter("frame_id").value)
        camera_index = int(self.get_parameter("camera_index").value)
        width = int(self.get_parameter("width").value)
        height = int(self.get_parameter("height").value)
        fps = float(self.get_parameter("fps").value)

        self._capture = cv2.VideoCapture(camera_index, cv2.CAP_V4L2)
        if not self._capture.isOpened():
            self.get_logger().warning(
                f"could not open /dev/video{camera_index}; retrying with default backend"
            )
            self._capture = cv2.VideoCapture(camera_index)

        self._capture.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self._capture.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        self._capture.set(cv2.CAP_PROP_FPS, fps)

        self._image_pub = self.create_publisher(Image, "/camera/image_raw", 10)
        self._info_pub = self.create_publisher(CameraInfo, "/camera/camera_info", 10)
        period = 1.0 / max(fps, 1.0)
        self._timer = self.create_timer(period, self._tick)
        self.get_logger().info(
            f"webcam publishing /camera/image_raw at {width}x{height} {fps:.1f} FPS"
        )

    def destroy_node(self) -> bool:
        if hasattr(self, "_capture") and self._capture is not None:
            self._capture.release()
        return super().destroy_node()

    def _camera_info(self, stamp, width: int, height: int) -> CameraInfo:
        info = CameraInfo()
        info.header.stamp = stamp
        info.header.frame_id = self._frame_id
        info.width = width
        info.height = height
        info.distortion_model = str(self.get_parameter("distortion_model").value)
        info.d = [float(value) for value in self.get_parameter("distortion_coefficients").value]
        k = [float(value) for value in self.get_parameter("camera_matrix").value]
        if len(k) == 9 and any(abs(value) > 1e-12 for value in k):
            info.k = k
            info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
            info.p = [k[0], k[1], k[2], 0.0, k[3], k[4], k[5], 0.0, k[6], k[7], k[8], 0.0]
        return info

    def _tick(self) -> None:
        ok, frame = self._capture.read()
        if not ok or frame is None:
            self.get_logger().warning("failed to read webcam frame", throttle_duration_sec=2.0)
            return

        stamp = self.get_clock().now().to_msg()
        header = Header()
        header.stamp = stamp
        header.frame_id = self._frame_id
        self._image_pub.publish(bgr_to_image_msg(frame, header))
        self._info_pub.publish(self._camera_info(stamp, frame.shape[1], frame.shape[0]))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = WebcamNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
