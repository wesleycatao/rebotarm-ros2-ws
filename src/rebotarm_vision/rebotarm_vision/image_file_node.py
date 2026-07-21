from __future__ import annotations

from pathlib import Path

import cv2
import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Header

from .image_tools import bgr_to_image_msg


DEFAULT_TEST_IMAGE = "default_vision_test.png"
FALLBACK_TEST_IMAGE = "garrafas.png"


class ImageFileNode(Node):
    def __init__(self) -> None:
        super().__init__("rebotarm_image_file")
        self.declare_parameter("image_path", "")
        self.declare_parameter("image_topic", "/camera/image_raw")
        self.declare_parameter("frame_id", "camera_link")
        self.declare_parameter("publish_rate", 1.0)
        self.declare_parameter("resize_width", 0)
        self.declare_parameter("resize_height", 0)

        self._image_path = str(self.get_parameter("image_path").value).strip()
        image_topic = str(self.get_parameter("image_topic").value)
        rate = max(0.1, float(self.get_parameter("publish_rate").value))

        self._frame = None
        image_path = self._resolve_image_path(self._image_path)
        if image_path:
            self._image_path = str(image_path)
            self._frame = cv2.imread(str(image_path), cv2.IMREAD_COLOR)

        if self._frame is None:
            self.get_logger().error(
                f"could not load image_path='{self._image_path}'. "
                "Set static_image_path:=/path/to/image.png when launching, "
                f"or add {DEFAULT_TEST_IMAGE} to test_images."
            )
        else:
            resize_width = int(self.get_parameter("resize_width").value)
            resize_height = int(self.get_parameter("resize_height").value)
            if resize_width > 0 and resize_height > 0:
                self._frame = cv2.resize(
                    self._frame,
                    (resize_width, resize_height),
                    interpolation=cv2.INTER_AREA,
                )
            height, width = self._frame.shape[:2]
            self.get_logger().info(
                f"publishing static image {self._image_path} ({width}x{height}) on {image_topic}"
            )

        self._publisher = self.create_publisher(Image, image_topic, 10)
        self.create_timer(1.0 / rate, self._publish)

    def _resolve_image_path(self, requested_path: str) -> Path | None:
        if requested_path:
            path = Path(requested_path).expanduser()
            if path.is_file():
                return path
            return None

        for path in self._default_image_candidates():
            if path.is_file():
                self.get_logger().info(f"using default test image: {path}")
                return path
        return None

    @staticmethod
    def _default_image_candidates() -> list[Path]:
        candidates: list[Path] = []
        try:
            share = Path(get_package_share_directory("rebotarm_vision"))
            candidates.extend(
                [
                    share / "test_images" / DEFAULT_TEST_IMAGE,
                    share / "test_images" / FALLBACK_TEST_IMAGE,
                ]
            )
        except Exception:
            pass

        source_root = Path(__file__).resolve().parents[1]
        candidates.extend(
            [
                source_root / "test_images" / DEFAULT_TEST_IMAGE,
                source_root / "test_images" / FALLBACK_TEST_IMAGE,
                Path.cwd() / "src" / "rebotarm_vision" / "test_images" / DEFAULT_TEST_IMAGE,
                Path.cwd() / "src" / "rebotarm_vision" / "test_images" / FALLBACK_TEST_IMAGE,
            ]
        )
        return candidates

    def _publish(self) -> None:
        if self._frame is None:
            return
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = str(self.get_parameter("frame_id").value)
        self._publisher.publish(bgr_to_image_msg(self._frame, header))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ImageFileNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
