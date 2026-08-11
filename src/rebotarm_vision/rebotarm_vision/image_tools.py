from __future__ import annotations

from typing import Optional

import numpy as np
from sensor_msgs.msg import Image
from std_msgs.msg import Header


def image_msg_to_bgr(message: Image) -> np.ndarray:
    channels = 1 if message.encoding in ("mono8", "8UC1") else 3
    dtype = np.uint8
    frame = np.frombuffer(message.data, dtype=dtype)
    if channels == 1:
        frame = frame.reshape((message.height, message.step))[:, : message.width]
        return np.repeat(frame[:, :, None], 3, axis=2)

    frame = frame.reshape((message.height, message.step // channels, channels))
    frame = frame[:, : message.width, :]
    if message.encoding == "rgb8":
        return frame[:, :, ::-1].copy()
    return frame.copy()


def bgr_to_image_msg(frame: np.ndarray, header: Optional[Header] = None) -> Image:
    message = Image()
    if header is not None:
        message.header = header
    message.height = int(frame.shape[0])
    message.width = int(frame.shape[1])
    message.encoding = "bgr8"
    message.is_bigendian = False
    message.step = int(frame.shape[1] * 3)
    message.data = frame.astype(np.uint8, copy=False).tobytes()
    return message
