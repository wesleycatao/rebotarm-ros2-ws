"""Plot em tempo real das juntas publicadas pelo mock.

Uso:
    ros2 run rebotarm_gui mock_arm
    python3 scripts/plot_mock_joints.py
"""

from __future__ import annotations

import argparse
import sys
import threading
from collections import deque
from pathlib import Path

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

try:
    import matplotlib
    matplotlib.use("Qt5Agg")
    import matplotlib.pyplot as plt
    from matplotlib.animation import FuncAnimation
except ImportError as exc:
    raise SystemExit("Instale matplotlib: pip install matplotlib") from exc


class JointPlotNode(Node):
    def __init__(self, max_points: int = 200):
        super().__init__("plot_mock_joints")
        self._sub = None
        self._lock = threading.Lock()
        self._names: list[str] = []
        self._history: dict[str, deque[float]] = {}
        self._times: deque[float] = deque(maxlen=max_points)
        self._start = self.get_clock().now().nanoseconds / 1e9

    def _cb(self, msg: JointState) -> None:
        now = self.get_clock().now().nanoseconds / 1e9 - self._start
        with self._lock:
            if not self._names:
                self._names = [n for n in msg.name if n != "gripper"]
                for n in self._names:
                    self._history[n] = deque(maxlen=self._times.maxlen)
            if len(self._times) == 0 or now - self._times[-1] > 0.05:
                self._times.append(now)
                for n in self._names:
                    idx = msg.name.index(n) if n in msg.name else -1
                    self._history[n].append(float(msg.position[idx]) if idx >= 0 else 0.0)

    def snapshot(self) -> tuple[list[float], dict[str, list[float]], list[str]]:
        with self._lock:
            return list(self._times), {n: list(self._history[n]) for n in self._names}, list(self._names)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Plota em tempo real as juntas do mock.")
    parser.add_argument("--topic", default="/rebotarm/joint_states", help="Tópico de JointState")
    parser.add_argument("--window", type=int, default=10, help="Janela de tempo em segundos")
    args = parser.parse_args(argv)

    rclpy.init(args=argv)
    node = JointPlotNode()
    node._sub = node.create_subscription(JointState, args.topic, node._cb, 10)

    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    fig, ax = plt.subplots(figsize=(10, 6))
    lines: dict[str, plt.Line2D] = {}

    def init():
        ax.set_xlim(0, 10)
        ax.set_ylim(-1.0, 1.0)
        ax.set_xlabel("Tempo (s)")
        ax.set_ylabel("Posição (rad)")
        ax.set_title("Juntas do mock /rebotarm/joint_states")
        ax.legend(loc="upper left")
        ax.grid(True)
        return []

    def update(_frame):
        times, history, names = node.snapshot()
        if not names:
            return []
        for n in names:
            if n not in lines:
                (line,) = ax.plot([], [], label=n, lw=1.5)
                lines[n] = line
            line = lines[n]
            line.set_data(times, history[n])
        if times:
            ax.set_xlim(max(0, times[-1] - 10), max(10, times[-1] + 1))
            all_vals = [v for hist in history.values() for v in hist]
            margin = 0.1
            ax.set_ylim(min(all_vals) - margin, max(all_vals) + margin)
        ax.legend(loc="upper left")
        return list(lines.values())

    ani = FuncAnimation(fig, update, init_func=init, blit=False, interval=100, cache_frame_data=False)
    plt.tight_layout()
    plt.show()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
