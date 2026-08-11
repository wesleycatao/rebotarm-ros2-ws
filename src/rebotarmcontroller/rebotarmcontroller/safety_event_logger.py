"""Buffer and persist the last torque safety event for GUI replay."""

from __future__ import annotations

from collections import deque
from datetime import datetime, timezone
import json
from pathlib import Path
from typing import Any

import numpy as np


DEFAULT_EVENT_PATH = Path.home() / ".config" / "rebotarmcontroller" / "last_safety_event.json"
MAX_SAMPLES = 200


class SafetyEventLogger:
    """Keeps a rolling buffer of safety samples and writes the last FAIL event."""

    def __init__(self, path: Path | str | None = None) -> None:
        self._path = Path(path) if path else DEFAULT_EVENT_PATH
        self._path.parent.mkdir(parents=True, exist_ok=True)
        self._buffer: deque[dict[str, Any]] = deque(maxlen=MAX_SAMPLES)
        self._last_saved_event: dict[str, Any] | None = None

    def record_sample(
        self,
        *,
        phase: float,
        step_id: str,
        joint_names: list[str],
        positions: list[float] | np.ndarray,
        velocities: list[float] | np.ndarray,
        torques: list[float] | np.ndarray,
    ) -> None:
        """Append one sample to the rolling buffer."""
        self._buffer.append(
            {
                "timestamp": datetime.now(timezone.utc).isoformat(),
                "phase": float(phase),
                "step_id": str(step_id),
                "joint_names": list(joint_names),
                "positions": [float(v) for v in positions],
                "velocities": [float(v) for v in velocities],
                "torques": [float(v) for v in torques],
            }
        )

    def save_event(self, severity: str, message: str, phase: float, step_id: str) -> None:
        """Write the current buffer plus event metadata to disk."""
        event = {
            "saved_at": datetime.now(timezone.utc).isoformat(),
            "severity": str(severity),
            "message": str(message),
            "phase": float(phase),
            "step_id": str(step_id),
            "samples": list(self._buffer),
        }
        try:
            with open(self._path, "w", encoding="utf-8") as f:
                json.dump(event, f, indent=2, ensure_ascii=False)
            self._last_saved_event = event
        except Exception:
            pass

    def load_event(self) -> dict[str, Any] | None:
        """Load the last saved event from disk."""
        try:
            with open(self._path, "r", encoding="utf-8") as f:
                return json.load(f)
        except Exception:
            return None

    def clear(self) -> None:
        """Clear the in-memory buffer and the on-disk event file."""
        self._buffer.clear()
        try:
            self._path.unlink(missing_ok=True)
        except Exception:
            pass

    def reset_buffer(self) -> None:
        """Start a new capture window without deleting the last fault record."""
        self._buffer.clear()
