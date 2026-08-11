"""Path-preserving dynamic trajectory speed override.

The override advances one common virtual clock for the complete joint path.
No joint is clipped or scaled independently.  Changes are ramped and the ramp
rate is reduced, when necessary, so the additional ``q_dot * scale_dot`` term
stays inside the configured joint-acceleration ceiling.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
import threading
from typing import Sequence

import numpy as np


@dataclass(frozen=True)
class SpeedOverrideSnapshot:
    requested: float
    applied: float
    ramping: bool
    minimum: float
    maximum: float
    increase_rate_per_s: float
    decrease_rate_per_s: float
    last_error: str


class DynamicSpeedOverride:
    """Thread-safe, acceleration-aware common-clock governor."""

    def __init__(
        self,
        max_accelerations: Sequence[float],
        *,
        safety_factor: float,
        minimum: float = 0.05,
        maximum: float = 1.0,
        increase_rate_per_s: float = 0.50,
        decrease_rate_per_s: float = 2.00,
    ) -> None:
        accelerations = np.asarray(max_accelerations, dtype=np.float64).reshape(-1)
        if accelerations.size == 0 or not np.all(np.isfinite(accelerations)):
            raise ValueError("max_accelerations must be finite and non-empty")
        if np.any(accelerations <= 0.0):
            raise ValueError("max_accelerations must be positive")
        if not math.isfinite(safety_factor) or not 0.0 < safety_factor <= 1.0:
            raise ValueError("safety_factor must be in (0, 1]")
        if not (math.isfinite(minimum) and math.isfinite(maximum)):
            raise ValueError("override bounds must be finite")
        if not 0.0 < minimum <= maximum <= 1.0:
            raise ValueError("override bounds must satisfy 0 < min <= max <= 1")
        if (
            not math.isfinite(increase_rate_per_s)
            or not math.isfinite(decrease_rate_per_s)
            or increase_rate_per_s <= 0.0
            or decrease_rate_per_s <= 0.0
        ):
            raise ValueError("override ramp rates must be finite and positive")

        self._lock = threading.RLock()
        self._effective_accelerations = accelerations * float(safety_factor)
        self.minimum = float(minimum)
        self.maximum = float(maximum)
        self.increase_rate_per_s = float(increase_rate_per_s)
        self.decrease_rate_per_s = float(decrease_rate_per_s)
        self._requested = self.maximum
        self._applied = self.maximum
        self._last_error = ""

    @property
    def requested(self) -> float:
        with self._lock:
            return self._requested

    @property
    def applied(self) -> float:
        with self._lock:
            return self._applied

    def request(self, value: object) -> tuple[bool, float, str]:
        """Store a target. Invalid input fails slow to the minimum override."""

        try:
            parsed = float(value)
        except (TypeError, ValueError):
            parsed = math.nan
        with self._lock:
            if not math.isfinite(parsed):
                self._requested = self.minimum
                self._last_error = "override não finito; aplicado mínimo seguro"
                return False, self._requested, self._last_error
            clamped = max(self.minimum, min(self.maximum, parsed))
            self._requested = clamped
            self._last_error = (
                ""
                if abs(clamped - parsed) <= 1e-12
                else "override fora da faixa; valor limitado"
            )
            return True, self._requested, self._last_error

    def settle_while_idle(self) -> float:
        """Apply the requested value immediately while no trajectory is active."""

        with self._lock:
            self._applied = self._requested
            return self._applied

    def advance(
        self,
        elapsed_s: float,
        *,
        path_velocities: Sequence[float] | np.ndarray | None = None,
        path_accelerations: Sequence[float] | np.ndarray | None = None,
        tracking_scale: float = 1.0,
    ) -> float:
        """Advance the ramp and return the common external clock scale.

        ``path_velocities`` and ``path_accelerations`` are derivatives with
        respect to the already-retimed trajectory clock.  The admissible ramp
        is the intersection of every joint's acceleration interval:

        ``|q'' * (override*tracking)^2 + q' * tracking * override'| <= a_max``.
        """

        try:
            elapsed = float(elapsed_s)
            tracking = float(tracking_scale)
        except (TypeError, ValueError):
            elapsed = 0.0
            tracking = 0.0
        if not math.isfinite(elapsed) or elapsed <= 0.0:
            return self.applied
        if not math.isfinite(tracking):
            tracking = 0.0
        tracking = max(0.0, min(1.0, tracking))

        with self._lock:
            delta = self._requested - self._applied
            if abs(delta) <= 1e-9:
                self._applied = self._requested
                return self._applied
            direction = 1.0 if delta > 0.0 else -1.0
            configured_rate = (
                self.increase_rate_per_s
                if direction > 0.0
                else self.decrease_rate_per_s
            )
            safe_rate = self._safe_ramp_rate(
                configured_rate,
                direction,
                tracking,
                path_velocities,
                path_accelerations,
            )
            step = min(abs(delta), safe_rate * min(elapsed, 0.1))
            self._applied = max(
                self.minimum,
                min(self.maximum, self._applied + direction * step),
            )
            return self._applied

    def _safe_ramp_rate(
        self,
        configured_rate: float,
        direction: float,
        tracking: float,
        velocities: Sequence[float] | np.ndarray | None,
        accelerations: Sequence[float] | np.ndarray | None,
    ) -> float:
        if velocities is None or accelerations is None:
            return configured_rate
        qd = np.asarray(velocities, dtype=np.float64).reshape(-1)
        qdd = np.asarray(accelerations, dtype=np.float64).reshape(-1)
        if (
            qd.shape != self._effective_accelerations.shape
            or qdd.shape != self._effective_accelerations.shape
            or not np.all(np.isfinite(qd))
            or not np.all(np.isfinite(qdd))
        ):
            self._last_error = (
                "derivadas inválidas no override; rampa temporariamente congelada"
            )
            return 0.0

        scale = self._applied * tracking
        base = qdd * scale * scale
        coefficient = qd * tracking * direction
        upper_rate = float(configured_rate)
        tolerance = 1e-12
        for index, limit in enumerate(self._effective_accelerations):
            if abs(base[index]) > limit + 1e-8:
                self._last_error = (
                    "aceleração base excedeu o limite; rampa temporariamente congelada"
                )
                return 0.0
            coeff = float(coefficient[index])
            if abs(coeff) <= tolerance:
                continue
            lower = (-float(limit) - float(base[index])) / coeff
            upper = (float(limit) - float(base[index])) / coeff
            if lower > upper:
                lower, upper = upper, lower
            # r=0 is feasible because the retimed base path is already valid.
            upper_rate = min(upper_rate, max(0.0, upper))
        return max(0.0, upper_rate)

    def snapshot(self) -> SpeedOverrideSnapshot:
        with self._lock:
            return SpeedOverrideSnapshot(
                requested=self._requested,
                applied=self._applied,
                ramping=abs(self._requested - self._applied) > 1e-6,
                minimum=self.minimum,
                maximum=self.maximum,
                increase_rate_per_s=self.increase_rate_per_s,
                decrease_rate_per_s=self.decrease_rate_per_s,
                last_error=self._last_error,
            )
