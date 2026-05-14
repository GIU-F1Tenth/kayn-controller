from dataclasses import dataclass
from collections import deque
from typing import Deque, Optional, Tuple

from ..logger_utils import KAYNLogger, LogLevel


@dataclass
class WobbleParams:
    """Parameters for wobble detection and mitigation.

    Values should come from ROS parameters / config.
    """
    max_lateral_accel: Optional[float] = None
    max_angular_velocity: Optional[float] = None
    steering_oscillation_threshold: Optional[float] = None
    wobble_time_threshold: Optional[float] = None
    history_size: Optional[int] = None
    min_samples: Optional[int] = None
    wobble_decel: Optional[float] = None
    wobble_steer_scale: Optional[float] = None

    def __post_init__(self) -> None:
        missing = [
            name for name, value in (
                ("max_lateral_accel", self.max_lateral_accel),
                ("max_angular_velocity", self.max_angular_velocity),
                ("steering_oscillation_threshold", self.steering_oscillation_threshold),
                ("wobble_time_threshold", self.wobble_time_threshold),
                ("history_size", self.history_size),
                ("min_samples", self.min_samples),
                ("wobble_decel", self.wobble_decel),
                ("wobble_steer_scale", self.wobble_steer_scale),
            )
            if value is None
        ]
        if missing:
            raise ValueError(f"WobbleParams missing values: {', '.join(missing)}")


class WobbleDetector:
    """Detect oscillatory steering/yaw behavior and apply gentle mitigation."""

    def __init__(self, params: WobbleParams, logger: Optional[KAYNLogger] = None):
        self.params = params
        self._log = logger
        self.steering_history: Deque[float] = deque(maxlen=params.history_size)
        self.angular_velocity_history: Deque[float] = deque(maxlen=params.history_size)
        self.lateral_accel_history: Deque[float] = deque(maxlen=params.history_size)
        self.timestamps: Deque[float] = deque(maxlen=params.history_size)

        self._wobbling_detected = False
        self._wobble_start_time: Optional[float] = None

    def update(self, steering_angle: float, angular_velocity: float,
               velocity: float, timestamp: float) -> None:
        """Update with the latest vehicle state sample."""
        lateral_accel = velocity * angular_velocity

        if self._log:
            self._log.debug(
                f"Wobble sample | steer={steering_angle:.3f} yaw_rate={angular_velocity:.3f} "
                f"v={velocity:.2f} lat_accel={lateral_accel:.2f}"
            )

        self.steering_history.append(steering_angle)
        self.angular_velocity_history.append(angular_velocity)
        self.lateral_accel_history.append(lateral_accel)
        self.timestamps.append(timestamp)

        self._detect_wobble()

    def _detect_wobble(self) -> None:
        if len(self.steering_history) < self.params.min_samples:
            return

        steering_oscillation = self._calc_oscillation(self.steering_history)
        max_ang_vel = max(abs(v) for v in list(self.angular_velocity_history)[-5:])
        max_lat_accel = max(abs(a) for a in list(self.lateral_accel_history)[-5:])

        excessive_steer = steering_oscillation > self.params.steering_oscillation_threshold
        excessive_yaw = max_ang_vel > self.params.max_angular_velocity
        excessive_lat = max_lat_accel > self.params.max_lateral_accel

        current_wobble = excessive_steer or excessive_yaw or excessive_lat

        if current_wobble and not self._wobbling_detected:
            self._wobble_start_time = self.timestamps[-1]
            self._wobbling_detected = True
            if self._log:
                self._log.warn(
                    "Wobble onset | "
                    f"steer_osc={steering_oscillation:.3f} "
                    f"yaw_rate={max_ang_vel:.3f} lat_accel={max_lat_accel:.3f}",
                    LogLevel.DEBUG,
                )
        elif not current_wobble:
            self._wobbling_detected = False
            self._wobble_start_time = None
            if self._log:
                self._log.debug("Wobble cleared")

    def _calc_oscillation(self, data: Deque[float]) -> float:
        if len(data) < 5:
            return 0.0

        diffs = [abs(data[i] - data[i - 1]) for i in range(1, len(data))]
        if not diffs:
            return 0.0
        return sum(diffs) / len(diffs)

    def is_wobbling(self) -> bool:
        if not self._wobbling_detected:
            return False

        if self._wobble_start_time is None or not self.timestamps:
            return False

        wobble_duration = self.timestamps[-1] - self._wobble_start_time
        return wobble_duration > self.params.wobble_time_threshold

    def apply_mitigation(self, control: Tuple[float, float]) -> Tuple[float, float]:
        """Apply wobble mitigation to [steering, accel] control."""
        steering, accel = control

        if not self.is_wobbling():
            return steering, accel

        steering *= self.params.wobble_steer_scale
        accel = min(accel, -abs(self.params.wobble_decel))

        return steering, accel

    def reset(self) -> None:
        self.steering_history.clear()
        self.angular_velocity_history.clear()
        self.lateral_accel_history.clear()
        self.timestamps.clear()
        self._wobbling_detected = False
        self._wobble_start_time = None
