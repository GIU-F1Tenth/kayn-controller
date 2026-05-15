#!/usr/bin/env python3
"""Pure Pursuit controller ."""

import math
from typing import Dict, List, Optional, Tuple

import numpy as np

from .bicycle_model import BicycleModel


class PurePursuitController:
    """Pure Pursuit path tracking controller.

    The controller consumes the KAYN trajectory format (list of dicts) and
    outputs a steering command for the current vehicle state.
    """

    def __init__(
        self,
        model: BicycleModel,
        min_lookahead_distance: float = 0.8,
        max_lookahead_distance: float = 2.2,
        min_velocity: float = 0.0,
        max_velocity: float = 6.0,
        vel_division_factor: float = 1.0,
        skidding_velocity_thresh: float = 0.0,
        kp: float = 1.0,
        kd: float = 0.1,
        k_sigmoid: float = 8.0,
        use_lateral_error_gamma_compensation: bool = True,
        lateral_error_compensation_gain: float = 0.1,
        use_lateral_error_speed_reducer: bool = True,
        lateral_error_speed_reducer_gain: float = 0.1,
        laser_base_link_length: float = 0.27,
        enable_speed_capping: bool = False,
    ):
        self.model = model
        self.min_lad = float(min_lookahead_distance)
        self.max_lad = float(max_lookahead_distance)
        self.min_velocity = float(min_velocity)
        self.max_velocity = float(max_velocity)
        self.vel_division_factor = float(vel_division_factor)
        self.skidding_velocity_thresh = float(skidding_velocity_thresh)
        self.kp = float(kp)
        self.kd = float(kd)
        self.k_sigmoid = float(k_sigmoid)
        self.use_lateral_error_gamma_compensation = bool(
            use_lateral_error_gamma_compensation
        )
        self.lateral_error_compensation_gain = float(lateral_error_compensation_gain)
        self.use_lateral_error_speed_reducer = bool(use_lateral_error_speed_reducer)
        self.lateral_error_speed_reducer_gain = float(
            lateral_error_speed_reducer_gain
        )
        self.laser_base_link_length = float(laser_base_link_length)
        self.enable_speed_capping = bool(enable_speed_capping)
        self.prev_gamma = 0.0
        self.target_velocity = -1.0
        self.last_speed_cmd: Optional[float] = None

    def set_speed_cap(self, cap: float) -> None:
        self.target_velocity = float(cap)

    def compute_control(
        self, x_curr: np.ndarray, trajectory: List[Dict], ref_idx: int
    ) -> float:
        if not trajectory:
            self.last_speed_cmd = 0.0
            return 0.0

        x = float(x_curr[0])
        y = float(x_curr[1])
        yaw = float(x_curr[2])
        v = float(x_curr[3])

        lad = self._lookahead_distance(v)
        closest_idx = self._closest_point_idx(trajectory, x, y)
        laser_x = x + self.laser_base_link_length * math.cos(yaw)
        laser_y = y + self.laser_base_link_length * math.sin(yaw)
        closest_laser_idx = self._closest_point_idx(trajectory, laser_x, laser_y)
        lookahead = self._find_lookahead_point(trajectory, closest_idx, x, y, lad)
        if lookahead is None:
            lookahead = trajectory[min(closest_idx, len(trajectory) - 1)]

        _, ly = self._transform_to_vehicle_frame(lookahead, x, y, yaw)
        laser_point = trajectory[min(closest_laser_idx, len(trajectory) - 1)]
        _, laser_ly = self._transform_to_vehicle_frame(laser_point, x, y, yaw)

        if lad <= 1e-6:
            gamma = 0.0
        else:
            gamma = 2.0 * ly / (lad * lad)

        d_controller = (gamma - self.prev_gamma) * self.kd
        p_controller = self.kp * gamma
        if self.use_lateral_error_gamma_compensation:
            p_controller += laser_ly * self.lateral_error_compensation_gain
        self.prev_gamma = gamma

        delta = p_controller + d_controller
        self.last_speed_cmd = self._compute_speed(
            gamma, laser_ly, v, trajectory, closest_idx
        )
        return float(np.clip(delta, -self.model.delta_max, self.model.delta_max))

    def _lookahead_distance(self, v: float) -> float:
        if self.max_velocity <= self.min_velocity:
            return self.max_lad
        m = (self.max_lad - self.min_lad) / (self.max_velocity - self.min_velocity)
        c = self.max_lad - m * self.max_velocity
        lad = m * v + c
        return max(self.min_lad, min(self.max_lad, lad))

    def _find_lookahead_point(
        self,
        trajectory: List[Dict],
        start_idx: int,
        x: float,
        y: float,
        lad: float,
    ) -> Optional[Dict]:
        if not trajectory:
            return None
        n = len(trajectory)

        for i in range(start_idx, n):
            dx = trajectory[i]["x"] - x
            dy = trajectory[i]["y"] - y
            if math.sqrt(dx * dx + dy * dy) >= lad:
                return trajectory[i]

        for i in range(0, start_idx):
            dx = trajectory[i]["x"] - x
            dy = trajectory[i]["y"] - y
            if math.sqrt(dx * dx + dy * dy) >= lad:
                return trajectory[i]

        return None

    def _closest_point_idx(
        self, trajectory: List[Dict], x: float, y: float
    ) -> int:
        pts = np.array([[wp["x"], wp["y"]] for wp in trajectory])
        dists = np.linalg.norm(pts - np.array([x, y]), axis=1)
        return int(np.argmin(dists))

    def _transform_to_vehicle_frame(
        self, point: Dict, x: float, y: float, yaw: float
    ) -> Tuple[float, float]:
        dx = point["x"] - x
        dy = point["y"] - y
        transformed_x = math.cos(-yaw) * dx - math.sin(-yaw) * dy
        transformed_y = math.sin(-yaw) * dx + math.cos(-yaw) * dy
        return transformed_x, transformed_y

    def _compute_speed(
        self,
        gamma: float,
        laser_ly: float,
        curr_vel: float,
        trajectory: List[Dict],
        closest_idx: int,
    ) -> float:
        speed = float(trajectory[closest_idx].get("v", 0.0))
        if speed <= 0.0:
            speed = self._velocity_sigmoid(gamma)

        if self.use_lateral_error_speed_reducer:
            speed = max(
                self.min_velocity,
                speed - abs(laser_ly) * self.lateral_error_speed_reducer_gain,
            )

        if self.vel_division_factor > 0.0:
            speed = speed / self.vel_division_factor

        if self.enable_speed_capping and self.target_velocity >= 0.0:
            speed = min(speed, self.target_velocity)

        speed = self._smooth_vel(curr_vel, speed)
        return max(self.min_velocity, min(self.max_velocity, speed))

    def _smooth_vel(self, curr_vel: float, target_vel: float) -> float:
        if self.skidding_velocity_thresh <= 0.0:
            return target_vel
        if (target_vel - curr_vel) > self.skidding_velocity_thresh:
            return curr_vel + self.skidding_velocity_thresh
        return target_vel

    def _compute_c(self, v_min: float, v_max: float, k: float) -> float:
        return -1.0 * (1.0 / k) * np.log((v_max - v_max * 0.999) / (v_max * 0.999 - v_min))

    def _velocity_sigmoid(self, gamma: float) -> float:
        k = self.k_sigmoid
        if k <= 0.0:
            return self.max_velocity
        c = self._compute_c(v_min=self.min_velocity, v_max=self.max_velocity, k=k)
        vel = self.min_velocity + (
            (self.max_velocity - self.min_velocity) / (1.0 + np.exp(k * (abs(gamma) - c)))
        )
        return max(self.min_velocity, min(self.max_velocity, vel))
