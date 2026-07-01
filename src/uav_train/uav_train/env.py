from __future__ import annotations

import math
import time
from pathlib import Path
from typing import Any

import gymnasium as gym
import numpy as np
from gymnasium import spaces

from .config import ProjectConfig, load_config
from .gazebo_adapter import RosGazeboAdapter
from .observation import (
    build_feature_names,
    clean_depth,
    closest_depth_grid,
    resize_depth,
    robust_near_depth,
    wrap_to_pi,
)
from .reward import RewardTerms, compute_reward


class XaiSacGazeboEnv(gym.Env):
    """Gymnasium environment matching the AirSim XAI SAC model contract."""

    metadata = {"render_modes": ["depth_array"]}

    def __init__(
        self,
        config: ProjectConfig | str | Path | None = None,
        connect: bool = True,
    ) -> None:
        super().__init__()
        if isinstance(config, ProjectConfig):
            self.config = config
        else:
            self.config = load_config(config)

        self.env_cfg = self.config.environment
        self.action_cfg = self.config.action
        self.reward_cfg = self.config.reward

        self.depth_grid_shape = (
            self.env_cfg.depth_grid_rows,
            self.env_cfg.depth_grid_cols,
        )
        self.depth_feature_count = (
            self.env_cfg.depth_grid_rows * self.env_cfg.depth_grid_cols
        )
        self.feature_names = build_feature_names(*self.depth_grid_shape)

        yaw_rate_max = math.radians(self.action_cfg.yaw_rate_max_deg)
        self.action_space = spaces.Box(
            low=np.array(
                [
                    self.action_cfg.v_xy_min,
                    -self.action_cfg.v_z_max,
                    -yaw_rate_max,
                ],
                dtype=np.float32,
            ),
            high=np.array(
                [
                    self.action_cfg.v_xy_max,
                    self.action_cfg.v_z_max,
                    yaw_rate_max,
                ],
                dtype=np.float32,
            ),
            dtype=np.float32,
        )

        obs_low = np.array(
            [0.0] * self.depth_feature_count
            + [0.0, -1.0, -1.0, 0.0, -1.0, -1.0],
            dtype=np.float32,
        )
        obs_high = np.array(
            [1.0] * self.depth_feature_count
            + [1.0, 1.0, 1.0, 1.0, 1.0, 1.0],
            dtype=np.float32,
        )
        self.observation_space = spaces.Box(
            low=obs_low,
            high=obs_high,
            dtype=np.float32,
        )

        self.adapter: RosGazeboAdapter | None = (
            RosGazeboAdapter(self.config) if connect else None
        )
        self.start_position = np.asarray(
            self.env_cfg.start_position,
            dtype=np.float32,
        )
        self.goal_position = np.zeros(3, dtype=np.float32)
        self.previous_distance = 0.0
        self.initial_distance = 1.0
        self.step_count = 0
        self.episode_count = 0
        self.last_action = np.zeros(3, dtype=np.float32)
        self.last_yaw_rad = 0.0
        self.last_depth_m: np.ndarray | None = None
        self.last_depth_raw_m: np.ndarray | None = None
        self.last_depth_feature_m = np.zeros(
            self.depth_feature_count,
            dtype=np.float32,
        )
        self.last_observation = np.zeros(
            self.depth_feature_count + 6,
            dtype=np.float32,
        )
        self.last_state_raw = np.zeros(6, dtype=np.float32)
        self.last_state_position = self.start_position.copy()
        self.last_reward_terms: RewardTerms | None = None
        self.last_crash_signal_available = False
        self.last_crash_source: str | None = None
        self.last_crash_reason = ""
        self.goal_sample_failures = 0
        self.goal_check_depth_m: np.ndarray | None = None

    def reset(
        self,
        *,
        seed: int | None = None,
        options: dict[str, Any] | None = None,
    ) -> tuple[np.ndarray, dict[str, Any]]:
        super().reset(seed=seed)
        self._ensure_connected()
        self.adapter.wait_until_ready()

        if self.config.ros.stop_on_reset:
            self.adapter.publish_zero(duration_sec=0.2)
        if self.config.ros.reset_sim_on_reset:
            reset_ok = self._reset_sim_with_retries()
            if reset_ok:
                self.adapter.wait_until_ready()
            elif self.config.ros.reset_sim_required:
                raise RuntimeError("Simulation reset service failed.")
            else:
                self.adapter.get_logger().warn(
                    "Simulation reset failed after retries; "
                    "continuing without Gazebo reset."
                )
        if self.config.ros.auto_arm:
            self.adapter.arm_offboard()

        self.step_count = 0
        self.episode_count += 1
        self.start_position, self.last_yaw_rad = self.adapter.get_pose()
        self.last_state_position = self.start_position.copy()
        self.goal_check_depth_m = None
        self.goal_position = self._sample_goal_position()
        self.initial_distance = max(
            float(np.linalg.norm(self.start_position - self.goal_position)),
            1e-6,
        )
        self.previous_distance = self.initial_distance
        self.last_action = np.zeros(3, dtype=np.float32)
        self.last_reward_terms = None
        self.last_crash_signal_available = False
        self.last_crash_source = None
        self.last_crash_reason = ""

        obs = self._get_observation()
        return obs, self._info()

    def _reset_sim_with_retries(self) -> bool:
        attempts = max(1, int(self.config.ros.reset_sim_attempts))
        delay_sec = max(
            0.0,
            float(self.config.ros.reset_sim_retry_delay_sec),
        )
        for attempt in range(1, attempts + 1):
            if self.adapter.reset_sim():
                return True
            if attempt < attempts:
                self.adapter.get_logger().warn(
                    f"Simulation reset attempt {attempt}/{attempts} "
                    "failed; retrying."
                )
                if delay_sec > 0.0:
                    time.sleep(delay_sec)
        return False

    def step(
        self,
        action: np.ndarray,
    ) -> tuple[np.ndarray, float, bool, bool, dict[str, Any]]:
        self._ensure_connected()
        action = np.asarray(action, dtype=np.float32)
        action = np.clip(action, self.action_space.low, self.action_space.high)
        self.last_action = action

        self.adapter.step(action, self.action_cfg.dt)
        obs = self._get_observation()
        self.step_count += 1

        position = self.last_state_position
        relative_yaw = float(self.last_state_raw[2])
        reached_goal = self._is_goal_reached(position)
        crashed = self._is_crashed()
        workspace_violation = self._workspace_violation(position)
        outside_workspace = bool(workspace_violation)
        truncated = self.step_count >= self.env_cfg.max_episode_steps

        reward_terms = compute_reward(
            self.reward_cfg,
            self.env_cfg,
            self.action_cfg,
            position=position,
            goal=self.goal_position,
            action=action,
            previous_distance=self.previous_distance,
            initial_distance=self.initial_distance,
            min_depth_m=self._near_depth_m(),
            relative_yaw_rad=relative_yaw,
            reached_goal=reached_goal,
            crashed=crashed,
            outside_workspace=outside_workspace,
        )
        self.last_reward_terms = reward_terms
        self.previous_distance = float(
            np.linalg.norm(position - self.goal_position)
        )

        terminated = reached_goal or crashed or outside_workspace
        info = self._info()
        info["is_success"] = reached_goal
        info["is_crash"] = crashed
        info["is_not_in_workspace"] = outside_workspace
        info["workspace_violation"] = workspace_violation
        info["is_timeout"] = truncated and not terminated
        info["termination_reason"] = (
            reward_terms.terminal_reason
            or ("timeout" if info["is_timeout"] else "")
        )
        info["position_x"] = float(position[0])
        info["position_y"] = float(position[1])
        info["position_z"] = float(position[2])
        return obs, reward_terms.reward, terminated, truncated, info

    def render(self) -> np.ndarray | None:
        return self.last_depth_m

    def close(self) -> None:
        if self.adapter is not None:
            self.adapter.close()
            self.adapter = None

    def _sample_goal_position(self) -> np.ndarray:
        if self.env_cfg.goal_position is not None:
            goal = np.asarray(self.env_cfg.goal_position, dtype=np.float32)
            rejection_reason = self._goal_rejection_reason(goal)
            if rejection_reason is not None:
                raise RuntimeError(
                    f"Configured goal_position {goal.tolist()} is invalid: "
                    f"{rejection_reason}."
                )
            return goal

        attempts = max(1, int(self.env_cfg.goal_sample_attempts))
        last_rejection_reason = "no candidates were sampled"
        last_goal: np.ndarray | None = None
        for _ in range(attempts):
            goal = self._sample_unchecked_goal_position()
            last_goal = goal
            rejection_reason = self._goal_rejection_reason(goal)
            if rejection_reason is None:
                return goal
            last_rejection_reason = rejection_reason

        self.goal_sample_failures += 1
        detail = f"after {attempts} attempts: {last_rejection_reason}"
        if last_goal is not None:
            detail += f"; last_candidate={last_goal.tolist()}"
        raise RuntimeError(f"Failed to sample a valid goal position {detail}.")

    def _sample_unchecked_goal_position(self) -> np.ndarray:
        angle = float(
            self.np_random.uniform(0.0, self.env_cfg.goal_random_yaw_rad)
        )
        goal = self.start_position.copy()
        goal[0] += self.env_cfg.goal_distance * math.cos(angle)
        goal[1] += self.env_cfg.goal_distance * math.sin(angle)
        goal[2] = self._sample_goal_z()
        return goal.astype(np.float32)

    def _sample_goal_z(self) -> float:
        z_range = self.env_cfg.goal_z_range
        if z_range is not None:
            z_min, z_max = self._range_bounds(z_range, "goal_z_range")
            return float(self.np_random.uniform(z_min, z_max))

        offset_range = self.env_cfg.goal_z_offset_range
        if offset_range is None:
            return float(self.start_position[2])

        z_min, z_max = self._range_bounds(
            offset_range,
            "goal_z_offset_range",
        )
        return float(
            self.start_position[2] + self.np_random.uniform(z_min, z_max)
        )

    def _range_bounds(
        self,
        bounds: list[float],
        name: str,
    ) -> tuple[float, float]:
        if len(bounds) != 2:
            raise RuntimeError(f"{name} must contain exactly two values.")
        lower = float(bounds[0])
        upper = float(bounds[1])
        if upper < lower:
            lower, upper = upper, lower
        return lower, upper

    def _goal_rejection_reason(self, goal: np.ndarray) -> str | None:
        workspace_violation = self._workspace_violation(goal)
        if workspace_violation:
            return f"outside workspace: {workspace_violation}"

        altitude_reason = self._goal_altitude_rejection_reason(goal)
        if altitude_reason:
            return altitude_reason

        if self.env_cfg.goal_collision_check:
            return self._goal_clearance_rejection_reason(goal)
        return None

    def _goal_altitude_rejection_reason(
        self,
        goal: np.ndarray,
    ) -> str | None:
        min_altitude = max(float(self.env_cfg.goal_min_altitude), 0.0)
        if self.env_cfg.goal_collision_check:
            min_altitude = max(
                min_altitude,
                self._effective_goal_clearance_m(),
            )

        altitude = float(goal[2])
        if altitude < min_altitude:
            return (
                f"goal altitude z={altitude:.3f}<"
                f"min_goal_altitude={min_altitude:.3f}"
            )
        return None

    def _effective_goal_clearance_m(self) -> float:
        clearance = max(float(self.env_cfg.goal_clearance_m), 0.0)
        if clearance <= 0.0:
            clearance = max(float(self.env_cfg.crash_distance), 0.0)
        return clearance

    def _goal_clearance_rejection_reason(
        self,
        goal: np.ndarray,
    ) -> str | None:
        depth_m = self._goal_check_depth_image()
        if depth_m is None:
            return "goal collision check requires a depth image"

        delta = goal - self.start_position
        horizontal_dist = float(np.linalg.norm(delta[:2]))
        if horizontal_dist <= 1e-6:
            return "goal has no horizontal separation"

        half_h_fov = math.radians(
            float(self.env_cfg.depth_horizontal_fov_deg)
        ) / 2.0
        if half_h_fov <= 0.0:
            return "depth_horizontal_fov_deg must be positive"

        clearance = self._effective_goal_clearance_m()
        bearing = math.atan2(float(delta[1]), float(delta[0]))
        relative_yaw = wrap_to_pi(bearing - self.last_yaw_rad)
        yaw_margin = math.atan2(clearance, horizontal_dist)
        if abs(relative_yaw) + yaw_margin > half_h_fov:
            return (
                "goal outside depth camera FOV for clearance check: "
                f"relative_yaw_deg={math.degrees(relative_yaw):.1f}, "
                f"half_fov_deg={math.degrees(half_h_fov):.1f}"
            )

        height, width = depth_m.shape[:2]
        half_v_fov = self._depth_vertical_fov_rad(height, width) / 2.0
        line_dist = float(np.linalg.norm(delta))
        relative_pitch = math.atan2(float(delta[2]), horizontal_dist)
        pitch_margin = math.atan2(clearance, max(line_dist, 1e-6))
        if abs(relative_pitch) + pitch_margin > half_v_fov:
            return (
                "goal outside depth camera vertical FOV for clearance "
                f"check: relative_pitch_deg="
                f"{math.degrees(relative_pitch):.1f}, "
                f"half_fov_deg={math.degrees(half_v_fov):.1f}"
            )

        col_center = int(
            round((relative_yaw + half_h_fov) / (2.0 * half_h_fov)
                  * (width - 1))
        )
        row_center = int(
            round((0.5 - relative_pitch / (2.0 * half_v_fov))
                  * (height - 1))
        )
        col_radius = max(
            1,
            int(math.ceil(yaw_margin / (2.0 * half_h_fov) * width)),
        )
        row_radius = max(
            1,
            int(math.ceil(pitch_margin / (2.0 * half_v_fov) * height)),
        )

        col_min = max(0, col_center - col_radius)
        col_max = min(width, col_center + col_radius + 1)
        row_min = max(0, row_center - row_radius)
        row_max = min(height, row_center + row_radius + 1)
        patch = depth_m[row_min:row_max, col_min:col_max]

        near_depth = robust_near_depth(
            patch,
            self.env_cfg.max_depth_meters,
            self.env_cfg.depth_min_valid_m,
            self.env_cfg.depth_crash_percentile,
        )
        required_depth = min(
            float(self.env_cfg.max_depth_meters),
            line_dist + clearance,
        )
        if near_depth + 1e-3 < required_depth:
            return (
                "goal blocked by depth obstacle: "
                f"near_depth_m={near_depth:.3f}<"
                f"required_clear_depth_m={required_depth:.3f}"
            )
        return None

    def _goal_check_depth_image(self) -> np.ndarray | None:
        if self.goal_check_depth_m is not None:
            return self.goal_check_depth_m
        if self.adapter is None:
            return None

        depth_raw_m = clean_depth(
            self.adapter.get_depth_image(),
            self.env_cfg.max_depth_meters,
            self.env_cfg.depth_min_valid_m,
        )
        depth_m = resize_depth(
            depth_raw_m,
            self.env_cfg.depth_image_width,
            self.env_cfg.depth_image_height,
        )
        self.goal_check_depth_m = depth_m.astype(np.float32)
        return self.goal_check_depth_m

    def _depth_vertical_fov_rad(self, height: int, width: int) -> float:
        half_h_fov = math.radians(
            float(self.env_cfg.depth_horizontal_fov_deg)
        ) / 2.0
        aspect = float(height) / max(float(width), 1.0)
        return 2.0 * math.atan(math.tan(half_h_fov) * aspect)

    def _get_observation(self) -> np.ndarray:
        self._ensure_connected()
        depth_raw_m = clean_depth(
            self.adapter.get_depth_image(),
            self.env_cfg.max_depth_meters,
            self.env_cfg.depth_min_valid_m,
        )
        self.last_depth_raw_m = depth_raw_m.astype(np.float32)
        depth_m = resize_depth(
            depth_raw_m,
            self.env_cfg.depth_image_width,
            self.env_cfg.depth_image_height,
        )
        self.last_depth_m = depth_m.astype(np.float32)

        depth_feature_m = closest_depth_grid(depth_m, *self.depth_grid_shape)
        self.last_depth_feature_m = depth_feature_m.astype(np.float32)
        depth_proximity = 1.0 - np.clip(
            depth_feature_m / self.env_cfg.max_depth_meters,
            0.0,
            1.0,
        )

        state_norm, state_raw, position = self._get_state_features()
        self.last_state_raw = state_raw.astype(np.float32)
        self.last_state_position = position.astype(np.float32)

        obs = np.concatenate([depth_proximity, state_norm]).astype(np.float32)
        obs = np.clip(
            obs,
            self.observation_space.low,
            self.observation_space.high,
        )
        self.last_observation = obs
        return obs

    def _get_state_features(self) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        position, yaw_rad = self.adapter.get_pose()
        velocity = self.adapter.get_velocity()
        self.last_yaw_rad = yaw_rad

        goal_delta = self.goal_position - position
        d_xy = float(np.linalg.norm(goal_delta[:2]))
        d_z = float(position[2] - self.goal_position[2])
        goal_yaw = math.atan2(float(goal_delta[1]), float(goal_delta[0]))
        relative_yaw = wrap_to_pi(goal_yaw - yaw_rad)
        v_xy, v_z, yaw_rate = map(float, velocity)

        d_xy_norm = np.clip(d_xy / max(self.initial_distance, 1e-6), 0.0, 1.0)
        d_z_norm = np.clip(
            d_z / max(self.env_cfg.state_z_norm_m, 1e-6),
            -1.0,
            1.0,
        )
        relative_yaw_norm = np.clip(relative_yaw / math.pi, -1.0, 1.0)
        v_xy_norm = np.clip(
            (v_xy - self.action_cfg.v_xy_min)
            / max(self.action_cfg.v_xy_max - self.action_cfg.v_xy_min, 1e-6),
            0.0,
            1.0,
        )
        v_z_norm = np.clip(
            v_z / max(self.action_cfg.v_z_max, 1e-6),
            -1.0,
            1.0,
        )
        yaw_rate_norm = np.clip(
            yaw_rate
            / max(math.radians(self.action_cfg.yaw_rate_max_deg), 1e-6),
            -1.0,
            1.0,
        )

        state_norm = np.array(
            [
                d_xy_norm,
                d_z_norm,
                relative_yaw_norm,
                v_xy_norm,
                v_z_norm,
                yaw_rate_norm,
            ],
            dtype=np.float32,
        )
        state_raw = np.array(
            [d_xy, d_z, relative_yaw, v_xy, v_z, yaw_rate],
            dtype=np.float32,
        )
        return state_norm, state_raw, position

    def _is_goal_reached(self, position: np.ndarray) -> bool:
        return (
            float(np.linalg.norm(position - self.goal_position))
            < self.env_cfg.accept_radius
        )

    def _is_crashed(self) -> bool:
        crash_state = self.adapter.get_crash_state()
        self.last_crash_signal_available = crash_state is not None

        if crash_state is not None:
            self.last_crash_source = "uav_crash_topic" if crash_state else None
            self.last_crash_reason = self.adapter.get_crash_reason()
            return bool(crash_state)

        min_depth_m = self._near_depth_m()
        altitude_m = float(self.last_state_position[2])
        min_airborne_altitude = float(
            self.env_cfg.depth_min_airborne_altitude
        )
        if (
            min_airborne_altitude > 0.0
            and altitude_m < min_airborne_altitude
        ):
            self.last_crash_source = None
            self.last_crash_reason = (
                f"depth_suppressed:low_altitude:altitude_m={altitude_m:.3f}<"
                f"min_airborne_altitude_m={min_airborne_altitude:.3f} "
                f"min_depth_m={min_depth_m:.3f}"
            )
            return False

        depth_crash = min_depth_m < self.env_cfg.crash_distance
        self.last_crash_source = "depth" if depth_crash else None
        self.last_crash_reason = (
            f"min_depth_m={min_depth_m:.3f} < "
            f"crash_distance={self.env_cfg.crash_distance:.3f}"
            if depth_crash
            else ""
        )
        return bool(depth_crash)

    def _near_depth_m(self) -> float:
        return robust_near_depth(
            self.last_depth_m,
            self.env_cfg.max_depth_meters,
            self.env_cfg.depth_min_valid_m,
            self.env_cfg.depth_crash_percentile,
        )

    def _is_outside_workspace(self, position: np.ndarray) -> bool:
        return bool(self._workspace_violation(position))

    def _workspace_violation(self, position: np.ndarray) -> str:
        checks = (
            ("x", float(position[0]), self.env_cfg.workspace_x),
            ("y", float(position[1]), self.env_cfg.workspace_y),
            ("z", float(position[2]), self.env_cfg.workspace_z),
        )
        for axis, value, bounds in checks:
            lower = float(bounds[0])
            upper = float(bounds[1])
            if value < lower:
                return f"{axis}={value:.3f}<min={lower:.3f}"
            if value > upper:
                return f"{axis}={value:.3f}>max={upper:.3f}"
        return ""

    def _info(self) -> dict[str, Any]:
        return {
            "episode_num": self.episode_count,
            "step": self.step_count,
            "goal_position": self.goal_position.copy(),
            "position": self.last_state_position.copy(),
            "yaw_rad": self.last_yaw_rad,
            "state_raw": self.last_state_raw.copy(),
            "depth_feature_m": self.last_depth_feature_m.copy(),
            "min_depth_m": (
                float(np.nanmin(self.last_depth_m))
                if self.last_depth_m is not None
                else None
            ),
            "last_action": self.last_action.copy(),
            "crash_signal_available": self.last_crash_signal_available,
            "crash_source": self.last_crash_source,
            "crash_reason": self.last_crash_reason,
            "reward_terms": (
                self.last_reward_terms.to_dict()
                if self.last_reward_terms
                else None
            ),
        }

    def _ensure_connected(self) -> None:
        if self.adapter is None:
            raise RuntimeError(
                "ROS connection is disabled. "
                "Create the environment with connect=True."
            )
