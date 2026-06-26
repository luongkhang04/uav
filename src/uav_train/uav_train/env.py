from __future__ import annotations

import math
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
        self.goal_sample_failures = 0

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
        if self.config.ros.auto_arm:
            self.adapter.arm_offboard()

        self.step_count = 0
        self.episode_count += 1
        self.start_position, self.last_yaw_rad = self.adapter.get_pose()
        self.last_state_position = self.start_position.copy()
        self.goal_position = self._sample_goal_position()
        self.initial_distance = max(
            float(np.linalg.norm(self.start_position - self.goal_position)),
            1e-6,
        )
        self.previous_distance = self.initial_distance
        self.last_action = np.zeros(3, dtype=np.float32)
        self.last_reward_terms = None

        obs = self._get_observation()
        return obs, self._info()

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
        outside_workspace = self._is_outside_workspace(position)
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
            min_depth_m=float(np.nanmin(self.last_depth_m)),
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
        info["is_timeout"] = truncated and not terminated
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
        if last_goal is not None:
            return last_goal
        raise RuntimeError(
            "Failed to sample a goal position: "
            f"{last_rejection_reason}"
        )

    def _sample_unchecked_goal_position(self) -> np.ndarray:
        angle = float(
            self.np_random.uniform(0.0, self.env_cfg.goal_random_yaw_rad)
        )
        z_min, z_max = self.env_cfg.goal_z_offset_range
        goal = self.start_position.copy()
        goal[0] += self.env_cfg.goal_distance * math.cos(angle)
        goal[1] += self.env_cfg.goal_distance * math.sin(angle)
        goal[2] += float(self.np_random.uniform(z_min, z_max))
        return goal.astype(np.float32)

    def _goal_rejection_reason(self, goal: np.ndarray) -> str | None:
        if self._is_outside_workspace(goal):
            return "outside workspace"
        return None

    def _get_observation(self) -> np.ndarray:
        self._ensure_connected()
        depth_raw_m = clean_depth(
            self.adapter.get_depth_image(),
            self.env_cfg.max_depth_meters,
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
        return bool(np.nanmin(self.last_depth_m) < self.env_cfg.crash_distance)

    def _is_outside_workspace(self, position: np.ndarray) -> bool:
        return (
            position[0] < self.env_cfg.workspace_x[0]
            or position[0] > self.env_cfg.workspace_x[1]
            or position[1] < self.env_cfg.workspace_y[0]
            or position[1] > self.env_cfg.workspace_y[1]
            or position[2] < self.env_cfg.workspace_z[0]
            or position[2] > self.env_cfg.workspace_z[1]
        )

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
