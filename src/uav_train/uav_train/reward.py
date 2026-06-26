from __future__ import annotations

from dataclasses import asdict, dataclass

import numpy as np

from .config import ActionConfig, EnvironmentConfig, RewardConfig


@dataclass
class RewardTerms:
    reward: float
    r_dist: float
    r_alt: float
    r_obs: float
    r_act: float
    r_yaw: float
    terminal_reason: str | None = None

    def to_dict(self) -> dict[str, float | str | None]:
        return asdict(self)


def compute_reward(
    reward_cfg: RewardConfig,
    env_cfg: EnvironmentConfig,
    action_cfg: ActionConfig,
    position: np.ndarray,
    goal: np.ndarray,
    action: np.ndarray,
    previous_distance: float,
    initial_distance: float,
    min_depth_m: float,
    relative_yaw_rad: float,
    reached_goal: bool,
    crashed: bool,
    outside_workspace: bool,
) -> RewardTerms:
    if reached_goal:
        return RewardTerms(
            reward=reward_cfg.goal,
            r_dist=0.0,
            r_alt=0.0,
            r_obs=0.0,
            r_act=0.0,
            r_yaw=0.0,
            terminal_reason="goal",
        )
    if crashed:
        return RewardTerms(
            reward=reward_cfg.crash,
            r_dist=0.0,
            r_alt=0.0,
            r_obs=0.0,
            r_act=0.0,
            r_yaw=0.0,
            terminal_reason="crash",
        )
    if outside_workspace:
        return RewardTerms(
            reward=reward_cfg.outside_workspace,
            r_dist=0.0,
            r_alt=0.0,
            r_obs=0.0,
            r_act=0.0,
            r_yaw=0.0,
            terminal_reason="outside_workspace",
        )

    distance_now = float(np.linalg.norm(position - goal))
    r_dist = (previous_distance - distance_now) / max(initial_distance, 1e-6)

    altitude_error = abs(float(position[2] - goal[2]))
    r_alt = float(
        np.clip(altitude_error / max(reward_cfg.gamma_z, 1e-6), 0.0, 1.0)
    )

    if min_depth_m < env_cfg.obstacle_warning_distance:
        denom = max(
            env_cfg.obstacle_warning_distance - env_cfg.crash_distance,
            1e-6,
        )
        r_obs = 1.0 - np.clip(
            (min_depth_m - env_cfg.crash_distance) / denom,
            0.0,
            1.0,
        )
    else:
        r_obs = 0.0

    v_z_cost = (abs(float(action[1])) / max(action_cfg.v_z_max, 1e-6)) ** 2
    yaw_limit = max(np.deg2rad(action_cfg.yaw_rate_max_deg), 1e-6)
    yaw_cost = abs(float(action[2])) / yaw_limit
    r_act = float(v_z_cost + yaw_cost)
    r_yaw = float(abs(relative_yaw_rad) / np.pi)

    reward = (
        reward_cfg.alpha_dist * r_dist
        - reward_cfg.alpha_alt * r_alt
        - reward_cfg.alpha_obs * r_obs
        - reward_cfg.alpha_act * r_act
        - reward_cfg.alpha_yaw * r_yaw
    )

    return RewardTerms(
        reward=float(reward),
        r_dist=float(r_dist),
        r_alt=float(r_alt),
        r_obs=float(r_obs),
        r_act=float(r_act),
        r_yaw=float(r_yaw),
    )
