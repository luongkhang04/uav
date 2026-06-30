from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np
import rclpy
from stable_baselines3 import SAC

from uav_train.config import default_config_path, load_config
from uav_train.env import XaiSacGazeboEnv


def default_model_path() -> Path:
    candidates = [
        Path.cwd() / "models" / "xai_sac" / "airsim.zip",
        Path.home() / "uav" / "models" / "xai_sac" / "airsim.zip",
    ]
    for candidate in candidates:
        if candidate.exists():
            return candidate
    return candidates[0]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Evaluate a trained XAI SAC policy with ROS/Gazebo."
    )
    parser.add_argument(
        "--config",
        default=str(default_config_path()),
        help="YAML config path.",
    )
    parser.add_argument(
        "--model",
        default=str(default_model_path()),
        help="Path to the SAC model zip.",
    )
    parser.add_argument("--episodes", type=int, default=10)
    parser.add_argument("--device", default="auto")
    parser.add_argument(
        "--output",
        default=None,
        help="Optional JSON metrics output path.",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    config = load_config(args.config)
    env = XaiSacGazeboEnv(config)
    model = SAC.load(args.model, device=args.device)
    episode_results: list[dict[str, float | bool | int]] = []

    try:
        for episode in range(args.episodes):
            obs, _ = env.reset()
            done = False
            total_reward = 0.0
            steps = 0
            success = False
            crashed = False
            timeout = False

            while not done:
                action, _ = model.predict(obs, deterministic=True)
                obs, reward, terminated, truncated, info = env.step(action)
                total_reward += float(reward)
                steps += 1
                done = terminated or truncated
                success = bool(info["is_success"])
                crashed = bool(info["is_crash"])
                timeout = bool(info["is_timeout"])

            result = {
                "episode": episode,
                "reward": total_reward,
                "steps": steps,
                "success": success,
                "crashed": crashed,
                "timeout": timeout,
            }
            episode_results.append(result)
            print(json.dumps(result))
    finally:
        env.close()
        if rclpy.ok():
            rclpy.shutdown()

    summary = {
        "episodes": len(episode_results),
        "success_rate": _mean_bool(episode_results, "success"),
        "crash_rate": _mean_bool(episode_results, "crashed"),
        "mean_reward": _mean_float(episode_results, "reward"),
        "mean_steps": _mean_float(episode_results, "steps"),
    }
    print(json.dumps({"summary": summary}, indent=2))

    if args.output:
        output_path = Path(args.output)
        output_path.parent.mkdir(parents=True, exist_ok=True)
        payload = {"summary": summary, "episodes": episode_results}
        output_path.write_text(json.dumps(payload, indent=2), encoding="utf-8")


def _mean_bool(rows: list[dict[str, float | bool | int]], key: str) -> float:
    if not rows:
        return 0.0
    return float(np.mean([bool(row[key]) for row in rows]))


def _mean_float(rows: list[dict[str, float | bool | int]], key: str) -> float:
    if not rows:
        return 0.0
    return float(np.mean([float(row[key]) for row in rows]))


if __name__ == "__main__":
    main()
