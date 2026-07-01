from __future__ import annotations

import argparse
import datetime as dt
import multiprocessing as mp
from pathlib import Path

import rclpy
import torch as th
from stable_baselines3 import SAC
from stable_baselines3.common.callbacks import CheckpointCallback
from stable_baselines3.common.env_checker import check_env
from stable_baselines3.common.monitor import Monitor

from .config import (
    ProjectConfig,
    default_config_path,
    load_config,
    save_config,
)
from .env import XaiSacGazeboEnv


MONITOR_INFO_KEYWORDS = (
    "termination_reason",
    "is_not_in_workspace",
    "workspace_violation",
    "is_crash",
    "is_timeout",
    "position_x",
    "position_y",
    "position_z",
)


def build_sac_model(
    config: ProjectConfig,
    env: Monitor,
    log_dir: Path,
    device: str,
) -> SAC:
    activation_fn = activation_from_name(config.sac.activation_fn)
    policy_kwargs = {
        "net_arch": config.sac.net_arch,
        "activation_fn": activation_fn,
    }
    return SAC(
        "MlpPolicy",
        env,
        learning_rate=config.sac.learning_rate,
        gamma=config.sac.gamma,
        learning_starts=config.sac.learning_starts,
        buffer_size=config.sac.buffer_size,
        batch_size=config.sac.batch_size,
        train_freq=(config.sac.train_freq, "step"),
        gradient_steps=config.sac.gradient_steps,
        ent_coef=config.sac.ent_coef,
        policy_kwargs=policy_kwargs,
        tensorboard_log=str(log_dir / "tb"),
        seed=config.sac.seed,
        verbose=1,
        device=device,
    )


def activation_from_name(name: str) -> type[th.nn.Module]:
    normalized = name.strip().lower()
    if normalized == "relu":
        return th.nn.ReLU
    if normalized == "tanh":
        return th.nn.Tanh
    if normalized == "elu":
        return th.nn.ELU
    if normalized == "leaky_relu":
        return th.nn.LeakyReLU
    raise ValueError(f"Unsupported activation_fn: {name}")


def make_run_dir(root: Path, name: str | None) -> Path:
    timestamp = dt.datetime.now().strftime("%Y%m%d_%H%M%S")
    run_name = name or f"xai_sac_gazebo_{timestamp}"
    run_dir = root / run_name
    run_dir.mkdir(parents=True, exist_ok=True)
    return run_dir


def infer_replay_buffer_path(model_path: str | Path) -> Path | None:
    path = Path(model_path)
    stem = path.stem
    if stem == "model_interrupt":
        return path.with_name("replay_buffer_interrupt.pkl")
    if stem == "model_final":
        return path.with_name("replay_buffer_final.pkl")
    if stem.endswith("_steps"):
        parts = stem.split("_")
        if len(parts) >= 3 and parts[-1] == "steps" and parts[-2].isdigit():
            prefix = "_".join(parts[:-2])
            return path.with_name(
                f"{prefix}_replay_buffer_{parts[-2]}_steps.pkl"
            )
    return None


def save_training_state(
    model: SAC,
    run_dir: Path,
    suffix: str,
) -> tuple[Path, Path]:
    model_path = run_dir / f"model_{suffix}.zip"
    replay_buffer_path = run_dir / f"replay_buffer_{suffix}.pkl"
    model.save(model_path)
    model.save_replay_buffer(replay_buffer_path)
    return model_path, replay_buffer_path


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Train the XAI SAC UAV policy with ROS/Gazebo."
    )
    parser.add_argument(
        "--config",
        default=str(default_config_path()),
        help="YAML config path.",
    )
    parser.add_argument(
        "--run-root",
        default="logs/xai_sac_gazebo",
        help="Directory for runs.",
    )
    parser.add_argument("--run-name", default=None)
    parser.add_argument("--resume", default=None)
    parser.add_argument(
        "--resume-replay-buffer",
        default="auto",
        help="Replay buffer path, 'auto', or 'none'.",
    )
    parser.add_argument("--device", default="auto")
    parser.add_argument("--check-env", action="store_true")
    parser.add_argument("--progress-bar", action="store_true")
    return parser.parse_args()


def main() -> None:
    mp.freeze_support()
    args = parse_args()
    config = load_config(args.config)
    run_dir = make_run_dir(Path(args.run_root), args.run_name)
    save_config(config, run_dir / "config.yaml")

    env = Monitor(
        XaiSacGazeboEnv(config),
        filename=str(run_dir / "monitor.csv"),
        info_keywords=MONITOR_INFO_KEYWORDS,
    )
    try:
        if args.check_env:
            check_env(env.unwrapped, warn=True)

        if args.resume:
            model = SAC.load(args.resume, env=env, device=args.device)
            replay_buffer_path = None
            if args.resume_replay_buffer.lower() != "none":
                if args.resume_replay_buffer == "auto":
                    replay_buffer_path = infer_replay_buffer_path(args.resume)
                else:
                    replay_buffer_path = Path(args.resume_replay_buffer)
            if replay_buffer_path is not None and replay_buffer_path.exists():
                model.load_replay_buffer(replay_buffer_path)
                print(f"Loaded replay buffer: {replay_buffer_path}")
            elif replay_buffer_path is not None:
                print(
                    "Replay buffer not found, resuming with empty buffer: "
                    f"{replay_buffer_path}"
                )
        else:
            model = build_sac_model(config, env, run_dir, args.device)

        checkpoint = CheckpointCallback(
            save_freq=config.sac.checkpoint_freq,
            save_path=str(run_dir / "checkpoints"),
            name_prefix="sac_xai_gazebo",
            save_replay_buffer=True,
            save_vecnormalize=True,
        )

        try:
            model.learn(
                total_timesteps=config.sac.total_timesteps,
                callback=checkpoint,
                log_interval=1,
                progress_bar=args.progress_bar,
                reset_num_timesteps=args.resume is None,
            )
        except KeyboardInterrupt:
            model_path, buffer_path = save_training_state(
                model,
                run_dir,
                "interrupt",
            )
            print("\nTraining interrupted. Saved safe resume state:")
            print(f"  model: {model_path}")
            print(f"  replay buffer: {buffer_path}")
        except Exception:
            try:
                model_path, buffer_path = save_training_state(
                    model,
                    run_dir,
                    "error",
                )
                print("\nTraining stopped by an exception.")
                print(f"  model: {model_path}")
                print(f"  replay buffer: {buffer_path}")
            except Exception as save_exc:
                print(f"\nTraining stopped, and saving failed: {save_exc}")
            raise
        else:
            model_path, buffer_path = save_training_state(
                model,
                run_dir,
                "final",
            )
            print("Training finished. Saved final state:")
            print(f"  model: {model_path}")
            print(f"  replay buffer: {buffer_path}")
    finally:
        env.close()
        if rclpy.ok():
            rclpy.shutdown()

    print(f"Saved run to {run_dir}")


if __name__ == "__main__":
    main()
