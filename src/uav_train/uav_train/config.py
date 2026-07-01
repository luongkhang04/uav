from __future__ import annotations

from dataclasses import asdict, dataclass, field
from pathlib import Path
from typing import Any

try:
    import yaml
except ImportError:
    yaml = None


DEFAULT_CONFIG_FILE = "xai_sac_gazebo.yaml"


@dataclass
class RosTopicsConfig:
    odom_topic: str = "/uav/odom"
    imu_topic: str = "/uav/imu"
    depth_topic: str = "/uav/camera/depth/image"
    crash_topic: str = "/uav/crash"
    crash_reason_topic: str = "/uav/crash_reason"
    cmd_topic: str = "/uav/cmd_vel_body"
    offboard_arm_service: str = "/uav/offboard_arm"
    land_service: str = "/uav/land"
    disarm_service: str = "/uav/disarm"
    reset_sim_service: str = "/uav/reset_sim"
    control_rate_hz: float = 10.0
    data_timeout_sec: float = 5.0
    service_timeout_sec: float = 15.0
    crash_state_timeout_sec: float = 1.0
    auto_arm: bool = True
    stop_on_reset: bool = True
    reset_sim_on_reset: bool = True
    reset_sim_attempts: int = 3
    reset_sim_retry_delay_sec: float = 0.5
    reset_sim_required: bool = False
    stop_on_close: bool = True
    depth_scale: float = 1.0


@dataclass
class EnvironmentConfig:
    env_name: str = "GazeboPx4Avoid"
    start_position: list[float] = field(
        default_factory=lambda: [0.0, 0.0, 0.0]
    )
    start_random_yaw_rad: float = 6.283185307179586
    goal_position: list[float] | None = None
    goal_distance: float = 50.0
    goal_random_yaw_rad: float = 6.283185307179586
    goal_z_range: list[float] | None = field(
        default_factory=lambda: [2.0, 5.0]
    )
    goal_z_offset_range: list[float] | None = None
    goal_sample_attempts: int = 100
    goal_collision_check: bool = True
    goal_clearance_m: float = 2.0
    goal_min_altitude: float = 1.0
    workspace_x: list[float] = field(default_factory=lambda: [-70.0, 70.0])
    workspace_y: list[float] = field(default_factory=lambda: [-70.0, 70.0])
    workspace_z: list[float] = field(default_factory=lambda: [-1.0, 50.0])
    max_episode_steps: int = 400
    accept_radius: float = 2.0
    crash_distance: float = 1.5
    depth_min_airborne_altitude: float = 0.75
    depth_min_valid_m: float = 0.05
    depth_crash_percentile: float = 0.5
    max_depth_meters: float = 15.0
    depth_image_width: int = 90
    depth_image_height: int = 60
    depth_grid_rows: int = 5
    depth_grid_cols: int = 5
    depth_horizontal_fov_deg: float = 90.0
    obstacle_warning_distance: float = 10.0
    state_z_norm_m: float = 5.0
    seed: int = 0


@dataclass
class ActionConfig:
    dt: float = 0.1
    v_xy_min: float = 0.5
    v_xy_max: float = 5.0
    v_z_max: float = 2.0
    yaw_rate_max_deg: float = 30.0


@dataclass
class RewardConfig:
    goal: float = 10.0
    crash: float = -20.0
    outside_workspace: float = -10.0
    alpha_dist: float = 50.0
    alpha_alt: float = 0.1
    alpha_obs: float = 0.2
    alpha_act: float = 0.1
    alpha_yaw: float = 0.5
    gamma_z: float = 5.0


@dataclass
class SACConfig:
    total_timesteps: int = 100_000
    learning_rate: float = 1e-3
    gamma: float = 0.99
    learning_starts: int = 2_000
    buffer_size: int = 50_000
    batch_size: int = 512
    train_freq: int = 100
    gradient_steps: int = 100
    net_arch: list[int] = field(default_factory=lambda: [64, 32, 16])
    activation_fn: str = "tanh"
    ent_coef: str | float = "auto"
    seed: int = 0
    checkpoint_freq: int = 10_000


@dataclass
class ProjectConfig:
    ros: RosTopicsConfig = field(default_factory=RosTopicsConfig)
    environment: EnvironmentConfig = field(default_factory=EnvironmentConfig)
    action: ActionConfig = field(default_factory=ActionConfig)
    reward: RewardConfig = field(default_factory=RewardConfig)
    sac: SACConfig = field(default_factory=SACConfig)

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> "ProjectConfig":
        data = data or {}
        reward_data = dict(data.get("reward") or {})
        if "alpha_pos" in reward_data and "alpha_alt" not in reward_data:
            reward_data["alpha_alt"] = reward_data.pop("alpha_pos")
        else:
            reward_data.pop("alpha_pos", None)
        reward_data.pop("beta_xy", None)
        return cls(
            ros=RosTopicsConfig(**data.get("ros", {})),
            environment=EnvironmentConfig(**data.get("environment", {})),
            action=ActionConfig(**data.get("action", {})),
            reward=RewardConfig(**reward_data),
            sac=SACConfig(**data.get("sac", {})),
        )

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


def default_config_path() -> Path:
    candidates: list[Path] = []
    try:
        from ament_index_python.packages import get_package_share_directory

        share_dir = Path(get_package_share_directory("uav_train"))
        candidates.append(share_dir / "config" / DEFAULT_CONFIG_FILE)
    except Exception:
        pass

    source_root = Path(__file__).resolve().parents[1]
    candidates.append(source_root / "config" / DEFAULT_CONFIG_FILE)
    candidates.append(
        Path.cwd() / "src" / "uav_train" / "config" / DEFAULT_CONFIG_FILE
    )

    for candidate in candidates:
        if candidate.exists():
            return candidate
    return candidates[0]


def load_config(path: str | Path | None = None) -> ProjectConfig:
    config_path = Path(path) if path is not None else default_config_path()
    with config_path.open("r", encoding="utf-8") as stream:
        if yaml is not None:
            data = yaml.safe_load(stream) or {}
        else:
            data = _load_simple_yaml(stream.read())
    return ProjectConfig.from_dict(data)


def save_config(config: ProjectConfig, path: str | Path) -> None:
    output_path = Path(path)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    with output_path.open("w", encoding="utf-8") as stream:
        if yaml is not None:
            yaml.safe_dump(config.to_dict(), stream, sort_keys=False)
        else:
            stream.write(_dump_simple_yaml(config.to_dict()))


def _load_simple_yaml(text: str) -> dict[str, Any]:
    data: dict[str, Any] = {}
    current_section: str | None = None

    for raw_line in text.splitlines():
        line = raw_line.split("#", 1)[0].rstrip()
        if not line.strip():
            continue

        indent = len(line) - len(line.lstrip(" "))
        key, sep, value = line.strip().partition(":")
        if not sep:
            raise ValueError(f"Invalid config line: {raw_line}")

        if indent == 0:
            if value.strip():
                data[key] = _parse_scalar(value.strip())
                current_section = None
            else:
                data[key] = {}
                current_section = key
        elif indent == 2 and current_section is not None:
            data[current_section][key] = _parse_scalar(value.strip())
        else:
            raise ValueError(f"Unsupported YAML line: {raw_line}")

    return data


def _parse_scalar(value: str) -> Any:
    value = value.strip()
    if value == "":
        return None
    lowered = value.lower()
    if lowered in {"null", "none", "~"}:
        return None
    if lowered == "true":
        return True
    if lowered == "false":
        return False
    if value.startswith("[") and value.endswith("]"):
        inner = value[1:-1].strip()
        if not inner:
            return []
        return [_parse_scalar(part.strip()) for part in inner.split(",")]
    if (
        (value.startswith('"') and value.endswith('"'))
        or (value.startswith("'") and value.endswith("'"))
    ):
        return value[1:-1]
    try:
        if any(marker in value for marker in [".", "e", "E"]):
            return float(value)
        return int(value)
    except ValueError:
        return value


def _dump_simple_yaml(data: dict[str, Any]) -> str:
    lines: list[str] = []
    for section, values in data.items():
        if isinstance(values, dict):
            lines.append(f"{section}:")
            for key, value in values.items():
                lines.append(f"  {key}: {_format_scalar(value)}")
        else:
            lines.append(f"{section}: {_format_scalar(values)}")
        lines.append("")
    return "\n".join(lines).rstrip() + "\n"


def _format_scalar(value: Any) -> str:
    if value is None:
        return "null"
    if isinstance(value, bool):
        return str(value).lower()
    if isinstance(value, list):
        return "[" + ", ".join(_format_scalar(item) for item in value) + "]"
    if isinstance(value, str):
        return value
    return str(value)
