from pathlib import Path
from typing import Any

from omegaconf import DictConfig, OmegaConf

DEFAULT_LEROBOT_JOINT_NAMES = (
    "shoulder_pan.pos",
    "shoulder_lift.pos",
    "elbow_flex.pos",
    "wrist_1.pos",
    "wrist_2.pos",
    "wrist_3.pos",
    "gripper.pos",
)


def resolve_config_path(config_path: str | Path) -> Path:
    path = Path(config_path).expanduser()
    if path.is_absolute():
        return path
    return Path.cwd() / path


def select_config(cfg: DictConfig, key: str, default: Any = None) -> Any:
    value = OmegaConf.select(cfg, key)
    return default if value is None else value


def lerobot_joint_names(cfg: DictConfig) -> tuple[str, ...]:
    names = tuple(
        str(name)
        for name in select_config(
            cfg,
            "lerobot.joint_names",
            DEFAULT_LEROBOT_JOINT_NAMES,
        )
    )
    if len(names) != 7:
        raise ValueError(f"lerobot.joint_names 必须是 7 个名称，当前为 {len(names)} 个")
    return names
