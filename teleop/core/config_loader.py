from __future__ import annotations

from pathlib import Path
from typing import Any, Sequence

import numpy as np
from omegaconf import DictConfig, ListConfig, OmegaConf

from my_teleop.teleop.follower.ur5_rtde_follower import UR5RTDEFollower
from my_teleop.teleop.leader.gello_leader import GelloLeader, GelloLeaderConfig
from my_teleop.teleop.safety.safety_checker import SafetyChecker

_EVAL_NAMESPACE = {"np": np}


def _eval_numeric_expr(value: Any) -> float:
    """将配置中的数值或含 np.pi 的表达式解析为 float。"""
    if isinstance(value, (int, float)):
        return float(value)
    if isinstance(value, str):
        try:
            result = eval(value, {"__builtins__": {}}, _EVAL_NAMESPACE)
        except Exception as exc:
            raise ValueError(f"配置表达式解析失败: {value!r}") from exc
        return float(result)
    raise TypeError(f"不支持的配置类型: {type(value)!r}")


def _eval_numeric_list(values: Sequence[Any]) -> tuple[float, ...]:
    return tuple(_eval_numeric_expr(v) for v in values)


def load_config(config_path: str | Path) -> DictConfig:
    cfg = OmegaConf.load(config_path)
    if isinstance(cfg, ListConfig):
        raise ValueError("配置文件根节点必须是字典，不能是列表")
    if not isinstance(cfg, DictConfig):
        raise TypeError(f"不支持的配置类型: {type(cfg)!r}")
    return cfg


def build_leader_config(cfg: DictConfig) -> GelloLeaderConfig:
    leader = cfg.leader
    gripper = leader.gripper
    gripper_read_interval = OmegaConf.select(
        cfg,
        "leader.gripper.read_interval",
        default=0.1,
    )
    return GelloLeaderConfig(
        port=leader.port,
        baudrate=int(leader.baudrate),
        joint_ids=tuple(int(j) for j in leader.joint_ids),
        joint_offsets=_eval_numeric_list(leader.joint_offsets),
        joint_signs=tuple(int(s) for s in leader.joint_signs),
        gripper_config=(
            int(gripper.id),
            float(gripper.open_position),
            float(gripper.close_position),
        ),
        gripper_read_interval=_eval_numeric_expr(gripper_read_interval),
    )


def build_leader(cfg: DictConfig) -> GelloLeader:
    return GelloLeader(build_leader_config(cfg))


def build_follower(cfg: DictConfig) -> UR5RTDEFollower:
    follower = cfg.follower
    servo = follower.servo
    gripper_command_interval = OmegaConf.select(
        cfg,
        "follower.gripper_command_interval",
        default=0.05,
    )
    return UR5RTDEFollower(
        robot_ip=follower.robot_ip,
        use_gripper=bool(follower.use_gripper),
        gripper_port=int(follower.gripper_port),
        gripper_command_interval=_eval_numeric_expr(gripper_command_interval),
        servo_velocity=_eval_numeric_expr(servo.velocity),
        servo_acceleration=_eval_numeric_expr(servo.acceleration),
        servo_dt=_eval_numeric_expr(servo.dt),
        servo_lookahead_time=_eval_numeric_expr(servo.lookahead_time),
        servo_gain=int(servo.gain),
    )


def build_safety_checker(cfg: DictConfig) -> SafetyChecker:
    return SafetyChecker(max_joint_delta=_eval_numeric_expr(cfg.control.max_joint_delta))