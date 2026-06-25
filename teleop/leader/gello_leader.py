import time
from dataclasses import dataclass
from typing import Sequence

import numpy as np

from my_teleop.teleop.leader.dynamixel_sdk_driver import (
    DEFAULT_BAUDRATE,
    DynamixelSDKDriver,
)


DEFAULT_UR_GELLO_PORT = "/dev/serial/by-id/YOUR_GELLO_DEVICE"


@dataclass
class GelloLeaderConfig:
    port: str = DEFAULT_UR_GELLO_PORT
    joint_ids: Sequence[int] = (1, 2, 3, 4, 5, 6)
    joint_offsets: Sequence[float] = (
        1 * np.pi / 2,
        3 * np.pi / 2,
        np.pi,
        np.pi,
        np.pi,
        np.pi,
    )
    joint_signs: Sequence[int] = (1, 1, -1, 1, 1, 1)
    gripper_config: tuple[int, float, float] = (7, 199, 157)
    baudrate: int = DEFAULT_BAUDRATE
    gripper_read_interval: float = 0.1

    def __post_init__(self) -> None:
        if len(self.joint_ids) != len(self.joint_offsets):
            raise ValueError("joint_ids 和 joint_offsets 长度必须一致")
        if len(self.joint_ids) != len(self.joint_signs):
            raise ValueError("joint_ids 和 joint_signs 长度必须一致")


class GelloLeader:
    """将 Dynamixel 原始角度解释成遥操作可用的 GELLO 状态。"""

    def __init__(self, config: GelloLeaderConfig) -> None:
        self.config = config
        gripper_id = config.gripper_config[0]
        driver_ids = tuple(config.joint_ids) + (gripper_id,)

        self.joint_offsets = np.array(config.joint_offsets, dtype=float)
        self.joint_signs = np.array(config.joint_signs, dtype=float)
        self.gripper_open_rad = np.deg2rad(config.gripper_config[1])
        self.gripper_close_rad = np.deg2rad(config.gripper_config[2])
        self._gripper_id = int(config.gripper_config[0])
        self._last_raw_gripper_position = self.gripper_open_rad
        self._last_gripper_read_time = 0.0

        self.driver = DynamixelSDKDriver(
            port=config.port,
            ids=driver_ids,
            baudrate=config.baudrate,
        )

    def read_raw(self) -> np.ndarray:
        """读取 7 个 Dynamixel 电机原始角度，单位是弧度。"""
        arm_positions = self.driver.read_positions(self.config.joint_ids)
        raw_gripper_position = self._read_gripper_position()
        return np.concatenate([arm_positions, np.array([raw_gripper_position])])

    def read(self) -> np.ndarray:
        """读取 GELLO 状态，输出 [6 个臂关节 + 1 个夹爪]。"""
        raw_positions = self.read_raw()
        expected_dim = len(self.config.joint_ids) + 1
        if len(raw_positions) != expected_dim:
            raise ValueError(f"期望读取 {expected_dim} 维，实际读取 {len(raw_positions)} 维")

        raw_arm_positions = raw_positions[: len(self.config.joint_ids)]
        raw_gripper_position = raw_positions[-1]

        arm_positions = (raw_arm_positions - self.joint_offsets) * self.joint_signs
        gripper_position = self.normalize_gripper(raw_gripper_position)

        return np.concatenate([arm_positions, np.array([gripper_position])])

    def normalize_gripper(self, raw_gripper_position: float) -> float:
        """将夹爪角度归一化到 0 到 1，0 表示打开，1 表示闭合。"""
        gripper_position = (raw_gripper_position - self.gripper_open_rad) / (
            self.gripper_close_rad - self.gripper_open_rad
        )
        return float(np.clip(gripper_position, 0.0, 1.0))

    def _read_gripper_position(self) -> float:
        """夹爪输入低频读取，避免第 7 个电机拖慢手臂主循环。"""
        now = time.monotonic()
        if now - self._last_gripper_read_time < self.config.gripper_read_interval:
            return self._last_raw_gripper_position

        raw_position = self.driver.read_positions((self._gripper_id,))[0]
        self._last_raw_gripper_position = float(raw_position)
        self._last_gripper_read_time = now
        return self._last_raw_gripper_position

    def close(self) -> None:
        self.driver.close()
