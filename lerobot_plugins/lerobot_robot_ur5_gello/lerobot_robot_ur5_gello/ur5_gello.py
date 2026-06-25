from __future__ import annotations

import time
from pathlib import Path
from typing import Any

import numpy as np
from omegaconf import DictConfig, OmegaConf

from lerobot.robots import Robot
from my_teleop.teleop.core.config_loader import (
    build_follower,
    build_leader,
    build_safety_checker,
    load_config,
)
from my_teleop.teleop.core.startup_alignment import align_to_leader

from .config_ur5_gello import UR5GelloRobotConfig

DEFAULT_JOINT_NAMES = (
    "shoulder_pan.pos",
    "shoulder_lift.pos",
    "elbow_flex.pos",
    "wrist_1.pos",
    "wrist_2.pos",
    "wrist_3.pos",
    "gripper.pos",
)


def _resolve_config_path(config_path: str | Path) -> Path:
    path = Path(config_path).expanduser()
    if path.is_absolute():
        return path
    return Path.cwd() / path


def _select(cfg: DictConfig, key: str, default: Any = None) -> Any:
    value = OmegaConf.select(cfg, key)
    return default if value is None else value


def _joint_names(cfg: DictConfig) -> tuple[str, ...]:
    names = tuple(str(name) for name in _select(cfg, "lerobot.joint_names", DEFAULT_JOINT_NAMES))
    if len(names) != 7:
        raise ValueError(f"lerobot.joint_names 必须是 7 个名称，当前为 {len(names)} 个")
    return names


class _RealSenseColorCamera:
    """只采集 RealSense 彩色图像，避免影响原有遥操链路。"""

    def __init__(self, name: str, cfg: DictConfig) -> None:
        self.name = name
        self.serial_number = str(_select(cfg, "serial_number", "") or "")
        self.width = int(_select(cfg, "width", 640))
        self.height = int(_select(cfg, "height", 480))
        self.fps = int(_select(cfg, "fps", 30))
        self.color_format = str(_select(cfg, "color_format", "rgb8")).lower()
        self.warmup_frames = int(_select(cfg, "warmup_frames", 15))
        self._pipeline: Any | None = None
        self._last_image: np.ndarray | None = None

    @property
    def shape(self) -> tuple[int, int, int]:
        return (self.height, self.width, 3)

    def connect(self) -> None:
        try:
            import pyrealsense2 as rs  # type: ignore[import-untyped]
        except ImportError as exc:
            raise ImportError("使用 RealSense 相机需要安装 pyrealsense2") from exc

        # pyrealsense2 没有完整类型标注，这里用 Any 避免 IDE 误报。
        rs_any: Any = rs
        pipeline = rs_any.pipeline()
        config = rs_any.config()
        if self.serial_number:
            config.enable_device(self.serial_number)
        config.enable_stream(
            rs_any.stream.color,
            self.width,
            self.height,
            rs_any.format.rgb8,
            self.fps,
        )
        pipeline.start(config)
        self._pipeline = pipeline
        self._warmup()

    def read(self) -> np.ndarray:
        if self._pipeline is None:
            raise ConnectionError(f"RealSense 相机 {self.name} 尚未连接")

        frames = self._pipeline.poll_for_frames()
        if not frames:
            if self._last_image is not None:
                return self._last_image
            frames = self._pipeline.wait_for_frames()

        return self._image_from_frames(frames)

    def _warmup(self) -> None:
        if self._pipeline is None:
            return
        for _ in range(max(1, self.warmup_frames)):
            self._image_from_frames(self._pipeline.wait_for_frames())

    def _image_from_frames(self, frames: Any) -> np.ndarray:
        color_frame = frames.get_color_frame()
        if not color_frame:
            raise RuntimeError(f"RealSense 相机 {self.name} 没有返回彩色图像")

        image = np.asanyarray(color_frame.get_data())
        if self.color_format == "bgr8":
            image = image[..., ::-1]
        cached_image = image.copy()
        self._last_image = cached_image
        return cached_image

    def disconnect(self) -> None:
        if self._pipeline is not None:
            self._pipeline.stop()
            self._pipeline = None


class UR5GelloRobot(Robot):
    config_class = UR5GelloRobotConfig
    name = "ur5_gello"

    def __init__(self, config: UR5GelloRobotConfig) -> None:
        super().__init__(config)
        self.config = config
        self._cfg: DictConfig | None = None
        self._follower: Any | None = None
        self._safety_checker: Any | None = None
        self._names_cache: tuple[str, ...] | None = None
        # LeRobot 在 connect() 之前就会读取 robot.cameras 来创建数据集。
        self.cameras: dict[str, _RealSenseColorCamera] = (
            self._camera_configs() if config.use_yaml_cameras else {}
        )
        self._last_arm_joints: np.ndarray | None = None
        self._last_gripper_position = 0.0
        self._last_gripper_read_time = 0.0
        self._gripper_read_interval = 0.2

    @property
    def _config_path(self) -> Path:
        return _resolve_config_path(self.config.teleop_config_path)

    def _load_cfg(self) -> DictConfig:
        if self._cfg is None:
            self._cfg = load_config(self._config_path)
        return self._cfg

    @property
    def _names(self) -> tuple[str, ...]:
        if self._names_cache is None:
            self._names_cache = _joint_names(self._load_cfg())
        return self._names_cache

    @property
    def action_features(self) -> dict[str, type]:
        return {name: float for name in self._names}

    @property
    def observation_features(self) -> dict[str, Any]:
        features: dict[str, Any] = {name: float for name in self._names}
        for name, camera in self.cameras.items():
            features[name] = camera.shape
        return features

    @property
    def is_connected(self) -> bool:
        return self._follower is not None

    @property
    def is_calibrated(self) -> bool:
        return True

    def calibrate(self) -> None:
        return None

    def configure(self) -> None:
        return None

    def connect(self, calibrate: bool = True) -> None:
        if self.is_connected:
            return

        cfg = self._load_cfg()
        self._follower = build_follower(cfg)
        if self.config.use_yaml_safety:
            self._safety_checker = build_safety_checker(cfg)

        self._align_start_pose(cfg)

        if self.config.use_yaml_cameras and not self.cameras:
            self.cameras = self._camera_configs()
        for camera in self.cameras.values():
            camera.connect()

    def disconnect(self) -> None:
        for camera in self.cameras.values():
            camera.disconnect()

        if self._follower is not None and hasattr(self._follower, "close"):
            self._follower.close()
        self._follower = None
        self._safety_checker = None

    def _align_start_pose(self, cfg: DictConfig) -> None:
        if self._follower is None or self._safety_checker is None:
            return

        leader = build_leader(cfg)
        try:
            ok = align_to_leader(
                leader=leader,
                follower=self._follower,
                safety_checker=self._safety_checker,
                max_start_delta=float(cfg.startup.max_start_delta),
                alignment_steps=int(cfg.startup.alignment_steps),
                hz=float(cfg.startup.alignment_hz),
            )
            if not ok:
                raise RuntimeError("启动对齐已取消，停止 LeRobot 采集。")
            self._last_arm_joints = self._follower.read_arm().copy()
        finally:
            if hasattr(leader, "close"):
                leader.close()

    def get_observation(self) -> dict[str, Any]:
        if self._follower is None:
            raise ConnectionError("UR5GelloRobot 尚未连接")

        joints = self._follower.read_arm()
        self._last_arm_joints = joints.copy()
        gripper_position = self._read_gripper_position()
        state = np.concatenate([joints, np.array([gripper_position], dtype=float)])

        observation: dict[str, Any] = {
            name: float(value)
            for name, value in zip(self._names, state, strict=True)
        }
        for name, camera in self.cameras.items():
            observation[name] = camera.read()
        return observation

    def send_action(self, action: dict[str, Any]) -> dict[str, Any]:
        if self._follower is None:
            raise ConnectionError("UR5GelloRobot 尚未连接")

        target_state = np.array([float(action[name]) for name in self._names], dtype=float)
        target_joints = target_state[:6]
        gripper_position = float(np.clip(target_state[6], 0.0, 1.0))

        current_joints = self._last_arm_joints
        if current_joints is None:
            current_joints = self._follower.read_arm()
            self._last_arm_joints = current_joints.copy()
        safe_joints = target_joints
        if self._safety_checker is not None:
            safe_joints = self._safety_checker.limit_step(current_joints, target_joints)

        self._follower.command_arm(safe_joints)
        self._follower.command_gripper(gripper_position)
        self._last_arm_joints = safe_joints.copy()
        self._last_gripper_position = gripper_position

        sent_state = np.concatenate([safe_joints, np.array([gripper_position], dtype=float)])
        return {
            name: float(value)
            for name, value in zip(self._names, sent_state, strict=True)
        }

    def _read_gripper_position(self) -> float:
        if self._follower is None or not getattr(self._follower, "use_gripper", False):
            return self._last_gripper_position

        gripper = getattr(self._follower, "gripper", None)
        if gripper is None or not hasattr(gripper, "get_current_position"):
            return self._last_gripper_position

        now = time.monotonic()
        if now - self._last_gripper_read_time < self._gripper_read_interval:
            return self._last_gripper_position

        try:
            self._last_gripper_position = float(np.clip(gripper.get_current_position() / 255.0, 0.0, 1.0))
            self._last_gripper_read_time = now
            return self._last_gripper_position
        except Exception as exc:
            print(f"warning: 读取夹爪位置失败，使用上一次命令值: {exc}")
            return self._last_gripper_position

    def _camera_configs(self) -> dict[str, _RealSenseColorCamera]:
        cfg = self._load_cfg()
        if not bool(_select(cfg, "lerobot.realsense.enabled", False)):
            return {}

        cameras_cfg = _select(cfg, "lerobot.realsense.cameras", {})
        if not isinstance(cameras_cfg, DictConfig):
            return {}

        return {
            str(name): _RealSenseColorCamera(str(name), camera_cfg)
            for name, camera_cfg in cameras_cfg.items()
        }
