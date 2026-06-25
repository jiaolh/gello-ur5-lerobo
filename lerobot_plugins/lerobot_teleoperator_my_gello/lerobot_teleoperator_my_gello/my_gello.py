from __future__ import annotations

import threading
import time
from pathlib import Path
from typing import Any

import numpy as np
from omegaconf import DictConfig, OmegaConf

from lerobot.teleoperators.teleoperator import Teleoperator
from my_teleop.teleop.core.config_loader import build_leader, load_config

from .config_my_gello import MyGelloTeleoperatorConfig

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


class MyGelloTeleoperator(Teleoperator):
    config_class = MyGelloTeleoperatorConfig
    name = "my_gello"

    def __init__(self, config: MyGelloTeleoperatorConfig) -> None:
        super().__init__(config)
        self.config = config
        self._cfg: DictConfig | None = None
        self._leader: Any | None = None
        self._latest_action: dict[str, float] | None = None
        self._action_lock = threading.Lock()
        self._reader_stop = threading.Event()
        self._reader_thread: threading.Thread | None = None
        self._reader_error: Exception | None = None
        self._read_period = 1.0 / 30.0

    @property
    def _config_path(self) -> Path:
        return _resolve_config_path(self.config.teleop_config_path)

    def _load_cfg(self) -> DictConfig:
        if self._cfg is None:
            self._cfg = load_config(self._config_path)
        return self._cfg

    @property
    def _names(self) -> tuple[str, ...]:
        return _joint_names(self._load_cfg())

    @property
    def action_features(self) -> dict[str, type]:
        return {name: float for name in self._names}

    @property
    def feedback_features(self) -> dict[str, type]:
        return {}

    @property
    def is_connected(self) -> bool:
        return self._leader is not None

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
        self._read_period = 1.0 / float(_select(cfg, "lerobot.teleop_read_hz", _select(cfg, "control.hz", 30)))
        self._leader = build_leader(cfg)
        self._latest_action = self._read_leader_action()
        self._reader_stop.clear()
        self._reader_thread = threading.Thread(
            target=self._reader_loop,
            name="gello-teleop-reader",
            daemon=True,
        )
        self._reader_thread.start()

    def disconnect(self) -> None:
        if self._reader_thread is not None:
            self._reader_stop.set()
            self._reader_thread.join(timeout=1.0)
            self._reader_thread = None
        if self._leader is not None and hasattr(self._leader, "close"):
            self._leader.close()
        self._leader = None
        self._latest_action = None

    def get_action(self) -> dict[str, float]:
        if self._leader is None:
            raise ConnectionError("MyGelloTeleoperator 尚未连接")

        with self._action_lock:
            if self._latest_action is not None:
                return dict(self._latest_action)

        if self._reader_error is not None:
            raise RuntimeError("GELLO 后台读取失败") from self._reader_error

        raise RuntimeError("GELLO 尚未读取到有效动作")

    def _read_leader_action(self) -> dict[str, float]:
        if self._leader is None:
            raise ConnectionError("MyGelloTeleoperator 尚未连接")

        state = np.asarray(self._leader.read(), dtype=float)
        if state.shape != (7,):
            raise ValueError(f"GELLO 状态必须是 7 维，当前形状为 {state.shape}")
        return {
            name: float(value)
            for name, value in zip(self._names, state, strict=True)
        }

    def _reader_loop(self) -> None:
        while not self._reader_stop.is_set():
            loop_start = time.perf_counter()
            try:
                action = self._read_leader_action()
                with self._action_lock:
                    self._latest_action = action
                self._reader_error = None
            except Exception as exc:
                self._reader_error = exc
                print(f"warning: GELLO 后台读取失败，继续使用上一帧动作: {exc}")

            elapsed = time.perf_counter() - loop_start
            time.sleep(max(0.0, self._read_period - elapsed))

    def send_feedback(self, feedback: dict[str, Any]) -> None:
        return None
