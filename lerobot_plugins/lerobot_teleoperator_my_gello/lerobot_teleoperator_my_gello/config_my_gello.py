from dataclasses import dataclass
from pathlib import Path

from lerobot.teleoperators.config import TeleoperatorConfig


@TeleoperatorConfig.register_subclass("my_gello")
@dataclass
class MyGelloTeleoperatorConfig(TeleoperatorConfig):
    """GELLO 主手插件配置。"""

    teleop_config_path: str = "config/ur5_gello.yaml"

    def __post_init__(self) -> None:
        if self.id is None:
            self.id = "my_gello"
        if self.calibration_dir is None:
            # GELLO 偏移仍由 ur5_gello.yaml 管理，这里只满足 LeRobot 的目录约定。
            self.calibration_dir = Path(".cache/lerobot_calibration/teleoperators")
