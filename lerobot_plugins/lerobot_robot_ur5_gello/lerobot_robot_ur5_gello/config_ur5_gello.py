from dataclasses import dataclass
from pathlib import Path

from lerobot.robots import RobotConfig


@RobotConfig.register_subclass("ur5_gello")
@dataclass
class UR5GelloRobotConfig(RobotConfig):
    """UR5 从手机器人插件配置。"""

    teleop_config_path: str = "config/ur5_gello.yaml"
    use_yaml_cameras: bool = True
    use_yaml_safety: bool = True

    def __post_init__(self) -> None:
        super().__post_init__()
        if self.id is None:
            self.id = "ur5_gello"
        if self.calibration_dir is None:
            # 本插件不使用 LeRobot 电机标定文件，但基类需要一个可写目录。
            self.calibration_dir = Path(".cache/lerobot_calibration/robots")
