import time

import numpy as np


class UR5RTDEFollower:
    def __init__(
        self,
        robot_ip: str = "192.168.1.100",
        servo_velocity: float = 0.5,
        servo_acceleration: float = 0.5,
        servo_dt: float = 1.0 / 500,
        servo_lookahead_time: float = 0.2,
        servo_gain: int = 100,
        use_gripper: bool = True,
        gripper_port: int = 63352,
        gripper_command_interval: float = 0.05,
    ):
        import rtde_control
        import rtde_receive

        self.robot_ip = robot_ip
        self.servo_velocity = servo_velocity
        self.servo_acceleration = servo_acceleration
        self.servo_dt = servo_dt
        self.servo_lookahead_time = servo_lookahead_time
        self.servo_gain = servo_gain
        self.use_gripper = use_gripper
        self.gripper_port = gripper_port
        self.gripper_command_interval = gripper_command_interval
        self._last_gripper_command: int | None = None
        self._last_gripper_command_time = 0.0

        if self.use_gripper:
            from my_teleop.teleop.follower.robotiq_gripper import RobotiqGripper

            self.gripper = RobotiqGripper()
            self.gripper.connect(hostname=self.robot_ip, port=self.gripper_port)

        # 读取机械臂当前状态。
        self.receive = rtde_receive.RTDEReceiveInterface(self.robot_ip)

        # 发送机械臂目标状态。
        self.control = rtde_control.RTDEControlInterface(self.robot_ip)

    def read_arm(self) -> np.ndarray:
        joints = self.receive.getActualQ()
        if len(joints) != 6:
            raise ValueError(f"Expected 6 joints, got {len(joints)}")
        return np.array(joints)

    def num_arm_joints(self) -> int:
        return 6

    def command_arm(self, joints: np.ndarray) -> None:
        joints = np.asarray(joints, dtype=float)
        if joints.shape != (6,):
            raise ValueError(f"Expected joints shape (6,), got {joints.shape}")

        t_start = self.control.initPeriod()
        success = self.control.servoJ(
            joints.tolist(),
            self.servo_velocity,
            self.servo_acceleration,
            self.servo_dt,
            self.servo_lookahead_time,
            self.servo_gain,
        )
        if success is False:
            raise RuntimeError("UR5 servoJ 执行失败，请检查机器人是否处于可远程控制状态")
        self.control.waitPeriod(t_start)

    def command_gripper(self, gripper_position: float) -> None:
        if not self.use_gripper:
            return

        gripper_position = float(np.clip(gripper_position, 0.0, 1.0))
        command_position = int(round(gripper_position * 255))

        # 夹爪 socket 命令会阻塞主循环，变化足够明显且到达发送间隔后再下发。
        if (
            self._last_gripper_command is not None
            and abs(command_position - self._last_gripper_command) < 3
        ):
            return

        now = time.monotonic()
        if (
            self._last_gripper_command is not None
            and now - self._last_gripper_command_time < self.gripper_command_interval
        ):
            return

        success, actual_command_position = self.gripper.move(command_position, 255, 10)
        if success is False:
            raise RuntimeError("UR5 gripper move 执行失败，请检查机器人是否处于可远程控制状态")

        self._last_gripper_command = actual_command_position
        self._last_gripper_command_time = now

    def stop(self) -> None:
        try:
            self.control.servoStop()
        except Exception as exc:
            print(f"warning: UR5 servoStop 失败: {exc}")

    def close(self) -> None:
        self.stop()
        if hasattr(self.control, "disconnect"):
            self.control.disconnect()
        if hasattr(self.receive, "disconnect"):
            self.receive.disconnect()
