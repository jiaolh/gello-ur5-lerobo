import time
from typing import Any, Dict

import numpy as np

_SLEEP_SPIN_THRESHOLD = 0.001


def _sleep_until(deadline: float) -> None:
    """睡到指定时间点，最后 1ms 用短暂自旋减少 Linux sleep 超时误差。"""
    while True:
        remaining = deadline - time.perf_counter()
        if remaining <= 0:
            return
        if remaining > _SLEEP_SPIN_THRESHOLD:
            time.sleep(remaining - _SLEEP_SPIN_THRESHOLD)
            continue
        while time.perf_counter() < deadline:
            pass
        return


class TeleopLoop:
    def __init__(
        self,
        leader: Any,
        follower: Any,
        safety_checker: Any,
        hz: float,
        diagnostics_interval: float = 1.0,
    ) -> None:
        self.leader = leader
        self.follower = follower
        self.safety_checker = safety_checker
        self.hz = hz
        self.period = 1.0 / hz
        self.diagnostics_interval = diagnostics_interval
        self._last_diagnostics_time = time.perf_counter()
        self._loop_count = 0

    def run_once(self) -> Dict[str, Any]:
        leader_state = self.leader.read()
        target_joints = leader_state[: self.follower.num_arm_joints()]

        current_joints = self.follower.read_arm()
        safe_joints = self.safety_checker.limit_step(current_joints, target_joints)

        max_error = float(np.max(np.abs(target_joints - current_joints)))
        limited = not np.allclose(safe_joints, target_joints)

        gripper_position = leader_state[-1]

        self.follower.command_arm(safe_joints)
        self.follower.command_gripper(gripper_position)

        return {
            "max_error": max_error,
            "limited": limited,
        }

    def run(self) -> None:
        next_deadline = time.perf_counter()
        try:
            while True:
                info = self.run_once()
                self._loop_count += 1

                now = time.perf_counter()
                diagnostics_elapsed = now - self._last_diagnostics_time
                if diagnostics_elapsed >= self.diagnostics_interval:
                    actual_hz = self._loop_count / diagnostics_elapsed
                    print(
                        f"hz={actual_hz:.1f}, "
                        f"max_error={info['max_error']:.4f} rad, "
                        f"limited={info['limited']}, "
                    )
                    self._loop_count = 0
                    self._last_diagnostics_time = now

                next_deadline += self.period
                if now - next_deadline > self.period:
                    # 如果硬件阻塞已经严重超期，重置 deadline，避免连续追帧。
                    next_deadline = now + self.period
                _sleep_until(next_deadline)
        except KeyboardInterrupt:
            print("\n遥操作循环已停止。")
        finally:
            self.close()

    def close(self) -> None:
        if hasattr(self.follower, "close"):
            self.follower.close()
        if hasattr(self.leader, "close"):
            self.leader.close()