import sys
import time
from typing import Any, List, Tuple

import numpy as np

_WAIT_DISPLAY_HZ = 2.0


class _LiveBlockDisplay:
    def __init__(self) -> None:
        self._line_count = 0

    def update(self, lines: List[str]) -> None:
        if self._line_count > 0:
            sys.stdout.write(f"\033[{self._line_count}F")
        for line in lines:
            sys.stdout.write("\033[K" + line + "\n")
        self._line_count = len(lines)
        sys.stdout.flush()

    def finish(self, message: str = "") -> None:
        if self._line_count > 0:
            sys.stdout.write(f"\033[{self._line_count}F")
            for _ in range(self._line_count):
                sys.stdout.write("\033[K\n")
            sys.stdout.write(f"\033[{self._line_count}F")
            self._line_count = 0
        if message:
            print(message)
        sys.stdout.flush()


def _read_pose_delta(
    leader: Any,
    follower: Any,
) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    leader_state = leader.read()
    target_joints = leader_state[: follower.num_arm_joints()]
    current_joints = follower.read_arm()
    abs_delta = np.abs(target_joints - current_joints)
    return target_joints, current_joints, abs_delta


def _format_wait_table(
    target_joints: np.ndarray,
    current_joints: np.ndarray,
    abs_delta: np.ndarray,
    max_delta: float,
    max_start_delta: float,
) -> List[str]:
    ready = max_delta <= max_start_delta
    status = "已就绪" if ready else "未就绪，请调整姿态"
    lines = [
        "等待启动对齐（Ctrl+C 取消）",
        f"最大关节差: {max_delta:.4f} / {max_start_delta:.4f} rad  [{status}]",
        "关节 |   GELLO(rad) |     UR5(rad) |   差值(rad)",
        "-----+--------------+--------------+-------------",
    ]
    for joint_id, (target, current, delta) in enumerate(
        zip(target_joints, current_joints, abs_delta)
    ):
        marker = " *" if delta > max_start_delta else ""
        lines.append(
            f"  J{joint_id} | {target:12.4f} | {current:12.4f} | {delta:11.4f}{marker}"
        )
    return lines


def wait_for_start_pose(
    leader: Any,
    follower: Any,
    max_start_delta: float,
    hz: float = _WAIT_DISPLAY_HZ,
) -> bool:
    period = 1.0 / hz
    display = _LiveBlockDisplay()

    try:
        while True:
            loop_start = time.time()
            target_joints, current_joints, abs_delta = _read_pose_delta(
                leader, follower
            )
            max_delta = float(np.max(abs_delta))
            display.update(
                _format_wait_table(
                    target_joints,
                    current_joints,
                    abs_delta,
                    max_delta,
                    max_start_delta,
                )
            )
            if max_delta <= max_start_delta:
                display.finish("姿态已接近，开始启动对齐...")
                return True

            elapsed = time.time() - loop_start
            time.sleep(max(0.0, period - elapsed))
    except KeyboardInterrupt:
        display.finish()
        print("等待已取消。")
        return False


def align_to_leader(
    leader: Any,
    follower: Any,
    safety_checker: Any,
    max_start_delta: float,
    alignment_steps: int,
    hz: float,
) -> bool:
    if not wait_for_start_pose(leader, follower, max_start_delta):
        return False

    print("开始启动对齐...")
    period = 1.0 / hz
    for _ in range(alignment_steps):
        loop_start = time.time()

        target_joints, current_joints, _ = _read_pose_delta(leader, follower)
        safe_joints = safety_checker.limit_step(current_joints, target_joints)
        follower.command_arm(safe_joints)

        elapsed = time.time() - loop_start
        time.sleep(max(0.0, period - elapsed))

    print("启动对齐完成。")
    return True
