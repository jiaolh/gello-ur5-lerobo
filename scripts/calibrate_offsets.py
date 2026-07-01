import argparse
import time
from pathlib import Path

import numpy as np

from my_teleop.teleop.core.config_loader import (
    build_follower,
    build_leader,
    load_config,
)
from my_teleop.teleop.follower.ur5_rtde_follower import UR5RTDEFollower
from my_teleop.teleop.leader.gello_leader import GelloLeader

# 与 gello_get_offset.py 一致：夹爪完全打开时读一次，再减去固定偏移。
GRIPPER_OPEN_OFFSET_DEG = 0.2
GRIPPER_CLOSE_OFFSET_DEG = 42.0
DEFAULT_UR_START_JOINTS = (0.0, -1.57, 1.57, -1.57, -1.57, 0.0)
DEFAULT_VERIFY_TOLERANCE = 0.01
DEFAULT_CONFIG_PATH = (
    Path(__file__).resolve().parents[1] / "config" / "ur5_gello.yaml"
)


def compute_offsets(
    raw_joints: np.ndarray,
    ur5_joints: np.ndarray,
    signs: np.ndarray,
) -> np.ndarray:
    signs = np.asarray(signs, dtype=float)
    return raw_joints - ur5_joints / signs


def snap_to_half_pi(offsets: np.ndarray) -> np.ndarray:
    return np.round(offsets / (np.pi / 2)) * (np.pi / 2)


def calibrate_gello_gripper(leader: GelloLeader) -> None:
    present_deg = float(np.rad2deg(leader.read_raw()[-1]))
    open_deg = present_deg - GRIPPER_OPEN_OFFSET_DEG
    close_deg = present_deg - GRIPPER_CLOSE_OFFSET_DEG

    print(f"gripper open_position: {open_deg:.2f}")
    print(f"gripper close_position: {close_deg:.2f}")


def calibrate_ur5_gripper(follower: UR5RTDEFollower) -> None:
    if not follower.use_gripper:
        return

    follower.command_gripper(0.0)
    time.sleep(1.0)
    follower.command_gripper(1.0)
    time.sleep(1.0)
    follower.command_gripper(0.0)
    time.sleep(1.0)


def parse_start_joints(values: list[float]) -> np.ndarray:
    if len(values) != 6:
        raise ValueError(f"--start-joints 需要 6 个关节角，实际收到 {len(values)} 个")
    return np.asarray(values, dtype=float)


def format_offset_exprs(offsets: np.ndarray) -> str:
    k_values = np.round(offsets / (np.pi / 2)).astype(int)
    return "[" + ", ".join(f"{k} * np.pi / 2" for k in k_values) + "]"


def print_calibration_diagnostics(
    raw_joints: np.ndarray,
    target_joints: np.ndarray,
    decoded_joints: np.ndarray,
    errors: np.ndarray,
) -> None:
    print("关节 | GELLO原始(rad) | 参考UR5(rad) | 解码后(rad) | 误差(deg)")
    print("-----+---------------+-------------+------------+----------")
    for joint_index, (raw, target, decoded, error) in enumerate(
        zip(raw_joints, target_joints, decoded_joints, errors),
        start=1,
    ):
        print(
            f" J{joint_index}  | {raw:13.4f} | {target:11.4f} | "
            f"{decoded:10.4f} | {np.rad2deg(error):8.3f}"
        )


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG_PATH)
    parser.add_argument(
        "--start-joints",
        type=float,
        nargs=6,
        default=DEFAULT_UR_START_JOINTS,
        help="GELLO 当前参考姿态对应的 UR5 关节角，单位 rad",
    )
    parser.add_argument(
        "--use-current-ur5",
        action="store_true",
        help="使用当前 UR5 实际关节角作为参考姿态，而不是 --start-joints",
    )
    parser.add_argument(
        "--verify-tolerance",
        type=float,
        default=DEFAULT_VERIFY_TOLERANCE,
        help="吸附到 pi/2 后的提示阈值，单位 rad",
    )
    args = parser.parse_args()
    cfg = load_config(args.config)

    leader = build_leader(cfg)
    follower = build_follower(cfg)

    try:
        if args.use_current_ur5:
            input("请把 GELLO 和 UR5 摆成尽量相同的姿态，并保持夹爪完全打开，按回车键开始标定。")
        else:
            input(
                "请把 GELLO 摆到 --start-joints 对应的参考姿态，并保持夹爪完全打开，按回车键开始标定。"
            )

        # 与原版 gello_get_offset.py 一样，先预热几次，避免刚打开串口时读到不稳定数据。
        for _ in range(10):
            leader.read_raw()
            time.sleep(0.01)

        raw_joints = leader.read_raw()[:6]
        if args.use_current_ur5:
            target_joints = follower.read_arm()
        else:
            target_joints = parse_start_joints(list(args.start_joints))
        signs = cfg.leader.joint_signs

        computed_offsets = compute_offsets(raw_joints, target_joints, signs)
        snapped_offsets = snap_to_half_pi(computed_offsets)
        decoded = (raw_joints - snapped_offsets) * np.asarray(signs, dtype=float)
        errors = np.abs(decoded - target_joints)
        max_error = float(np.max(errors))

        print(f"标定偏移量: {format_offset_exprs(snapped_offsets)}")
        print(f"最大吸附误差: {max_error:.4f} rad / {np.rad2deg(max_error):.3f} deg")
        print_calibration_diagnostics(raw_joints, target_joints, decoded, errors)

        if max_error <= args.verify_tolerance:
            print("标定校验通过")
        else:
            print(
                "警告: 标定偏移已输出，但 GELLO 姿态和参考姿态误差较大；"
                "请检查参考姿态、joint_signs 或手动姿态是否一致。"
            )

        calibrate_gello_gripper(leader)
        calibrate_ur5_gripper(follower)
    finally:
        leader.close()
        follower.close()


if __name__ == "__main__":
    main()
