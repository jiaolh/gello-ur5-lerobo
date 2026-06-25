import argparse
import time

import numpy as np

from my_teleop.teleop.core.config_loader import (
    load_config,
    build_leader,
    build_follower,
)

# 与 gello_get_offset.py 一致：夹爪完全打开时读一次，再减去固定偏移。
GRIPPER_OPEN_OFFSET_DEG = 0.2
GRIPPER_CLOSE_OFFSET_DEG = 42.0


def compute_offsets(
    raw_joints: np.ndarray,
    ur5_joints: np.ndarray,
    signs: np.ndarray,
) -> np.ndarray:
    signs = np.asarray(signs, dtype=float)
    return raw_joints - ur5_joints / signs


def snap_to_half_pi(offsets: np.ndarray) -> np.ndarray:
    return np.round(offsets / (np.pi / 2)) * (np.pi / 2)


def calibrate_gello_gripper(leader) -> None:
    present_deg = float(np.rad2deg(leader.read_raw()[-1]))
    open_deg = present_deg - GRIPPER_OPEN_OFFSET_DEG
    close_deg = present_deg - GRIPPER_CLOSE_OFFSET_DEG

    print(f"gripper open_position: {open_deg:.2f}")
    print(f"gripper close_position: {close_deg:.2f}")


def calibrate_ur5_gripper(follower) -> None:
    if not follower.use_gripper:
        return

    follower.command_gripper(0.0)
    time.sleep(1.0)
    follower.command_gripper(1.0)
    time.sleep(1.0)
    follower.command_gripper(0.0)
    time.sleep(1.0)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", type=str, default="my_teleop/config/ur5_gello.yaml")
    args = parser.parse_args()
    cfg = load_config(args.config)

    leader = build_leader(cfg)
    follower = build_follower(cfg)

    try:
        input("请把 GELLO 和 UR5 摆成尽量相同的姿态，并保持夹爪完全打开，按回车键开始标定。")

        raw_joints = leader.read_raw()[:6]
        ur5_joints = follower.read_arm()
        signs = cfg.leader.joint_signs

        computed_offsets = compute_offsets(raw_joints, ur5_joints, signs)
        snapped_offsets = snap_to_half_pi(computed_offsets)
        decoded = (raw_joints - snapped_offsets) * np.asarray(signs, dtype=float)

        if np.max(np.abs(decoded - ur5_joints)) < 0.01:
            k = np.round(snapped_offsets / (np.pi / 2))
            print(f"标定成功，偏移量: {k} * (np.pi / 2)")
            calibrate_gello_gripper(leader)
            calibrate_ur5_gripper(follower)
        else:
            print("标定失败")
    finally:
        leader.close()
        follower.close()


if __name__ == "__main__":
    main()
