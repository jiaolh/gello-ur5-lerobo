import argparse
from pathlib import Path

from my_teleop.teleop.core.config_loader import (
    build_follower,
    build_leader,
    build_safety_checker,
    load_config,
)
from my_teleop.teleop.core.startup_alignment import align_to_leader
from my_teleop.teleop.core.teleop_loop import TeleopLoop

DEFAULT_CONFIG_PATH = Path(__file__).resolve().parents[1] / "config" / "ur5_gello.yaml"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="GELLO + UR5 关节空间遥操作")
    parser.add_argument(
        "--config",
        type=Path,
        default=DEFAULT_CONFIG_PATH,
        help="遥操作配置文件路径",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    cfg = load_config(args.config)

    print("配置加载成功")
    print(f"配置文件: {args.config}")
    print(f"GELLO 串口: {cfg.leader.port}")
    print(f"GELLO 关节 ID: {list(cfg.leader.joint_ids)}")
    print(f"GELLO 关节偏移: {list(cfg.leader.joint_offsets)}")
    print(f"GELLO 关节方向: {list(cfg.leader.joint_signs)}")
    print(f"UR5 IP: {cfg.follower.robot_ip}")
    print(f"启用夹爪: {cfg.follower.use_gripper}")
    print(f"控制频率: {cfg.control.hz} Hz")
    print(f"最大单步变化: {cfg.control.max_joint_delta}")
    print()
    print("即将开始遥操作。")
    print("请确认急停可用，UR5 周围安全，GELLO 和 UR5 初始姿态尽量接近。")
    input("确认安全后按 Enter 开始，或按 Ctrl+C 取消...")

    leader = build_leader(cfg)
    follower = build_follower(cfg)
    safety_checker = build_safety_checker(cfg)

    teleop_loop = TeleopLoop(
        leader=leader,
        follower=follower,
        safety_checker=safety_checker,
        hz=cfg.control.hz,
        diagnostics_interval=cfg.diagnostics.print_interval,
    )
    ok = align_to_leader(
        leader=leader,
        follower=follower,
        safety_checker=safety_checker,
        max_start_delta=cfg.startup.max_start_delta,
        alignment_steps=cfg.startup.alignment_steps,
        hz=cfg.startup.alignment_hz,
    )
    if not ok:
        teleop_loop.close()
        return
    teleop_loop.run()


if __name__ == "__main__":
    main()
