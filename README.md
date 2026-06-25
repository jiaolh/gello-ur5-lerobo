# my_teleop

GELLO + UR5 关节空间遥操作项目，同时支持直接遥操作和采集 LeRobot 格式数据集。

本项目把 GELLO 主手作为 `leader`，UR5 + Robotiq 夹爪作为 `follower`。普通遥操作流程使用 `teleop/` 下的代码直接读取 GELLO 的 Dynamixel 电机角度，并通过 RTDE `servoJ` 控制 UR5；数据采集流程通过 `lerobot_plugins/` 中的两个插件接入 LeRobot 的 `lerobot-record`。

## 功能

- GELLO 主手读取：支持 6 个手臂关节 + 1 个夹爪电机。
- UR5 跟随控制：通过 `ur-rtde` 控制 UR5 关节空间运动。
- Robotiq 夹爪控制：支持 0 到 1 的归一化开合命令。
- 启动安全对齐：启动时检查 GELLO 与 UR5 初始姿态差异，并可缓慢对齐。
- 单步限幅保护：限制单次循环允许的 UR5 关节变化量。
- LeRobot 数据采集：提供 `ur5_gello` robot 插件和 `my_gello` teleoperator 插件。
- RealSense 彩色图像采集：相机参数统一写在 YAML 配置中。

## 硬件与环境

硬件：

- GELLO 主手，使用 Dynamixel 电机。
- UR5 机械臂。
- Robotiq 夹爪，可选。
- Intel RealSense 相机，可选，用于 LeRobot 数据采集。

软件环境：

- Linux。
- Python 3.11。
- `uv`，用于创建虚拟环境和安装依赖。
- UR 控制器需要允许远程控制，并保证电脑能访问 UR5 的 IP。

## 项目结构

```text
.
├── config/
│   ├── ur5_gello.example.yaml    # 配置模板（提交到 Git）
│   └── ur5_gello.yaml              # 本地实际配置（已 gitignore，勿提交）
├── scripts/
│   ├── run_teleop.py               # 直接遥操作入口
│   └── calibrate_offsets.py        # GELLO 与 UR5 关节偏移标定工具
├── teleop/
│   ├── core/                       # 配置加载、启动对齐、遥操作主循环
│   ├── follower/                   # UR5 RTDE 与 Robotiq 夹爪控制
│   ├── leader/                     # GELLO / Dynamixel 读取
│   └── safety/                     # 单步关节限幅
└── lerobot_plugins/
    ├── lerobot_robot_ur5_gello/    # LeRobot robot 插件
    └── lerobot_teleoperator_my_gello/ # LeRobot teleoperator 插件
```

## 安装

建议使用 Python 3.11 虚拟环境。LeRobot 和 RealSense 相关包通常不建议使用 Python 3.13。

```bash
git clone https://github.com/YOUR_USERNAME/my_teleop.git
cd my_teleop
uv venv --python 3.11
source .venv/bin/activate
uv pip install -e .
```

如果需要使用 LeRobot 数据采集功能，再安装 LeRobot 和本项目插件：

```bash
uv pip install lerobot
uv pip install -e "./lerobot_plugins/lerobot_robot_ur5_gello[realsense]"
uv pip install -e "./lerobot_plugins/lerobot_teleoperator_my_gello"
```

如果已经安装过依赖，只是修改了本地插件代码，可以只重新安装插件元数据：

```bash
uv pip install --no-deps --no-build-isolation \
  -e "./lerobot_plugins/lerobot_robot_ur5_gello" \
  -e "./lerobot_plugins/lerobot_teleoperator_my_gello"
```

如果 `pyrealsense2` 不能通过 pip 安装，请先按 Intel RealSense SDK 的方式安装系统包。

## 配置

首次使用请从模板生成本地配置：

```bash
cp config/ur5_gello.example.yaml config/ur5_gello.yaml
```

然后编辑 `config/ur5_gello.yaml`，至少修改以下项：

- `leader.port`：GELLO 串口路径（`ls /dev/serial/by-id/` 查看）。
- `follower.robot_ip`：UR5 控制器 IP。
- `lerobot.realsense.cameras`：RealSense 相机序列号（如启用相机）。

`config/ur5_gello.yaml` 含本机硬件信息，已在 `.gitignore` 中忽略，不会提交到 GitHub。

常用配置项：

- `leader.port`：GELLO 的串口路径。
- `leader.joint_ids`：GELLO 手臂 6 个 Dynamixel 电机 ID。
- `leader.joint_offsets`：GELLO 与 UR5 对齐所需的关节零点偏移。
- `leader.joint_signs`：每个关节方向，取值为 `1` 或 `-1`。
- `leader.gripper`：GELLO 夹爪电机 ID、打开位置、闭合位置和读取间隔。
- `follower.robot_ip`：UR5 的 IP 地址。
- `follower.use_gripper`：是否启用 Robotiq 夹爪。
- `follower.servo`：UR5 `servoJ` 控制参数。
- `startup`：启动时姿态检查和缓慢对齐参数。
- `control.hz`：直接遥操作主循环频率。
- `control.max_joint_delta`：单次循环允许的最大关节变化量。
- `lerobot`：LeRobot 数据集关节命名、采集频率、GELLO 后台读取频率和 RealSense 相机配置。

## 标定

首次使用或更换 GELLO / UR5 姿态关系后，建议先标定关节偏移。

1. 修改 `config/ur5_gello.yaml` 中的串口、UR5 IP 和夹爪配置。
2. 将 GELLO 和 UR5 摆成尽量相同的姿态。
3. 保持 GELLO 夹爪完全打开。
4. 运行：

```bash
python scripts/calibrate_offsets.py --config config/ur5_gello.yaml
```

脚本会打印关节偏移和夹爪开合位置。将结果写回 `config/ur5_gello.yaml` 后再进行遥操作或数据采集。

## 直接遥操作

确认急停可用、UR5 周围安全、GELLO 和 UR5 初始姿态尽量接近后运行：

```bash
python scripts/run_teleop.py --config config/ur5_gello.yaml
```

启动流程：

1. 程序打印当前配置。
2. 手动确认安全后按 Enter。
3. 程序检查 GELLO 与 UR5 初始姿态差异。
4. 如差异在允许范围内，UR5 会缓慢对齐到 GELLO。
5. 进入实时遥操作循环。

停止时按 `Ctrl+C`，程序会尝试调用 `servoStop()` 并断开 RTDE 连接。

## 采集 LeRobot 格式数据

本项目提供两个 LeRobot 插件：

- `--robot.type=ur5_gello`：UR5 + Robotiq + RealSense follower。
- `--teleop.type=my_gello`：GELLO leader。

示例命令：

```bash
lerobot-record \
  --dataset.repo_id=YOUR_USERNAME/ur5_gello_demo \
  --dataset.single_task="使用 GELLO 遥操 UR5 完成一次示教任务" \
  --dataset.fps=30 \
  --dataset.num_episodes=10 \
  --dataset.episode_time_s=30 \
  --dataset.reset_time_s=15 \
  --dataset.push_to_hub=false \
  --robot.type=ur5_gello \
  --robot.teleop_config_path=config/ur5_gello.yaml \
  --teleop.type=my_gello \
  --teleop.teleop_config_path=config/ur5_gello.yaml
```

RealSense 相机写在 `config/ur5_gello.yaml` 的 `lerobot.realsense` 段。模板中提供了 `front` 相机示例，可按需要取消注释 `wrist` 等相机。

采集得到的数据集通常体积较大，不建议直接提交到 GitHub。可以保存在本地，或按 LeRobot / Hugging Face Hub 的流程上传到数据集仓库。

## 安全注意事项

- 每次运行前确认 UR5 急停、保护停止和远程控制状态正常。
- 确认 UR5 工作空间内没有人员或障碍物。
- 启动前尽量让 GELLO 和 UR5 姿态接近，避免启动对齐动作过大。
- 首次调试时建议降低 `control.hz`、`control.max_joint_delta` 和 `follower.servo.gain`。
- 如果出现异常运动、RTDE 报错或夹爪阻塞，请立即停止程序并检查配置。
- 不要在不了解含义的情况下增大 `startup.max_start_delta` 或 `control.max_joint_delta`。

## 常见问题

### 找不到 GELLO 串口

检查 `config/ur5_gello.yaml` 中的 `leader.port` 是否和当前设备一致：

```bash
ls /dev/serial/by-id/
```

### UR5 连接失败

检查以下项目：

- `follower.robot_ip` 是否正确。
- 电脑是否能 ping 通 UR5。
- UR 控制器是否处于可远程控制状态。
- RTDE 端口是否被其他程序占用。

### RealSense 无法打开

检查以下项目：

- 是否安装了 `pyrealsense2` 或 Intel RealSense SDK。
- 相机序列号是否与 `lerobot.realsense.cameras` 中的配置一致。
- USB 是否为足够带宽的接口。

## 致谢

本项目在以下开源项目的基础上构建，特此致谢：

- [GELLO](https://wuphilipp.github.io/gello_site/) — 关节读取与偏移标定思路参考了 [gello_software](https://github.com/wuphilipp/gello_software)。
- [LeRobot](https://github.com/huggingface/lerobot) — 数据采集通过 LeRobot 插件接口实现。
- [py_robotiq_gripper](https://github.com/githubuser0xFFFF/py_robotiq_gripper) — Robotiq 夹爪控制代码参考/改编自该项目。

若你在研究中使用了 GELLO，建议同时引用其论文：

```bibtex
@article{wu2023gello,
  title={GELLO: A General, Low-Cost, and Intuitive Teleoperation Framework for Robot Manipulators},
  author={Wu, Philip and others},
  journal={arXiv preprint arXiv:2309.13037},
  year={2023}
}
```

## 许可证

本项目采用 [MIT License](LICENSE)。
