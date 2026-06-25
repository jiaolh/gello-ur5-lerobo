# LeRobot 插件说明

这里的插件只包装现有 `my_teleop` 遥操类，不修改 `teleop/` 和 `scripts/run_teleop.py`。

## Python 版本

建议使用项目自带的 Python 3.11 虚拟环境。LeRobot 和 RealSense 的 Python 包通常不建议使用 Python 3.13。

```bash
cd /home/birlab/my_teleop
source .venv/bin/activate
python --version
```

## 安装

```bash
uv pip install --python ./.venv/bin/python -e .
uv pip install --python ./.venv/bin/python lerobot
uv pip install --python ./.venv/bin/python -e "./lerobot_plugins/lerobot_robot_ur5_gello[realsense]"
uv pip install --python ./.venv/bin/python -e "./lerobot_plugins/lerobot_teleoperator_my_gello"
```

如果依赖已经安装过，只需要重新安装本地插件元数据，可以不联网执行：

```bash
uv pip install --python ./.venv/bin/python --no-deps --no-build-isolation \
  -e "./lerobot_plugins/lerobot_robot_ur5_gello" \
  -e "./lerobot_plugins/lerobot_teleoperator_my_gello"
```

如果 `pyrealsense2` 不能通过 pip 安装，请按 Intel RealSense SDK 的方式安装系统包后再重试。

## 采集

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

RealSense 参数统一写在 `config/ur5_gello.yaml` 的 `lerobot.realsense` 段。
