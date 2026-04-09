# Project 2 运行与复现说明

本文档说明当前仓库中 `proj2` 的常用运行入口，以及如何复现最新的最终验证结果 `tmp/proj2_runs/full_check_gui_04_1`。

## 1. 环境前提

运行前需要满足以下条件：

- 已安装 ROS 2 Jazzy。
- 当前工作区已经完成构建。
- 终端已经进入仓库根目录。

如果你是在 Docker 中运行，先进入容器，再执行后续命令即可；本文档不依赖某一台机器上的固定容器名或挂载路径。

## 2. 常用运行入口

普通项目运行：

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch ee4308_bringup proj2_sim.launch.py
```

如果图形环境需要额外的 `LibGL` 兼容参数：

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch ee4308_bringup proj2_sim.launch.py libgl:=True
```

## 3. 复现最终验证结果

当前仓库里最直接的完整验证入口是 [`tools/run_proj2_full_check.py`](../tools/run_proj2_full_check.py)。它会自动：

1. 录制对齐后的误差数据；
2. 可选录制 `drone_plan.csv`；
3. 启动 `proj2` 仿真；
4. 在结束后自动生成绘图和 `summary.txt`。

复现 `full_check_gui_04_1` 的推荐命令：

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
python3 tools/run_proj2_full_check.py \
  --run-name full_check_gui_04_1 \
  --duration 200 \
  --record-plan
```

如果你的图形环境存在 OpenGL 兼容问题，可改为：

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
python3 tools/run_proj2_full_check.py \
  --run-name full_check_gui_04_1 \
  --duration 200 \
  --record-plan \
  --libgl
```

如果只想批量跑而不打开 GUI，可以使用：

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
python3 tools/run_proj2_full_check.py \
  --run-name full_check_headless_01 \
  --duration 200 \
  --record-plan \
  --headless \
  --libgl
```

## 4. 结果目录说明

`full_check` 运行完成后，输出目录通常包含：

- `aligned_pose_error.csv`
- `drone_plan.csv`（仅在开启 `--record-plan` 时生成）
- `plots/summary.txt`
- `plots/trajectory_3d.png`
- `plots/position_vs_time.png`
- `plots/error_vs_time.png`

当前仓库中，最新最终验证目录为：

- [`tmp/proj2_runs/full_check_gui_04_1`](/home/liuyi/projects/ee4308_proj2/tmp/proj2_runs/full_check_gui_04_1)

它对应的摘要文件是：

- [`summary.txt`](/home/liuyi/projects/ee4308_proj2/tmp/proj2_runs/full_check_gui_04_1/plots/summary.txt)

## 5. 命名建议

为了方便后续写报告和对比实验，建议统一采用下面的命名方式：

```bash
python3 tools/run_proj2_full_check.py \
  --run-name full_check_gui_05 \
  --duration 200 \
  --record-plan
```

如果是无界面运行，则在名字中明确体现：

```bash
python3 tools/run_proj2_full_check.py \
  --run-name full_check_headless_02 \
  --duration 200 \
  --record-plan \
  --headless \
  --libgl
```

这样只看目录名，就能区分 GUI 运行和 headless 运行。
