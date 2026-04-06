# Project 2 Estimator 中文定稿记录

## 1. 当前共享版本的定位

这个版本已经按“共享给队友阅读”的目标做过收束：

- 保留最终 `proj2` 源码、参数和复现实验脚本。
- 保留最终报告草稿和最终一次完整验证结果。
- 删除大批中间 sweep 结果、个人工作笔记和编辑器缓存。

当前仓库中与 `proj2` 直接相关的核心文件如下：

- 行为与控制：
  - [behavior.cpp](/home/liuyi/projects/ee4308_proj2/src/ee4308_drone/src/behavior.cpp)
  - [controller.cpp](/home/liuyi/projects/ee4308_proj2/src/ee4308_drone/src/controller.cpp)
- 状态估计：
  - [estimator.hpp](/home/liuyi/projects/ee4308_proj2/src/ee4308_drone/include/ee4308_drone/estimator.hpp)
  - [estimator.cpp](/home/liuyi/projects/ee4308_proj2/src/ee4308_drone/src/estimator.cpp)
- 最终参数：
  - [proj2.yaml](/home/liuyi/projects/ee4308_proj2/src/ee4308_bringup/params/proj2.yaml)
  - [proj2_gt.yaml](/home/liuyi/projects/ee4308_proj2/src/ee4308_bringup/params/proj2_gt.yaml)
- 复现与分析工具：
  - [record_drone_alignment.py](/home/liuyi/projects/ee4308_proj2/tools/record_drone_alignment.py)
  - [record_drone_plan.py](/home/liuyi/projects/ee4308_proj2/tools/record_drone_plan.py)
  - [plot_drone_bag.py](/home/liuyi/projects/ee4308_proj2/tools/plot_drone_bag.py)
  - [run_proj2_param_sweep.py](/home/liuyi/projects/ee4308_proj2/tools/run_proj2_param_sweep.py)
  - [run_proj2_full_check.py](/home/liuyi/projects/ee4308_proj2/tools/run_proj2_full_check.py)
  - [summarize_proj2_logs.py](/home/liuyi/projects/ee4308_proj2/tools/summarize_proj2_logs.py)

共享仓库只保留一个最终验证目录：

- [full_check_gui_04](/home/liuyi/projects/ee4308_proj2/tmp/proj2_runs/full_check_gui_04)

## 2. 最终 baseline 设计

最终 estimator 仍采用分轴滤波设计：

- `Xx = [x, vx]^T`
- `Xy = [y, vy]^T`
- `Xz = [z, vz, b_baro]^T`
- `Xa = [yaw, yaw_rate]^T`

最终保留的核心改进如下：

1. `z` 轴 sonar gating  
   仅在低空且创新较小时接受 sonar，防止高空错误量测破坏高度估计。
2. barometer bias augmentation  
   用 `b_baro` 显式建模 barometer 偏置。
3. Joseph form covariance update  
   提升数值稳定性。
4. odom 时间戳统一  
   `/drone/odom` 使用状态更新时间，减少表观延迟。
5. GPS velocity pseudo-measurement  
   用 GPS 相邻位置差分出的平面速度减轻 `x/y` lag。
6. GPS forward compensation  
   对滞后的 GPS 位置做前推补偿再修正。

不再属于最终共享主线的内容：

- 大批调参中间结果目录
- 个人过程笔记
- 编辑器和 Python 缓存文件

## 3. 最终参数与结论

当前最终 baseline 以 [proj2.yaml](/home/liuyi/projects/ee4308_proj2/src/ee4308_bringup/params/proj2.yaml) 为准。

关键参数结论：

- `var_sonar = 0.03`
- `var_gps_x = var_gps_y = 0.15`
- `var_imu_x = var_imu_y = 1.5`
- `gps_forward_compensation_enable = true`
- `gps_forward_compensation_max_dt = 0.5`
- `gps_velocity_alpha = 0.9`
- `gps_velocity_variance_scale = 0.1`
- `gps_velocity_min_variance = 0.04`

这些选择对应的总判断是：

- `z` 轴问题主要靠 sonar gating 和 barometer bias augmentation 解决。
- 平面方向的主要改进来自 GPS 伪速度量测和 GPS 前向补偿。
- 最终参数不是追求单次最好，而是追求更稳定的整体表现。

[proj2_gt.yaml](/home/liuyi/projects/ee4308_proj2/src/ee4308_bringup/params/proj2_gt.yaml) 仅用于隔离 `behavior/controller` 问题，不作为最终 estimator 实验参数文件。

## 4. 最终证据与报告使用方式

共享仓库中只保留最终一次完整验证结果：

- [summary.txt](/home/liuyi/projects/ee4308_proj2/tmp/proj2_runs/full_check_gui_04/plots/summary.txt)
- [trajectory_3d.png](/home/liuyi/projects/ee4308_proj2/tmp/proj2_runs/full_check_gui_04/plots/trajectory_3d.png)
- [position_vs_time.png](/home/liuyi/projects/ee4308_proj2/tmp/proj2_runs/full_check_gui_04/plots/position_vs_time.png)
- [error_vs_time.png](/home/liuyi/projects/ee4308_proj2/tmp/proj2_runs/full_check_gui_04/plots/error_vs_time.png)
- [drone_plan.csv](/home/liuyi/projects/ee4308_proj2/tmp/proj2_runs/full_check_gui_04/drone_plan.csv)

当前报告草稿已经以这个目录为主证据，因此共享清理版不再保留旧 sweep 原始输出。旧实验结论只保留为文字总结：

- sonar gating 解决了高空 sonar 失真问题。
- barometer bias augmentation 稳定了 `z` 轴。
- GPS 伪速度量测能够减轻平面 lag，但过强设置会更差。
- GPS forward compensation 在最终设置下优于关闭。
- 软门控尝试没有带来净收益，因此没有进入最终 runtime 主线。

如果后续需要补做对照实验，应直接重新运行脚本生成新结果，而不是依赖已删除的历史目录。

## 5. 对照 proj2.md 的实现完整度

对照 [proj2.md](/home/liuyi/projects/ee4308_proj2/docs/proj2.md) 中 estimator 部分，当前实现状态如下：

| 要求 | 状态 |
| --- | --- |
| `callbackSubIMU_()` | 已实现 |
| `callbackSubGPS_()` | 已实现 |
| `getECEF_()` | 已实现 |
| `callbackSubMagnetic_()` | 已实现 |
| `callbackSubSonar_()` | 已实现 |
| `callbackSubBaro_()` | 已实现 |

另外，以下内容属于高于最低要求的改进：

- Joseph form 协方差更新
- baro bias augmentation
- sonar gating
- GPS velocity pseudo-measurement
- GPS forward compensation

因此从实现完整度上看，当前版本已经足够进入最终报告与团队共享阶段。
