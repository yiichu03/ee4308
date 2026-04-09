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

## 6. 2026-04-07 报告 estimator 小节落稿

已将 estimator 正式报告内容写入 [main.tex](/home/liuyi/projects/ee4308_proj2/docs/main.tex) 的 `\section{Estimator}`。

这次落稿遵守了两个约束：

- 只写最终代码里真正实现的内容，不写未进入最终版本的想法。
- 句子尽量短，重点解释“为什么这样做”和“这样做如何改善结果”。

当前报告小节结构为：

1. `Algorithm Examination`
   - 分轴状态定义
   - IMU prediction
   - GPS / sonar / barometer / magnetometer correction model
   - Joseph form 协方差更新
2. `Improvements`
   - sonar gating
   - barometer bias state
   - GPS velocity correction
   - GPS forward compensation
3. `Parameter Tuning`
   - 只保留最终参数和对应理由
4. `Final Performance`
   - 使用最终验证实验的 MAE / RMSE

这样写的好处是：

- 与 `proj2.md` 的 estimator 要求直接对应；
- 与老师在 [email.md](/home/liuyi/projects/ee4308_proj2/docs/email.md) 里强调的“写 why / how、给 equations、说明调参理由”一致；
- 不会因为写了未实现内容而被扣分。

## 7. 2026-04-07 报告可直接使用的实验对比

这次又检查了当前最终验证目录和旧提交 `a884ead6afcaef09e847bc444111579beb86752a` 中保留下来的实验记录，得到两个可以直接写进报告的结论。

### 7.1 最适合放进报告的最终图

当前最新最终验证目录为：

- [full_check_gui_04_1](/home/liuyi/projects/ee4308_proj2/tmp/proj2_runs/full_check_gui_04_1)

其中最值得放进报告的是：

1. `position_vs_time.png`
   - 最适合说明 estimator 跟完整个任务过程；
   - 可以直接看出 `z` 轴在起飞、巡航和降落时都比较贴近 ground truth；
   - 也能看出主要剩余误差在 `x/y`。
2. `error_vs_time.png`
   - 最适合说明误差分布；
   - 可以直接支持“`z` 和 yaw 已经较好，主要残差集中在平面方向”。

`trajectory_3d.png` 也可以放，但它更像总览图，定量信息不如上面两张直接。如果版面有限，优先放前两张。

### 7.2 最适合做前后对比的旧实验

旧提交文档里最干净、最容易解释的一组前后对比，不是平面方向，而是 `z` 轴问题。

旧文档中记录的相关命令为：

1. 早期 baseline sweep

```bash
source /opt/ros/jazzy/setup.bash
cd /ws/ee4308
python3 tools/run_proj2_param_sweep.py --duration 40 --output-root tmp/proj2_param_sweeps/run1
```

2. 加入 `z` 轴修正后的对比实验

```bash
source /opt/ros/jazzy/setup.bash
cd /ws/ee4308
python3 tools/run_proj2_param_sweep.py \
  --no-default-cases \
  --case current_logic:var_gps_z=0.5 \
  --case gps_z_0p3_logic:var_gps_z=0.3 \
  --duration 40 \
  --output-root tmp/proj2_param_sweeps/gating_run1
```

3. 复跑一次检查是否只是偶然结果

```bash
source /opt/ros/jazzy/setup.bash
cd /ws/ee4308
python3 tools/run_proj2_param_sweep.py \
  --no-default-cases \
  --case current_logic_repeat:var_gps_z=0.5 \
  --duration 40 \
  --output-root tmp/proj2_param_sweeps/gating_run2
```

旧文档里保留的关键数字是：

- 早期 baseline：
  - `run1/current` 的 `aligned_all_mae_z = 0.649`
  - `run1/current` 的 `aligned_w7_25_mae_z = 0.533`
- 加入 `z` 轴修正后：
  - `gating_run1/current_logic` 的 `aligned_all_mae_z = 0.020`
  - `gating_run1/current_logic` 的 `aligned_w7_25_mae_z = 0.019`
- 复跑确认：
  - `gating_run2/current_logic_repeat` 的 `aligned_w7_25_mae_z = 0.020`

这组结果非常适合写成简短前后对比，因为它能直接支撑：

- 早期问题不是简单“有点 noisy”，而是 `z` 轴会明显失真；
- 后来的改进不是只靠调方差，而是靠改变 sensor model；
- 改进后结果不是一次偶然 run，而是复跑后仍然成立。

### 7.3 当前最终验证实验可引用数字

当前最新最终验证 [`full_check_gui_04_1`](/home/liuyi/projects/ee4308_proj2/tmp/proj2_runs/full_check_gui_04_1) 的摘要为：

- MAE:
  - `x = 0.209877 m`
  - `y = 0.301763 m`
  - `z = 0.019890 m`
  - `yaw = 0.004353 rad`
- RMSE:
  - `x = 0.287710 m`
  - `y = 0.383183 m`
  - `z = 0.025589 m`
  - `yaw = 0.005508 rad`

因此，当前报告主线可以很自然地写成：

1. 先用旧实验说明早期 `z` 轴问题确实严重；
2. 再说明 sonar gating 和 barometer bias state 为什么能解决这个问题；
3. 最后用当前最终验证图和最终 MAE / RMSE 说明完整任务下 estimator 的最终表现。

### 7.4 已提取的报告图片

为了方便直接上传到 Overleaf，旧提交里的两张代表性前后对比图已经提取并换成更干净的文件名：

- 早期 baseline 位置曲线：`docs/figures/estimator_z_baseline_position.png`
- 加入 `z` 轴修正后的位置曲线：`docs/figures/estimator_z_fixed_position.png`

这两张图现在已经接到 `docs/main.tex` 的 `Representative Comparison` 小节，用来配合前后的 $z$ 轴 MAE 表格。

### 7.5 最后一轮表达审查

最后一轮只做了轻量文字收口，没有改技术内容。检查重点是：

- 不写代码里没有实现的东西；
- 不把一般性的 manual 内容重复太多；
- 少用“看起来像解释、但其实信息量不高”的句子；
- 保留必要公式和真正反映实现的改进点。

当前 `Estimator` 小节的主线是安全的：结构、门限、bias state、GPS pseudo-velocity、forward compensation、参数值、以及最终数字都能在当前代码或实验记录里对应到。
