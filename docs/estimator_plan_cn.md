# Project 2 Estimator 中文定稿记录

## 1. 当前最终状态

当前项目已经从“边做边试”切换到“冻结主线实现、保留少量确认实验、开始写报告”的阶段。

当前仓库中与 `proj2` 直接相关的核心实现如下：

- 行为与控制：
  - [behavior.cpp](/home/liuyi/projects/ee4308_proj2/src/ee4308_drone/src/behavior.cpp)
  - [controller.cpp](/home/liuyi/projects/ee4308_proj2/src/ee4308_drone/src/controller.cpp)
- 状态估计：
  - [estimator.hpp](/home/liuyi/projects/ee4308_proj2/src/ee4308_drone/include/ee4308_drone/estimator.hpp)
  - [estimator.cpp](/home/liuyi/projects/ee4308_proj2/src/ee4308_drone/src/estimator.cpp)
- 最终参数：
  - [proj2.yaml](/home/liuyi/projects/ee4308_proj2/src/ee4308_bringup/params/proj2.yaml)
  - [proj2_gt.yaml](/home/liuyi/projects/ee4308_proj2/src/ee4308_bringup/params/proj2_gt.yaml)
- 实验与画图工具：
- [record_drone_alignment.py](/home/liuyi/projects/ee4308_proj2/tools/record_drone_alignment.py)
- [record_drone_plan.py](/home/liuyi/projects/ee4308_proj2/tools/record_drone_plan.py)
- [plot_drone_bag.py](/home/liuyi/projects/ee4308_proj2/tools/plot_drone_bag.py)
- [run_proj2_param_sweep.py](/home/liuyi/projects/ee4308_proj2/tools/run_proj2_param_sweep.py)
- [run_proj2_full_check.py](/home/liuyi/projects/ee4308_proj2/tools/run_proj2_full_check.py)
- [summarize_proj2_logs.py](/home/liuyi/projects/ee4308_proj2/tools/summarize_proj2_logs.py)

当前代码层面的原则：

- 保留最终 baseline 真正在用的 estimator 逻辑。
- 删除仅用于调参的 `TMP LOG`。
- 删除实验已经证明无正收益、且最终版本未启用的 GPS / magnet / baro 软门控逻辑。
- 保留实验脚本与 `tmp/` 中少量关键目录，作为报告证据链。

---

## 2. 最终 baseline 设计

### 2.1 状态向量

最终 estimator 仍采用分轴滤波设计：

- `Xx = [x, vx]^T`
- `Xy = [y, vy]^T`
- `Xz = [z, vz, b_baro]^T`
- `Xa = [yaw, yaw_rate]^T`

这样做的原因：

- 课程框架本身就是按分轴简化建模。
- 各轴量测来源天然分离，分轴实现更直观，矩阵规模更小。
- `z` 轴为支持 barometer bias augmentation，扩展为 3 维状态。

### 2.2 最终保留的 estimator 改进

以下逻辑是最终版本保留的核心改进：

1. `z` 轴 sonar gating  
   仅在低空且创新较小时接受 sonar，防止高空量到错误平面导致高度塌陷。

2. barometer bias augmentation  
   使用 `Xz = [z, vz, b_baro]^T` 和 `H_baro = [1, 0, 1]`，让 baro 的系统偏差成为显式状态。

3. Joseph form covariance update  
   取代更脆弱的简单协方差更新形式，提升数值稳定性。

4. odom 时间戳统一  
   `/drone/odom` 使用最近一次状态更新时间，而不是 `this->now()`，减少后处理中的表观延迟。

5. GPS velocity pseudo-measurement  
   由相邻 GPS 位置差分出平面速度，再用低通滤波后作为 `vx / vy` 的伪量测，减轻平面 lag。

6. GPS forward compensation  
   若 GPS 消息时间戳落后于当前状态，则用当前速度将 GPS 位置前推到当前时刻，再进行 correction。

### 2.3 明确删除的逻辑

以下内容已不再属于最终 runtime 主线：

- `behavior` / `controller` 中的调试 `TMP LOG`
- `estimator` 中所有临时调试 `TMP LOG`
- GPS 位置软门控
- magnet 软门控
- baro 软门控

保留但不作为最终实现亮点：

- `odom_history` 轨迹历史可视化
- `tools/` 中的 Lab2 辅助脚本

---

## 3. 最终参数定稿

当前最终 baseline 以 [proj2.yaml](/home/liuyi/projects/ee4308_proj2/src/ee4308_bringup/params/proj2.yaml) 为准。

### 3.1 Estimator 参数

```yaml
var_imu_x: 1.5
var_imu_y: 1.5
var_imu_z: 20.0
var_imu_a: 1.0

var_gps_x: 0.15
var_gps_y: 0.15
var_gps_z: 0.5

var_baro: 1.0
var_sonar: 0.03
var_magnet: 1.0

gps_forward_compensation_enable: true
gps_forward_compensation_max_dt: 0.5

gps_velocity_alpha: 0.9
gps_velocity_variance_scale: 0.1
gps_velocity_min_variance: 0.04
gps_velocity_max_innovation: 1.5
gps_velocity_min_dt: 0.2
gps_velocity_max_dt: 2.0
```

### 3.2 参数含义与当前选择理由

`var_sonar = 0.03`

- 来自早期 `lab2` 和 `proj2` 的 `z` 轴调参。
- 结合 sonar gating 后稳定性足够好。

`var_gps_x = var_gps_y = 0.15`

- 平面方向唯一的绝对位置传感器是 GPS。
- 将 `var_gps_x / y` 降低到 `0.15` 后，位置 correction 更强，平面 lag 明显减轻。
- 继续降低到 `0.10` 虽然局部会改善某一轴，但总体不稳定。

`var_imu_x = var_imu_y = 1.5`

- 与 `3.0` 相比，中位数性能近似，但重复实验方差更小。
- 最终不是选“偶尔最好”的参数，而是选“更稳”的参数。

`gps_velocity_variance_scale = 0.1`
`gps_velocity_min_variance = 0.04`
`gps_velocity_alpha = 0.9`

- 这是保守版 GPS 伪速度量测的有效区间。
- 更强或更弱都在实验中表现更差。

`gps_forward_compensation_enable = true`
`gps_forward_compensation_max_dt = 0.5`

- 当前实验显示，启用前向补偿优于关闭。
- `0.5s` 的窗口优于更小的 `0.2s`。

### 3.3 `proj2_gt.yaml` 的角色

[proj2_gt.yaml](/home/liuyi/projects/ee4308_proj2/src/ee4308_bringup/params/proj2_gt.yaml) 仅用于隔离 `behavior/controller`：

- 参数与 `proj2.yaml` 保持一致
- 唯一区别是 `use_ground_truth: true`

它不是最终实验参数文件，而是用于冒烟测试与问题定位。

---

## 4. 核心代码结构梳理

### 4.1 最终需要保留的核心文件

#### 行为与控制

- [behavior.cpp](/home/liuyi/projects/ee4308_proj2/src/ee4308_drone/src/behavior.cpp)
  - 负责 `TAKEOFF -> TURTLE_POSITION -> TURTLE_WAYPOINT -> INITIAL -> LANDING`
- [controller.cpp](/home/liuyi/projects/ee4308_proj2/src/ee4308_drone/src/controller.cpp)
  - 负责 holonomic pure pursuit 跟踪与固定 yaw 旋转

#### 状态估计

- [estimator.cpp](/home/liuyi/projects/ee4308_proj2/src/ee4308_drone/src/estimator.cpp)
  - `callbackSubIMU_()`：预测
  - `callbackSubGPS_()`：GPS 位置 correction + 伪速度 correction + 前向补偿
  - `callbackSubSonar_()`：低空可信 sonar correction
  - `callbackSubBaro_()`：baro bias augmentation correction
  - `callbackSubMagnetic_()`：yaw correction
  - `publishOdomAndHistory_()`：发布 `/drone/odom` 和估计轨迹历史

#### 参数与复现工具

- [proj2.yaml](/home/liuyi/projects/ee4308_proj2/src/ee4308_bringup/params/proj2.yaml)
- [proj2_gt.yaml](/home/liuyi/projects/ee4308_proj2/src/ee4308_bringup/params/proj2_gt.yaml)
- [run_proj2_param_sweep.py](/home/liuyi/projects/ee4308_proj2/tools/run_proj2_param_sweep.py)
- [record_drone_alignment.py](/home/liuyi/projects/ee4308_proj2/tools/record_drone_alignment.py)
- [plot_drone_bag.py](/home/liuyi/projects/ee4308_proj2/tools/plot_drone_bag.py)

### 4.2 当前可以视为“非主线”的内容

- Lab2 残留实验工具：
  - [record_sonar_samples.py](/home/liuyi/projects/ee4308_proj2/tools/record_sonar_samples.py)
  - [estimate_sonar_variance.py](/home/liuyi/projects/ee4308_proj2/tools/estimate_sonar_variance.py)
- 早期大范围 sweep 的中间结果目录
- 已被放弃的门控调试逻辑

---

## 5. `tmp/` 实验时间线与保留目录

下面是当前建议保留的关键目录，以及它们在最终报告中的用途。

| 目录 | 角色 | 最终用途 |
|---|---|---|
| [tmp/proj2_param_sweeps/run1](/home/liuyi/projects/ee4308_proj2/tmp/proj2_param_sweeps/run1) | 最早的 `z` 轴基线 sweep | 说明最初 `z` 轴问题与早期调参范围 |
| [tmp/proj2_param_sweeps/gating_compare1](/home/liuyi/projects/ee4308_proj2/tmp/proj2_param_sweeps/gating_compare1) | 软门控开关对比 | 作为“尝试过但放弃”的证据 |
| [tmp/proj2_param_sweeps/xy_logic_run2](/home/liuyi/projects/ee4308_proj2/tmp/proj2_param_sweeps/xy_logic_run2) | GPS 伪速度量测对比 | 支撑“保守版伪速度量测有效” |
| [tmp/proj2_param_sweeps/forward_comp_compare2](/home/liuyi/projects/ee4308_proj2/tmp/proj2_param_sweeps/forward_comp_compare2) | GPS 前向补偿对比 | 支撑“前向补偿有效” |
| [tmp/proj2_param_sweeps/vel_fix_run2](/home/liuyi/projects/ee4308_proj2/tmp/proj2_param_sweeps/vel_fix_run2) | 平面参数主 sweep | 最终 `var_gps_x/y` 与伪速度参数主依据 |
| [tmp/proj2_param_sweeps/stability_check](/home/liuyi/projects/ee4308_proj2/tmp/proj2_param_sweeps/stability_check) | 同参数重复运行 | 证明 run-to-run 方差存在 |
| [tmp/proj2_param_sweeps/imu_sweep_b](/home/liuyi/projects/ee4308_proj2/tmp/proj2_param_sweeps/imu_sweep_b) | `var_imu_x/y` 重复性对比 | 支撑最终选 `1.5` 而不是 `3.0` |

建议删除或归档的目录：

- `tmp/lab2_*`
- `tmp/bag_examples`
- `tmp/proj2_param_sweeps/xy_run1`
- `tmp/proj2_param_sweeps/xy_run2`
- `tmp/proj2_param_sweeps/xy_run3`
- `tmp/proj2_param_sweeps/gating_run1`
- `tmp/proj2_param_sweeps/gating_run2`
- `tmp/proj2_param_sweeps/planar_logic_run1`
- `tmp/proj2_param_sweeps/vel_fix_run1`

这部分的原则很简单：

- 只保留能直接支撑最终报告论点的目录。
- 已经被写进结论、但不再需要原始目录支撑的早期实验，直接删掉。

---

## 6. 关键实验链与当前结论

### 6.1 `z` 轴收敛：先解决高度塌陷

早期阶段最明显的问题是：

- 高空时 `sonar` 不可靠
- `baro` 只有量测没有真正形成偏置模型
- `z` 轴会出现长时间漂移或偶发塌陷

最终解决方案：

- sonar gating
- `Xz = [z, vz, b_baro]^T`
- `H_baro = [1, 0, 1]`

这一块现在已经不再是主要短板。

### 6.2 平面 lag：GPS 伪速度量测是第一步有效改进

[xy_logic_run2](/home/liuyi/projects/ee4308_proj2/tmp/proj2_param_sweeps/xy_logic_run2) 的结论是：

- 保守版 GPS velocity pseudo-measurement 略优于完全关闭
- 更强或更弱都变差

这说明：

- 平面 lag 不是单靠参数就能完全消掉
- 但用 GPS 差分速度做保守 correction 是值得保留的

### 6.3 软门控：尝试过，但最终放弃

[gating_compare1](/home/liuyi/projects/ee4308_proj2/tmp/proj2_param_sweeps/gating_compare1) 的结论是：

- `gating_default` 不如 `gating_off`
- `gating_tighter` 明显更差

因此最终决策是：

- 软门控代码与参数从 runtime 中删除
- 只把这轮实验保留在文档里，作为“尝试过但无收益”的例子

### 6.4 GPS forward compensation：有效，且优于关闭

[forward_comp_compare2](/home/liuyi/projects/ee4308_proj2/tmp/proj2_param_sweeps/forward_comp_compare2/summary.txt) 的结果：

1. `fc_on_no_gate | aligned_score=0.559`
2. `fc_off_no_gate | aligned_score=0.864`
3. `fc_small_dt_no_gate | aligned_score=0.940`

这轮实验支撑两个结论：

- 前向补偿是有效的
- `0.5s` 的补偿窗口优于 `0.2s`

### 6.5 平面参数定稿：`vel_fix_run2`

[vel_fix_run2](/home/liuyi/projects/ee4308_proj2/tmp/proj2_param_sweeps/vel_fix_run2/summary.txt) 的最佳结果集中在：

- `var_gps_x/y = 0.15`
- 伪速度量测开启

代表性结果：

1. `gps0p15_repeat | aligned_score=0.330 | aligned w10_18 mae(x,y,z)=(0.230, 0.058, 0.020)`
2. `gps0p12 | aligned_score=0.381 | aligned w10_18 mae(x,y,z)=(0.235, 0.110, 0.019)`
3. `gps0p15_no_vel | aligned_score=0.575 | aligned w10_18 mae(x,y,z)=(0.393, 0.121, 0.028)`

可直接得出的结论：

- `var_gps_x/y = 0.15` 比更高的值跟手
- 关闭伪速度量测后，平面表现明显变差

### 6.6 稳定性：要选“更稳”的参数，而不是偶尔更好

[stability_check](/home/liuyi/projects/ee4308_proj2/tmp/proj2_param_sweeps/stability_check/summary.txt) 显示：

- 同参数重复运行，`aligned_score` 波动明显
- 说明单次最好成绩不能直接当作最终参数依据

[imu_sweep_b](/home/liuyi/projects/ee4308_proj2/tmp/proj2_param_sweeps/imu_sweep_b/summary.txt) 显示：

- `var_imu = 1.5` 与 `3.0` 的中位数表现近似
- 但 `1.5` 的 run-to-run 波动更小

因此最终选择：

- `var_imu_x = var_imu_y = 1.5`

---

## 7. 已放弃或仅保留在文档中的尝试

这些尝试不再出现在最终 runtime 逻辑里，但值得在报告里简要提到。

### 7.1 GPS / magnet / baro 软门控

实现思路：

```cpp
if (innovation within abs-threshold and sigma-threshold) {
    apply correction;
}
```

为什么放弃：

- `gating_default` 已经不如关闭门控
- 更严格的门控显著伤害正常 correction
- 最终它更像“挡掉正常更新”，而不是“只挡掉坏点”

### 7.2 更激进的 GPS 伪速度量测

尝试过更强或更弱的伪速度量测参数，但结果都更差。

最终保留的是：

- `gps_velocity_alpha = 0.9`
- `gps_velocity_variance_scale = 0.1`
- `gps_velocity_min_variance = 0.04`

### 7.3 早期大范围 `x/y` sweep

如 `xy_run1`、`xy_run2`、`xy_run3` 这些目录在探索阶段有价值，但最终只保留其结论，不再保留完整目录。

原因：

- 它们帮助缩小搜索区间
- 但最终参数依据已经由 `vel_fix_run2`、`stability_check`、`imu_sweep_b` 接管

---

## 8. 对照 proj2.md：当前实现是否完整

对照 [proj2.md](/home/liuyi/projects/ee4308_proj2/docs/proj2.md) 中 estimator 部分，当前实现状态如下：

| 要求 | 状态 |
|---|---|
| `callbackSubIMU_()` | 已实现 |
| `callbackSubGPS_()` | 已实现 |
| `getECEF_()` | 已实现 |
| `callbackSubMagnetic_()` | 已实现 |
| `callbackSubSonar_()` | 已实现 |
| `callbackSubBaro_()` | 已实现，且属于三人组可选增强 |

另外，以下内容属于超出最低要求的改进：

- Joseph form 协方差更新
- baro bias augmentation
- sonar gating
- GPS velocity pseudo-measurement
- GPS forward compensation

因此从实现完整度上看，当前版本已经足够进入报告阶段。

---

## 9. 报告写作建议

结合 [email.md](/home/liuyi/projects/ee4308_proj2/docs/email.md)，老师更看重的是：

- 为什么改
- 怎么改
- 改动在什么情况下有效
- 参数是怎么实验出来的
- 失败尝试为什么失败

### 9.1 推荐叙事主线

按“问题 -> 设计 -> 实验 -> 结论”来写，而不是按函数一个个流水账介绍。

推荐顺序：

1. 基础 estimator 结构
2. `z` 轴问题与修复
3. 平面 lag 问题
4. GPS 伪速度量测
5. GPS 前向补偿
6. 参数定稿与稳定性验证

### 9.2 最值得放进报告的图

优先级最高的图：

1. 最终 baseline 的 `trajectory_3d.png`
2. 最终 baseline 的 `position_vs_time.png`
3. 最终 baseline 的 `error_vs_time.png`
4. `forward_comp_compare2` 的对比结果
5. `vel_fix_run2` 的参数对比表
6. `imu_sweep_b` 的中位数与方差对比表

### 9.3 报告里应该强调的点

- `z` 轴问题不是单调参解决，而是通过 sensor modeling 解决
- 平面 lag 是结构性问题，因此用了保守版 GPS 伪速度量测
- 前向补偿解决了 GPS correction 的时序偏差
- 最终参数不是看单次最好成绩，而是看重复实验的中位数和方差

### 9.4 报告里不要写的内容

- 未保留在最终代码里的逻辑被写成最终方案
- 把单次最好结果当作稳定结论
- 把已经放弃的软门控写成最终优势
- 把未实际开启的逻辑写成最终实验结论

---

## 10. 提交前清单

### 10.1 代码层

- `TMP LOG` 已全部清理
- 废弃软门控已从 runtime 代码删除
- `proj2.yaml` 只保留最终使用参数
- `proj2_gt.yaml` 仅作为 ground-truth 冒烟配置

### 10.2 文档层

- 本文档前半部分仅保留最终实现与最终结论
- 历史失败尝试集中放在后部
- 报告中只写最终保留的功能和真实实验结论

### 10.3 实验层

从现在起只做：

1. baseline 重复运行 2 到 3 次
2. 1 到 2 个关键 ablation

不再继续开放式大 sweep。

### 10.4 建议的最后几条命令

#### baseline 重复验证

```bash
source /opt/ros/jazzy/setup.bash
cd /ws/ee4308
python3 tools/run_proj2_param_sweep.py \
  --no-default-cases \
  --duration 40 \
  --output-root tmp/proj2_param_sweeps/final_baseline_repeat \
  --case baseline_r1: \
  --case baseline_r2: \
  --case baseline_r3:
```

#### 关键对照：关闭 GPS 伪速度量测

```bash
source /opt/ros/jazzy/setup.bash
cd /ws/ee4308
python3 tools/run_proj2_param_sweep.py \
  --no-default-cases \
  --duration 40 \
  --output-root tmp/proj2_param_sweeps/final_ablation_vel \
  --case baseline: \
  --case no_gps_vel:gps_velocity_variance_scale=0.0
```

#### 关键对照：关闭 GPS 前向补偿

```bash
source /opt/ros/jazzy/setup.bash
cd /ws/ee4308
python3 tools/run_proj2_param_sweep.py \
  --no-default-cases \
  --duration 40 \
  --output-root tmp/proj2_param_sweeps/final_ablation_forward_comp \
  --case baseline: \
  --case no_forward_comp:gps_forward_compensation_max_dt=0.0
```

---

## 11. 完整任务验证推荐流程

如果要验证 `behavior + controller + estimator` 的完整联动，而不是只跑固定时长的调参窗口，推荐直接使用：

- [run_proj2_full_check.py](/home/liuyi/projects/ee4308_proj2/tools/run_proj2_full_check.py)

这个脚本会：

- 前台启动 `proj2` 仿真，便于直接观察 Gazebo / RViz。
- 后台记录 `/drone/odom` 和 `/drone/true_odom` 的对齐 CSV。
- 可选记录 `/drone/plan`，用于核对 waypoint 切换。
- 在到达默认时长后自动清理并生成图。
- 也支持手动 `Ctrl+C` 提前结束。

推荐命令：

```bash
source /opt/ros/jazzy/setup.bash
cd /ws/ee4308
source install/setup.bash
python3 tools/run_proj2_full_check.py \
  --run-name full_check_01 \
  --param-file proj2 \
  --record-plan
```

说明：

- 默认 `--duration 150`，适合覆盖当前仿真配置下的一整段任务。
- 如果观察到无人机已经完成 cycle 并降落，也可以直接按 `Ctrl+C` 提前结束。
- 如果想完全手动控制时长，可以显式传：
  - `--duration 0`
- 输出目录默认是：
  - [tmp/proj2_runs/full_check_01](/home/liuyi/projects/ee4308_proj2/tmp/proj2_runs/full_check_01)

主要输出包括：

- `aligned_pose_error.csv`
- `drone_plan.csv`（如果启用了 `--record-plan`）
- `plots/trajectory_3d.png`
- `plots/position_vs_time.png`
- `plots/error_vs_time.png`
- `plots/summary.txt`

如果只想做数据验证、不需要 GUI，可以额外加 `--headless`。

---

## 12. 最终判断

当前项目的合理策略不是继续追求更复杂的 estimator，而是：

- 冻结主线实现
- 清理运行时代码
- 保留少量关键实验目录
- 开始根据老师偏好写报告

换句话说，当前版本已经足够“交作业”，现在更需要的是“把故事讲清楚”。
