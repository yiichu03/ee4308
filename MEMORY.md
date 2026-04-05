# Proj2 Handoff Memory

本文件用于给新对话快速提供上下文。它不替代详细记录，详细技术过程仍在：

- [estimator_plan_cn.md](/home/liuyi/projects/ee4308_proj2/docs/estimator_plan_cn.md)

---

## 1. 当前项目状态

当前项目已经进入“冻结主线实现 + 做少量确认实验 + 开始写报告”的阶段。

代码实现层面：

- `behavior`：已完成主状态机
- `controller`：已完成 holonomic path tracking
- `estimator`：已完成最终 baseline，包含：
  - `z` 轴 sonar gating
  - baro bias augmentation
  - Joseph form covariance update
  - odom 时间戳统一
  - GPS velocity pseudo-measurement
  - GPS forward compensation

当前默认运行的最终配置文件是：

- [proj2.yaml](/home/liuyi/projects/ee4308_proj2/src/ee4308_bringup/params/proj2.yaml)

只用于 ground-truth 冒烟和隔离 `behavior/controller` 的配置是：

- [proj2_gt.yaml](/home/liuyi/projects/ee4308_proj2/src/ee4308_bringup/params/proj2_gt.yaml)

---

## 2. 当前 baseline 参数

当前默认参数来自 [proj2.yaml](/home/liuyi/projects/ee4308_proj2/src/ee4308_bringup/params/proj2.yaml#L28)。

核心 estimator 参数：

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

行为与控制关键参数：

```yaml
cruise_height: 5.0
reached_thres: 0.3
lookahead_distance: 1.0
max_xy_vel: 1.0
max_z_vel: 0.5
yaw_vel: -0.3
```

说明：

- 这不是“理论最优”，而是当前实验支持下“效果好且更稳”的最终 baseline。
- 后续如果没有明显回归，不建议继续大改结构。

---

## 3. 关键代码文件

行为与控制：

- [behavior.cpp](/home/liuyi/projects/ee4308_proj2/src/ee4308_drone/src/behavior.cpp)
- [controller.cpp](/home/liuyi/projects/ee4308_proj2/src/ee4308_drone/src/controller.cpp)

状态估计：

- [estimator.hpp](/home/liuyi/projects/ee4308_proj2/src/ee4308_drone/include/ee4308_drone/estimator.hpp)
- [estimator.cpp](/home/liuyi/projects/ee4308_proj2/src/ee4308_drone/src/estimator.cpp)

实验和可视化脚本：

- [run_proj2_full_check.py](/home/liuyi/projects/ee4308_proj2/tools/run_proj2_full_check.py)
- [record_drone_alignment.py](/home/liuyi/projects/ee4308_proj2/tools/record_drone_alignment.py)
- [record_drone_plan.py](/home/liuyi/projects/ee4308_proj2/tools/record_drone_plan.py)
- [plot_drone_bag.py](/home/liuyi/projects/ee4308_proj2/tools/plot_drone_bag.py)
- [run_proj2_param_sweep.py](/home/liuyi/projects/ee4308_proj2/tools/run_proj2_param_sweep.py)
- [summarize_proj2_logs.py](/home/liuyi/projects/ee4308_proj2/tools/summarize_proj2_logs.py)

---

## 4. 已完成的重要实验结论

### 4.1 `z` 轴

- 早期主要问题是高空时 `sonar` 不可靠、`baro` 只有量测没有 bias modeling。
- 最终通过：
  - sonar gating
  - `Xz = [z, vz, b_baro]^T`
  - `H_baro = [1, 0, 1]`
  解决。
- 当前 `z` 轴已经不是主要短板。

### 4.2 平面 `x/y`

- 主要问题是 lag 和周期性锯齿，不是坐标系写反或发散。
- 先后尝试过：
  - 调 `var_gps_x/y`
  - 调 `var_imu_x/y`
  - GPS velocity pseudo-measurement
  - GPS forward compensation
- 当前保留：
  - 保守版 GPS velocity pseudo-measurement
  - GPS forward compensation

### 4.3 被否决的方向

- GPS / magnet / baro 软门控作为默认增强没有带来正收益，已从最终 runtime 主线删除。
- 更激进的伪速度修正收益不稳定，也没有保留。

---

## 5. 保留的关键实验目录

这些目录是当前报告证据链的主要来源：

- [run1](/home/liuyi/projects/ee4308_proj2/tmp/proj2_param_sweeps/run1)
- [gating_compare1](/home/liuyi/projects/ee4308_proj2/tmp/proj2_param_sweeps/gating_compare1)
- [xy_logic_run2](/home/liuyi/projects/ee4308_proj2/tmp/proj2_param_sweeps/xy_logic_run2)
- [forward_comp_compare2](/home/liuyi/projects/ee4308_proj2/tmp/proj2_param_sweeps/forward_comp_compare2)
- [vel_fix_run2](/home/liuyi/projects/ee4308_proj2/tmp/proj2_param_sweeps/vel_fix_run2)
- [stability_check](/home/liuyi/projects/ee4308_proj2/tmp/proj2_param_sweeps/stability_check)
- [imu_sweep_b](/home/liuyi/projects/ee4308_proj2/tmp/proj2_param_sweeps/imu_sweep_b)

详细解释见：

- [estimator_plan_cn.md](/home/liuyi/projects/ee4308_proj2/docs/estimator_plan_cn.md#L205)

---

## 6. 最近一次完整运行结果

### `full_check_gui_02`

目录：

- [full_check_gui_02](/home/liuyi/projects/ee4308_proj2/tmp/proj2_runs/full_check_gui_02)

结果：

- [summary.txt](/home/liuyi/projects/ee4308_proj2/tmp/proj2_runs/full_check_gui_02/plots/summary.txt)

关键数值：

- `x MAE = 0.303 m`
- `y MAE = 0.350 m`
- `z MAE = 0.016 m`
- `yaw MAE = 0.019 rad`

判断：

- 整体任务基本跑通
- `z` 和 `yaw` 很好
- `x/y` 仍有一定偏差，但已经是“可接受并可解释”的程度

注意：

- 这次旧版 `drone_plan.log` 没录上，因为当时还是 `ros2 topic echo` 方案
- 之后已修复为 [record_drone_plan.py](/home/liuyi/projects/ee4308_proj2/tools/record_drone_plan.py)

### `full_check_gui_03`

目录：

- [full_check_gui_03](/home/liuyi/projects/ee4308_proj2/tmp/proj2_runs/full_check_gui_03)

结果：

- [summary.txt](/home/liuyi/projects/ee4308_proj2/tmp/proj2_runs/full_check_gui_03/plots/summary.txt)
- [trajectory_3d.png](/home/liuyi/projects/ee4308_proj2/tmp/proj2_runs/full_check_gui_03/plots/trajectory_3d.png)
- [position_vs_time.png](/home/liuyi/projects/ee4308_proj2/tmp/proj2_runs/full_check_gui_03/plots/position_vs_time.png)
- [error_vs_time.png](/home/liuyi/projects/ee4308_proj2/tmp/proj2_runs/full_check_gui_03/plots/error_vs_time.png)

关键数值：

- `x MAE = 0.262 m`
- `y MAE = 0.288 m`
- `z MAE = 0.024 m`
- `yaw MAE = 0.013 rad`

补充观察：

- `drone_plan.csv` 显示完整跑完多轮 `turtle -> turtle goal -> initial` 循环
- 约 `129.7 s` 开始进入 `LANDING`
- 固定 `150 s` 记录窗口结束时仍在下降段，因此这次 `z` 统计会包含 landing 样本

判断：

- 相比 `full_check_gui_02`，整体 `x/y/yaw` 更好
- `z` 和 `yaw` 仍然足够好，不再是主要问题
- 当前最值得在报告里强调的仍然是平面 lag 的结构性改进与实验论证

---

## 7. 用于报告的完整运行与后续建议

`full_check_gui_03` 已完成并完成分析，当前可以把它作为报告中的最终 baseline run。

对应命令是：

```bash
source /opt/ros/jazzy/setup.bash
cd /ws/ee4308
source install/setup.bash
python3 tools/run_proj2_full_check.py \
  --run-name full_check_gui_03 \
  --param-file proj2 \
  --record-plan
```

该脚本的当前行为：

- 默认 `duration = 150s`
- 前台启动仿真，便于观察 Gazebo / RViz
- 后台记录：
  - `aligned_pose_error.csv`
  - `drone_plan.csv`
- 到时自动收尾，也支持手动 `Ctrl+C`
- 自动生成：
  - `trajectory_3d.png`
  - `position_vs_time.png`
  - `error_vs_time.png`
  - `summary.txt`

输出目录应为：

- [full_check_gui_03](/home/liuyi/projects/ee4308_proj2/tmp/proj2_runs/full_check_gui_03)

优先引用这些文件：

- [summary.txt](/home/liuyi/projects/ee4308_proj2/tmp/proj2_runs/full_check_gui_03/plots/summary.txt)
- [trajectory_3d.png](/home/liuyi/projects/ee4308_proj2/tmp/proj2_runs/full_check_gui_03/plots/trajectory_3d.png)
- [position_vs_time.png](/home/liuyi/projects/ee4308_proj2/tmp/proj2_runs/full_check_gui_03/plots/position_vs_time.png)
- [error_vs_time.png](/home/liuyi/projects/ee4308_proj2/tmp/proj2_runs/full_check_gui_03/plots/error_vs_time.png)
- [drone_plan.csv](/home/liuyi/projects/ee4308_proj2/tmp/proj2_runs/full_check_gui_03/drone_plan.csv)

重点判断：

- 最终 baseline 是否完成完整 cycle 并进入 landing
- `x/y` 是否维持在“可接受且可解释”的误差范围
- `z/yaw` 是否继续保持明显优于平面方向

当前报告草稿可直接从这里继续改：

- [proj2_report_draft.md](/home/liuyi/projects/ee4308_proj2/docs/proj2_report_draft.md)

---

## 8. 报告写作建议

对照老师要求：

- [proj2.md](/home/liuyi/projects/ee4308_proj2/docs/proj2.md)
- [email.md](/home/liuyi/projects/ee4308_proj2/docs/email.md)

老师更看重：

- 为什么这样改，而不是只列函数
- 参数为什么这样选，要有实验支撑
- 失败尝试为什么失败，也值得写

建议报告主线：

1. 基础 estimator 结构
2. `z` 轴问题与修复
3. 平面 lag 问题
4. GPS velocity pseudo-measurement
5. GPS forward compensation
6. 参数定稿与稳定性验证

不要把以下内容写成“最终优势”：

- 已放弃的软门控
- 未保留在最终 runtime 中的逻辑
- 单次最好成绩

---

## 9. 新对话推荐任务

如果开新对话，建议让它优先做这些事：

1. 读取本文件和 [estimator_plan_cn.md](/home/liuyi/projects/ee4308_proj2/docs/estimator_plan_cn.md)
2. 分析 [full_check_gui_03](/home/liuyi/projects/ee4308_proj2/tmp/proj2_runs/full_check_gui_03) 的结果
3. 判断是否已经足够：
   - 做最终录屏
   - 开始写报告
4. 帮忙输出报告草稿，优先写：
   - estimator 方法
   - 实验设计
   - 参数选择依据
   - ablation/失败尝试

可以直接对新对话这样说：

```text
请先阅读仓库里的 MEMORY.md 和 docs/estimator_plan_cn.md，再分析 tmp/proj2_runs/full_check_gui_03 的结果，并帮助我写 proj2 报告。
```
