# Proj2 收束、清理与报告优先计划

## Summary

当前项目已经不适合继续做开放式调优，应该转入“冻结主线实现 + 清理代码与文档 + 保留少量确认实验 + 开始写报告”的阶段。

原因很明确：
- `behavior` 和 `controller` 已经基本满足 [proj2.md](/home/liuyi/projects/ee4308_proj2/docs/proj2.md) 的硬要求，继续投入的收益很低。
- `estimator` 已经形成一条完整主线：`z` 轴稳定化、Joseph form、统一时间戳、GPS 伪速度量测、GPS 前向补偿、批量实验与画图工具。
- 老师在 [email.md](/home/liuyi/projects/ee4308_proj2/docs/email.md) 更看重“为什么这样改、实验如何设计、失败尝试为什么失败”，而不是无限继续卷参数。
- 当前 [tmp](/home/liuyi/projects/ee4308_proj2/tmp) 里的实验已经足够支撑报告，只需要压缩成一条清晰叙事链。

## Key Changes

### 1. 代码层只保留“最终采用”的逻辑

保留：
- `behavior` / `controller` 的现有功能实现。
- `estimator` 中真正进入最终 baseline 的改动：
  - `z` 轴 sonar gating
  - baro bias augmentation
  - Joseph form 协方差更新
  - 统一 odom 时间戳
  - GPS velocity pseudo-measurement
  - GPS forward compensation
- 当前 [proj2.yaml](/home/liuyi/projects/ee4308_proj2/src/ee4308_bringup/params/proj2.yaml) 里的最终候选参数。

删除或关闭：
- `behavior.cpp`、`controller.cpp` 里的 `TMP LOG`。
- `estimator.cpp` 里仅用于调参和诊断的 `TMP LOG`。
- 当前实验表明无正收益、且最终 baseline 未启用的逻辑：
  - GPS / magnet / baro 的软门控代码与对应参数。
- 如果最终录屏不再依赖估计轨迹线，可把 `odom_history` 可视化链路作为可选删除项；默认先保留到视频完成。

不删但不作为最终实现叙事重点：
- 实验脚本保留在 `tools/`，因为它们对复现实验和写报告有直接价值。
- `plot_drone_bag.py`、`record_drone_alignment.py`、`run_proj2_param_sweep.py` 这类脚本应保留。
- 明显属于 Lab2 历史工具的 `record_sonar_samples.py`、`estimate_sonar_variance.py` 可保留在仓库，但不进入最终 proj2 代码说明主线。

### 2. `tmp/` 按“实验时间线”整理成少数里程碑目录

保留作为最终证据的目录：
- `tmp/proj2_param_sweeps/run1`
  - 最早的 `z` 轴参数对比基线。
- `tmp/proj2_param_sweeps/gating_compare1`
  - 证明软门控默认值无收益，适合写“尝试过但放弃”。
- `tmp/proj2_param_sweeps/xy_logic_run2`
  - 证明保守版 GPS velocity pseudo-measurement 有价值。
- `tmp/proj2_param_sweeps/forward_comp_compare2`
  - 证明 GPS forward compensation 有价值。
- `tmp/proj2_param_sweeps/vel_fix_run2`
  - 当前平面参数主结论来源。
- `tmp/proj2_param_sweeps/stability_check`
  - 说明 run-to-run 方差存在，避免报告夸大单次结果。
- `tmp/proj2_param_sweeps/imu_sweep_b`
  - 当前 `var_imu_x/y` 选择依据。

可归档或删除的目录：
- `tmp/lab2_*`
- `tmp/bag_examples`
- `tmp/proj2_param_sweeps/xy_run1`
- `tmp/proj2_param_sweeps/xy_run2`
- `tmp/proj2_param_sweeps/xy_run3`
- `tmp/proj2_param_sweeps/gating_run1`
- `tmp/proj2_param_sweeps/gating_run2`
- `tmp/proj2_param_sweeps/planar_logic_run1`
- `tmp/proj2_param_sweeps/vel_fix_run1`

整理原则：
- 只保留“能支撑最终报告论点”的目录。
- 其余实验如果结论已经写进文档，就不再留原始目录占视线。
- 在 [docs/estimator_plan_cn.md](/home/liuyi/projects/ee4308_proj2/docs/estimator_plan_cn.md) 补一张 `tmp` 时间线索引表，说明每个保留目录的用途。

### 3. `estimator_plan_cn.md` 需要从“流水账”收束成“最终可引用文档”

重构目标：
- 前面放“最终 baseline 和最终结论”。
- 中间放“最终采用的设计与理由”。
- 后面放“被否决/失败的尝试”。
- 最后放“报告写作建议与图表索引”。

必须清理的点：
- 合并重复实验结论，避免同一主题出现多个版本。
- 修正相互冲突的表述，尤其是 GPS forward compensation 的结论。
- 将“过时参数结论”明确标记为历史尝试，避免和最终参数混淆。
- 失败尝试保留，但压缩成“尝试内容 + 为什么无效 + 最终为什么不用”。

最终文档建议结构：
- 当前最终实现
- 参数定稿与原因
- 关键实验链
- 失败尝试与放弃理由
- 报告可直接引用的图和论点
- 提交前清单

### 4. 现在开始写报告，只做少量确认实验

从现在起，不再做大范围 sweep。只做两类实验：
- 最终 baseline 的确认性重复运行
  - 用于生成最终图、视频和稳定性说明。
- 1 到 2 个对照实验
  - 例如关闭 GPS velocity pseudo-measurement
  - 或关闭 GPS forward compensation
  - 只选最能支撑“为什么当前方案更好”的对照

报告主线建议：
- 问题：平面方向 lag / 锯齿，`z` 轴高空量测不稳。
- 设计：Joseph form、baro bias、sonar gating、GPS velocity pseudo-measurement、GPS forward compensation。
- 实验：用少量有因果关系的对比证明每个关键选择的价值。
- 结论：当前方案在老师要求范围内实现完整，且有明确的工程改进与实验支撑。

## Public Interfaces / Final Runtime Surface

- 不改 `behavior` / `controller` / `planner` 的对外接口。
- `estimator` 最终保持现有核心 topic 行为：
  - `/drone/odom`
  - `/drone/true_odom`
- `proj2.yaml` 保留最终真正使用的参数；删除已废弃的门控参数，避免“代码没用、参数还在”。
- `tools/` 保留复现实验和画图所需脚本，不视为 runtime API。

## Test Plan

- 构建检查：
  - `ee4308_drone`
  - `ee4308_bringup`
- 功能冒烟：
  - `proj2_gt` 跑一次，确认 `behavior/controller` 没被清理动作破坏。
  - `proj2` 跑一次，确认 estimator baseline 正常。
- 报告实验：
  - baseline 至少重复 2 到 3 次。
  - 选 1 到 2 个最关键 ablation。
- 提交前检查：
  - 源码里不再残留 `TMP LOG`。
  - `estimator_plan_cn.md` 前半部分只出现最终参数与最终结论。
  - `tmp/` 目录只保留少量里程碑实验。
  - 报告中不写未采用或未实现的内容。

## Assumptions

- 当前方向应以“报告优先”替代“继续开放式调优”。
- 失败尝试应从 runtime 代码中删除，但应以简洁形式保留在文档中，用于报告叙事。
- 当前最终 baseline 以 [proj2.yaml](/home/liuyi/projects/ee4308_proj2/src/ee4308_bringup/params/proj2.yaml) 为准，后续只允许非常小的确认性修改。
- 长时间实验继续由你本地运行，我只提供命令与结果解读。
