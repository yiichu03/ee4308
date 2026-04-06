# Proj2 共享仓库说明

## Summary

这个仓库版本面向队友协作与报告整理，不再保留大批中间调参输出。共享版本的原则如下：

- 保留最终 `proj2` 源码、参数和实验脚本。
- 保留最终报告草稿。
- 只保留最终一次完整验证结果 [full_check_gui_04](/home/liuyi/projects/ee4308_proj2/tmp/proj2_runs/full_check_gui_04)。
- 删除个人工作笔记、编辑器配置和历史 sweep 结果。

## Shared Contents

共享版本重点关注这些内容：

- 最终实现：
  - [behavior.cpp](/home/liuyi/projects/ee4308_proj2/src/ee4308_drone/src/behavior.cpp)
  - [controller.cpp](/home/liuyi/projects/ee4308_proj2/src/ee4308_drone/src/controller.cpp)
  - [estimator.cpp](/home/liuyi/projects/ee4308_proj2/src/ee4308_drone/src/estimator.cpp)
- 最终参数：
  - [proj2.yaml](/home/liuyi/projects/ee4308_proj2/src/ee4308_bringup/params/proj2.yaml)
  - [proj2_gt.yaml](/home/liuyi/projects/ee4308_proj2/src/ee4308_bringup/params/proj2_gt.yaml)
- 复现实验脚本：
  - [run_proj2_full_check.py](/home/liuyi/projects/ee4308_proj2/tools/run_proj2_full_check.py)
  - [run_proj2_param_sweep.py](/home/liuyi/projects/ee4308_proj2/tools/run_proj2_param_sweep.py)
  - [record_drone_alignment.py](/home/liuyi/projects/ee4308_proj2/tools/record_drone_alignment.py)
  - [record_drone_plan.py](/home/liuyi/projects/ee4308_proj2/tools/record_drone_plan.py)
  - [plot_drone_bag.py](/home/liuyi/projects/ee4308_proj2/tools/plot_drone_bag.py)
- 最终报告材料：
  - [proj2_report_draft.md](/home/liuyi/projects/ee4308_proj2/docs/proj2_report_draft.md)
  - [estimator_plan_cn.md](/home/liuyi/projects/ee4308_proj2/docs/estimator_plan_cn.md)

## Final Evidence

共享仓库中仅保留最终一次完整验证结果：

- [summary.txt](/home/liuyi/projects/ee4308_proj2/tmp/proj2_runs/full_check_gui_04/plots/summary.txt)
- [trajectory_3d.png](/home/liuyi/projects/ee4308_proj2/tmp/proj2_runs/full_check_gui_04/plots/trajectory_3d.png)
- [position_vs_time.png](/home/liuyi/projects/ee4308_proj2/tmp/proj2_runs/full_check_gui_04/plots/position_vs_time.png)
- [error_vs_time.png](/home/liuyi/projects/ee4308_proj2/tmp/proj2_runs/full_check_gui_04/plots/error_vs_time.png)

如果需要更多对照实验，请直接用 `tools/` 下脚本重新生成，不再依赖仓库里保存的历史 sweep 目录。

## Notes

- 最终 baseline 结论以 [proj2.yaml](/home/liuyi/projects/ee4308_proj2/src/ee4308_bringup/params/proj2.yaml) 和 [proj2_report_draft.md](/home/liuyi/projects/ee4308_proj2/docs/proj2_report_draft.md) 为准。
- `proj2_gt.yaml` 仅用于隔离 `behavior/controller` 与 estimator 问题。
- 清理版仓库的目标是便于队友阅读，不是保存完整调参历史。
