# Possible Questions And Reference Answers

Each answer is given first in English, then in Chinese.

## 1. Why did you keep the axis-by-axis simplified filter instead of building a full 3D EKF?

English:
We kept the axis-by-axis simplified Kalman filter because it already matched the structure suggested by the handout, and it was sufficient for the main problems we observed. Our problems were not strongly coupled across all states. The most obvious issues were unreliable vertical correction and horizontal lag, so a simpler decomposed filter let us fix the real bottlenecks directly without introducing unnecessary implementation complexity.

中文：
我们保留分轴 simplified Kalman filter，是因为它本来就符合 handout 给出的结构，而且已经足够应对我们真正观察到的问题。我们的问题并不是所有状态强耦合在一起，最明显的是竖直方向修正不可靠以及平面方向 lag。所以用分解后的滤波器，可以更直接地针对瓶颈做改进，而不用引入不必要的实现复杂度。

## 2. What was the main problem in the early estimator?

English:
There were two main problems. Along the vertical axis, the sonar could become unreliable at higher altitude and the barometer had noticeable bias. Along the horizontal plane, the estimate was usually stable but tended to lag behind the true motion. So the core issues were not general instability, but incorrect vertical correction and delayed planar tracking.

中文：
主要有两个问题。竖直方向上，sonar 在较高高度时会变得不可靠，而 barometer 又有明显偏置。平面方向上，估计通常是稳定的，但会明显落后于真实运动。所以核心问题不是普遍的不稳定，而是竖直修正错误和水平跟踪滞后。

## 3. Why did you add sonar gating instead of removing sonar completely?

English:
We did not remove sonar completely because it was still very useful near the ground. The issue was that it was not equally trustworthy at all altitudes. Gating let us keep the strong low-altitude correction while rejecting updates that were likely to be unreliable. In other words, the goal was selective trust, not complete removal.

中文：
我们没有完全删掉 sonar，因为它在近地面时仍然很有用。问题不在于它完全没价值，而在于它在不同高度下的可信度不同。gating 让我们保留低空时很强的修正作用，同时拒绝那些大概率不可靠的更新。也就是说，我们要的是“有选择地相信”，而不是“一刀切地删除”。

## 4. What conditions do you use for sonar gating?

English:
In the final code, we only accept the sonar update when the predicted height is at most `3.8 m` and the sonar innovation is at most `1.0 m`. These two conditions help reject cases where the measurement is likely inconsistent with the current state.

中文：
在最终代码里，我们只在预测高度不超过 `3.8 m` 且 sonar innovation 不超过 `1.0 m` 时接受 sonar 更新。这两个条件一起用来拒绝那些与当前状态明显不一致的测量。

## 5. Why did you augment the vertical state with a barometer bias term?

English:
We augmented the vertical state because the barometer was not behaving like a clean direct measurement of altitude. It behaved more like altitude plus a bias. If we treated it as direct altitude, that bias would be forced into the `z` estimate. By introducing a separate bias state, the filter could absorb the slow offset into `b_baro` instead of distorting the actual altitude estimate.

中文：
我们给竖直状态加上 barometer bias，是因为 barometer 并不像一个干净的高度直接观测，它更像是“高度加上一个偏置”。如果把它直接当成高度，那这个偏置就会被硬塞进 `z` 估计里。加入单独的 bias 状态以后，滤波器就可以把这个慢偏差吸收到 `b_baro` 中，而不是扭曲真实高度。

## 6. Why was the barometer improvement especially reasonable in this project?

English:
This improvement was especially reasonable because the handout itself pointed out that barometer altitude could contain significant bias and suggested that augmenting the state could be more useful than subtracting only an initial offset. So our approach was consistent with both the observed data and the project guidance.

中文：
这个改进特别合理，因为 handout 本身就指出了 barometer 高度可能带有显著 bias，并建议说扩展状态去估计 bias 可能比只减去一个初始偏移更有用。所以我们的做法同时符合实验现象和题目指引。

## 7. Why did you say the horizontal problem was lag rather than instability?

English:
We called it lag because the estimate usually followed the correct overall trend and remained bounded, but it systematically stayed behind the true motion. That is different from instability, where the estimate would diverge or oscillate uncontrollably. This distinction mattered because it guided us toward time-alignment and velocity-related improvements instead of trying to solve the wrong problem.

中文：
我们把它称为 lag，是因为估计通常能跟住正确的大趋势，也没有发散，但会系统性地落后于真实运动。这和 instability 不一样；如果是不稳定，估计会发散或者出现难以控制的振荡。这个区分很重要，因为它决定了我们要做的是时序对齐和速度相关的改进，而不是去解决一个并不存在的问题。

## 8. Why did you create a GPS pseudo-velocity correction?

English:
GPS gave us position but not direct planar velocity. Since horizontal lag was a key problem, we used consecutive GPS positions to construct a pseudo-velocity estimate, smoothed it, and used it conservatively to correct `v_x` and `v_y`. The purpose was not to trust GPS too aggressively, but to add a weak external constraint that reduced persistent lag.

中文：
GPS 给我们的是位置，不是直接的平面速度。既然水平 lag 是核心问题，我们就用连续两次 GPS 位置构造一个伪速度，对它做平滑后，再保守地用来修正 `v_x` 和 `v_y`。目的不是过度相信 GPS，而是给水平速度增加一个较弱但有用的外部约束，从而减轻持续 lag。

## 9. Why did you still need GPS forward compensation after adding pseudo-velocity correction?

English:
The pseudo-velocity correction helped the velocity states, but it did not directly solve the fact that a GPS position message could already be slightly stale when it arrived. Forward compensation addressed that time mismatch. We compensated the GPS position using the current estimated velocity when the lag was small and positive, so that the position correction was less outdated when it was applied.

中文：
伪速度修正主要帮助的是速度状态，但它并不能直接解决另一个问题：GPS 位置消息到达时本身可能已经有一点过时。forward compensation 解决的是这个时序错位问题。当 lag 较小且为正时，我们用当前估计速度把 GPS 位置往前补一点，这样做 correction 时就不会那么“落后”。

## 10. How did you choose the final parameters?

English:
We did not choose the parameters from a single lucky run. We used small sweeps and repeat runs, and we kept values that consistently supported the same behavior across runs. The main logic was to reduce trust in vertical IMU prediction, keep low-altitude sonar correction strong after gating, and make GPS-related planar correction active but conservative.

中文：
我们不是根据一次碰巧表现好的 run 来定参数，而是通过一些小规模 sweep 和重复实验，保留那些在多次运行中都能支持相同行为的值。总体逻辑是：降低对竖直 IMU prediction 的信任，在 gating 前提下保留低空 sonar 的强修正，同时让基于 GPS 的平面修正保持有效但不过激。

## 11. What are the most important final parameters to remember?

English:
The most important ones are `var_imu_z = 20.0`, `var_gps_x = var_gps_y = 0.15`, `var_sonar = 0.03`, `gps_forward_compensation_max_dt = 0.5`, `gps_velocity_alpha = 0.9`, `gps_velocity_variance_scale = 0.1`, and `gps_velocity_max_innovation = 1.5`. These values reflect the final trust balance among IMU, sonar, and GPS-related corrections.

中文：
最重要的参数包括 `var_imu_z = 20.0`、`var_gps_x = var_gps_y = 0.15`、`var_sonar = 0.03`、`gps_forward_compensation_max_dt = 0.5`、`gps_velocity_alpha = 0.9`、`gps_velocity_variance_scale = 0.1`、`gps_velocity_max_innovation = 1.5`。这些值体现了最终对 IMU、sonar 和 GPS 相关修正之间的信任平衡。

## 12. What was your strongest result?

English:
Our strongest result was the improvement on the `z` axis. In the representative comparison, the early baseline had very large `z` error, but after the vertical fixes, the `z` MAE dropped to around `0.020 m`. This showed that the main height failure was not just reduced slightly; it was fundamentally addressed.

中文：
我们最强的结果是 `z` 轴改进。在代表性对比里，早期 baseline 的 `z` 误差很大，但做完竖直方向改进后，`z` MAE 降到了大约 `0.020 m`。这说明高度问题不是“稍微变好一点”，而是主要失效模式被真正解决了。

## 13. What do the final full-mission results show?

English:
The final full-mission results show that `z` and yaw are already quite accurate, and that the estimator is stable enough to support the whole mission. The remaining weakness is mainly in `x` and `y`, where moderate planar lag still exists.

中文：
最终整段任务结果表明，`z` 和 yaw 已经比较准确，estimator 也足够稳定，可以支持整个任务。剩下的主要弱点仍然是 `x` 和 `y`，也就是中等程度的平面 lag。

## 14. What limitation would you openly admit in the defense?

English:
I would openly admit that the estimator is not fully solved in the horizontal plane. The system is usable and stable, but the planar estimate still lags behind the true motion. I think this is the most honest and technically accurate limitation to state.

中文：
我会直接承认 estimator 在水平面上还没有被完全解决。系统已经可用而且稳定，但平面估计仍然落后于真实运动。我认为这是最诚实、也最技术准确的限制描述。

## 15. What is your shortest complete answer if the teacher asks, “What exactly did you improve?”

English:
I kept the simplified axis-by-axis filter structure from the handout, then made four targeted estimator improvements: sonar gating, barometer bias augmentation in the vertical state, GPS pseudo-velocity correction for planar lag, and GPS forward compensation for mild GPS delay. These changes mainly fixed the vertical failure mode and reduced horizontal lag, while keeping the estimator simple and explainable.

中文：
我保留了 handout 的分轴 simplified filter 结构，然后做了四个针对性的 estimator 改进：sonar gating、在竖直状态中加入 barometer bias、用 GPS 伪速度修正平面 lag、以及针对轻微 GPS 延迟做 forward compensation。这些改动主要解决了竖直方向的失效模式，并减轻了水平 lag，同时保持了 estimator 结构简单、可解释。
