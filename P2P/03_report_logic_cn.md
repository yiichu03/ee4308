# Estimator 报告逻辑整理

这份文档只用中文整理我在答辩时应该怎么讲 estimator，不追求逐句对应报告，而是追求逻辑顺。

## 1. 开头先定框架

我做的 estimator 不是 full 3D EKF，而是沿用 handout 的 simplified Kalman filter 分轴结构。

原因有三个：

- 这是课程 handout 给出的基本框架，符合题目方向。
- 分轴结构更容易实现，也更容易调试。
- 我们观察到的问题本来就是分开的：`z` 轴问题和 `x/y` lag 问题，不需要上来就做大耦合模型。

所以我一开始的整体策略不是“推翻原结构”，而是“保留基本结构，针对具体问题做针对性改进”。

## 2. 再讲 baseline 到底有什么问题

我需要明确说出 baseline 问题，不要只说“效果一般”。

我观察到的主要问题有两类：

- `z` 轴问题：sonar 在近地面有帮助，但在较高处会变得不可靠；barometer 又存在明显 bias，所以高度估计会被带偏。
- `x/y` 问题：平面方向通常不是发散，而是明显 lag，也就是估计跟得上大体趋势，但总是落后于真实运动。

这两个问题决定了后面的改进方向。

## 3. Prediction 部分怎么讲

prediction 用 IMU 做。

`x/y` 方向：

- IMU 测到的是机体系加速度；
- 但状态是世界系中的 `x/y` 位置和速度；
- 所以要先用 yaw 把机体系加速度旋转到世界系；
- 然后用 constant-acceleration model 推进 `x/vx` 和 `y/vy`。

`z` 方向：

- `z` 轴先去掉重力；
- 再用同样的常加速度形式预测 `z/vz`；
- 另外我对 `z` 向加速度做了限幅，避免 IMU 瞬时异常把 `z` 预测直接拉坏。

yaw：

- yaw 由角速度积分得到；
- 不用 IMU orientation；
- 每次更新后做 angle wrapping。

这一部分答辩时不要讲太长，重点是说明我理解 prediction 模型，而不是把所有公式背一遍。

## 4. Correction 部分怎么讲

correction 是按传感器分开的，而且是异步做的。

各传感器分工如下：

- GPS 修正位置。
- Sonar 修正高度。
- Barometer 提供带 bias 的高度信息。
- Magnetometer 修正 yaw。

我这里最该强调的是：不同传感器不是“谁都一样”，而是各自只修正最有意义的状态。

## 5. 真正的亮点一：sonar gating

这是我最重要的改进之一。

我观察到：

- sonar 在低空时对高度很有帮助；
- 但在较高高度或者测量明显不一致时，会把 `z` 轴拉坏。

所以我最后不是删除 sonar，而是给 sonar 加了 gating。

只有在下面条件满足时才做 sonar correction：

- 当前预测高度不超过 `3.8 m`；
- sonar innovation 不超过 `1.0 m`。

这个改进的核心逻辑是：

- 保留 sonar 在近地面的优点；
- 拒绝它在不可信条件下造成的破坏。

如果老师问“为什么不是直接不用 sonar”，这个就是最直接的回答。

## 6. 真正的亮点二：barometer bias state

这是另一个最重要的改进。

barometer 的问题不是“完全没信息”，而是它测的是“真实高度 + 偏置”。

如果直接把 barometer 当成 `z` 的观测，bias 会直接污染高度状态。

所以我把 `z` 轴状态扩展成：

`Xz = [z, vz, b_baro]`

然后 barometer 的观测模型变成：

`z_baro = z + b_baro`

这样做以后：

- 高度状态负责真实高度；
- bias 状态负责吸收慢偏差；
- barometer 不会再把整个 `z` 估计整体拉偏。

这个点特别适合答辩，因为 handout 其实已经暗示了这个方向，所以你可以说这不是随意加状态，而是基于题目提示和实验现象做出的合理扩展。

## 7. 平面方向为什么要做 GPS velocity correction

我在平面方向看到的主要问题是 lag。

这个判断很重要，因为它说明问题不是“滤波器发散”，而是“动态响应跟不上”。

GPS 只提供位置，不直接提供速度。
所以我用两次连续 GPS 位置做差，得到一个 pseudo-velocity，再把它拿来保守地修正 `vx` 和 `vy`。

这件事的意义是：

- 给平面速度状态增加一个弱但有用的外部约束；
- 减少 `x/y` 状态持续落后的问题。

这里答辩时一定要强调“保守”：

- 会做平滑；
- 会限制时间间隔；
- 会限制 innovation；
- 不是无条件强行相信 GPS 差分速度。

## 8. 为什么还要做 GPS forward compensation

我还做了 GPS forward compensation。

动机是：

- GPS 到来的时刻，可能对应的是稍早一点的状态；
- 如果拿这个“旧位置”直接修正当前估计，就会进一步加重 lag。

所以我在 GPS 延迟较小且可接受时，用当前估计速度把 GPS 位置向前补偿一点，再拿去做 correction。

这个改进的逻辑非常直白：

- 不是改 GPS 本身；
- 而是减少时序不对齐导致的系统性落后。

## 9. 参数部分怎么讲才不虚

答辩时不要把参数背成一串数字，而是按逻辑讲。

可以这样讲：

- `var_imu_z` 设大，是因为不想让竖直方向过度依赖 IMU prediction。
- `var_sonar` 设得比较小，是因为在通过 gating 后，近地 sonar 是可信的强修正。
- `var_gps_x/y` 设得比以前更小，是为了减轻平面方向 lag。
- pseudo-velocity 的相关参数设得比较保守，是因为 GPS 差分速度本身噪声不小，只能轻量使用。

也就是说，参数不是单独存在的，它们都服务于前面已经讲清楚的问题和改进。

## 10. 结果部分怎么收尾

收尾时一定要诚实，而且要把优势和限制都说清楚。

最好的收尾方式是：

- `z` 和 yaw 已经比较准确；
- `z` 轴前后对比提升非常明显；
- estimator 能稳定支撑完整任务；
- 但 `x/y` 仍然存在一定 lag，是当前剩余的主要限制。

这比说“我们的 estimator 很好”更有说服力。

## 11. 我答辩时的一分钟版本

我在 estimator 上保留了 handout 的 simplified Kalman filter 分轴结构，没有改成 full 3D EKF，因为这种结构更符合课程要求，也更方便定位问题。实际实验里，我看到两个核心问题：`z` 轴会被不可靠 sonar 和有偏差的 barometer 拉坏，而 `x/y` 主要表现为 lag。针对这两个问题，我做了四个关键改进：第一，给 sonar 加 gating，只在低空且 innovation 合理时才使用；第二，把 barometer bias 加入 `z` 状态，让 barometer 的慢偏差不会直接污染高度；第三，用连续 GPS 位置构造 pseudo-velocity，去保守修正 `vx` 和 `vy`；第四，对稍微滞后的 GPS 做 forward compensation，减少时序不对齐导致的 lag。最终结果说明 `z` 和 yaw 已经比较准确，系统能稳定完成整段任务，但平面方向仍然有一定 lag，这也是当前 estimator 的主要限制。
