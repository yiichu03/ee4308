# Estimator Requirements From `proj2.md`

This note extracts the main estimator-related requirements from the handout and adds a short Chinese explanation after each point.

## 1. Node Role

English:
The estimator node implements a simplified Kalman Filter algorithm for estimating the drone's pose and twist using sensor data. Roll and pitch are ignored in the simplified motion model.

中文翻译：
estimator 节点实现一个简化的 Kalman Filter，用传感器数据估计无人机的位姿和速度。这个简化运动模型忽略了 roll 和 pitch。

讲解：
老师这里已经限定了整体方向，所以你答辩时可以明确说自己遵循的是 simplified KF，而不是完整姿态耦合模型。

English:
The estimator node publishes the estimated pose and twist into `/drone/odom`.

中文翻译：
estimator 节点会把估计出的 pose 和 twist 发布到 `/drone/odom`。

讲解：
这说明 estimator 不是孤立模块，它的输出直接给 behavior 和 controller 用。

## 2. Demonstration Settings

English:
`verbose` must be set to `true` for the demonstration.

中文翻译：
演示时 `verbose` 必须设为 `true`。

讲解：
如果老师问 demo 时为什么终端会打印这么多信息，你可以说这是 handout 明确要求的。

English:
`use_ground_truth` can be set to `true` to troubleshoot behavior and controller.

中文翻译：
`use_ground_truth` 可以设成 `true`，让 behavior 和 controller 用 ground truth，便于排查别的模块问题。

讲解：
这个点很好用来解释你们是怎么隔离 estimator 问题和非 estimator 问题的。

## 3. Prediction Requirements

English:
Extend the Lab 2 IMU prediction to the other states in `x`, `y`, and yaw.

中文翻译：
把 Lab 2 里的 IMU prediction 扩展到 `x`、`y` 和 yaw。

讲解：
也就是说，prediction 的主干本来就是 handout 要求你完成的内容，不是你额外发明的新结构。

English:
`msg.orientation` should not be used.

中文翻译：
不允许使用 `msg.orientation`。

讲解：
你必须明确说明 yaw 是由角速度积分得到，而不是直接从 IMU orientation 拿来的。

English:
By ignoring roll and pitch, the accelerations in the world frame are obtained by rotating the IMU `x/y` acceleration using yaw.

中文翻译：
在忽略 roll 和 pitch 的前提下，世界坐标系中的 `x/y` 加速度由 yaw 旋转机体系加速度得到。

讲解：
这是你报告里 prediction equation 的直接来源。

English:
For the `z` axis, determine whether gravity should be added or subtracted.

中文翻译：
对 `z` 轴，要判断重力项应该加还是减。

讲解：
你的最终做法是减去重力。

English:
Implement the process model for yaw and yaw velocity.

中文翻译：
实现 yaw 和 yaw velocity 的过程模型。

讲解：
你的实现就是 yaw 由 `omega_z * dt` 推进，并维护 `Xa = [yaw, yaw_rate]`。

## 4. Correction Requirements

English:
The correction stage is asynchronous and occurs when a new sensor measurement becomes available.

中文翻译：
correction 是异步发生的，每当有新的传感器测量到来时就进行一次修正。

讲解：
这正是 callback 结构的理论依据。你答辩时可以把“异步 correction”与 ROS subscription callback 对上。

English:
Multiple corrections from different sensors can occur between predictions.

中文翻译：
在两次 prediction 之间，可以发生多次来自不同传感器的 correction。

讲解：
说明 estimator 不要求“先预测再整齐地一次性融合所有传感器”，而是允许异步串行更新。

## 5. Sensor-Specific Requirements

### Sonar

English:
Implement the correction for the `z` states when a sonar message is received.

中文翻译：
当 sonar 消息到来时，实现对 `z` 状态的 correction。

讲解：
老师原始 handout 只要求基本的 sonar correction；你后面再加 gating，是在这个基础上做改进。

### GPS

English:
Implement the correction for the `x`, `y`, `z` states when a GPS message is received.

中文翻译：
当 GPS 消息到来时，实现对 `x`、`y`、`z` 状态的 correction。

讲解：
基础要求是位置修正；你后来的 pseudo-velocity 和 forward compensation 是对这个基本 GPS correction 的增强。

English:
Convert GPS data from latitude, longitude, altitude to ECEF, then to local NED, then to the world frame.

中文翻译：
把 GPS 的经纬高先转成 ECEF，再转到本地 NED，最后转到世界坐标系。

讲解：
这是你 `callbackSubGPS_()` 里最容易被问公式来源的一段。

### ECEF

English:
Implement `getECEF_()` based on the WGS84 ellipsoid equations.

中文翻译：
根据 WGS84 椭球模型公式实现 `getECEF_()`。

讲解：
如果老师问你 GPS 世界坐标是怎么算出来的，这就是最基础的起点。

### Magnetometer

English:
Implement the heading correction when a message from the magnetic compass arrives.

中文翻译：
当磁力计消息到来时，实现 heading correction。

讲解：
这个部分在你最终结果里很稳，所以它是解释 yaw 准确度的重要支撑。

### Barometer

English:
This is optional for teams of 3, but the handout explicitly notes that the calculated height can contain a significant bias, and that augmenting the `z` state may be more useful than naively subtracting an initial offset.

中文翻译：
这个部分对 3 人组是 optional，但 handout 明确指出 barometer 高度可能带有显著 bias，而且扩展 `z` 状态去估计 bias，往往比简单减去初始偏移更合理。

讲解：
这点非常关键，因为它说明你加 `b_baro` 不是瞎改，而是老师文档本来就给出的高质量方向。

## 6. Report Expectations Relevant To Estimator

English:
In a good report, the existing algorithms are examined in detail and simple solutions are proposed to significantly improve the algorithms.

中文翻译：
一份好的报告会详细分析原有算法，并提出简单但能显著改善算法的方案。

讲解：
所以你的报告重点不能只是“我实现了什么”，还要讲“原方法有什么问题，我的改进为什么有用”。

English:
Experiments and methodologies to tune parameters are well designed and justified.

中文翻译：
参数实验和调参方法应当设计得合理，并且有充分理由。

讲解：
这就是为什么你在答辩时最好强调你们用的是多次实验和稳定性判断，而不是一次最优结果。

English:
The narrative is concise and clear.

中文翻译：
叙述要简洁清楚。

讲解：
答辩时也一样，少绕，多讲问题、改进、证据链。

## 7. One-Line Takeaway

中文可直接记：
老师 handout 对 estimator 的基本要求是把 simplified KF 的 prediction 和各传感器 correction 做出来，而我在这个基础上重点强化了 `z` 轴可信度和 `x/y` lag 问题，所以我的答辩主线应该是“遵循 handout 基础结构，然后针对观测到的问题做了有证据支持的改进”。
