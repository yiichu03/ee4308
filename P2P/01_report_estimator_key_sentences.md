# Estimator Report Key Sentences

This note rewrites the main sentences from the final report's estimator section into short Chinese understanding.

## 1. Structure

English:
We keep the axis-by-axis structure from the handout. Instead of one large coupled filter, we use four small filters.

中文理解：
我们没有做一个完整的大状态 EKF，而是沿用老师 handout 的分轴思路。这样更容易实现、更容易调参，也更容易把问题拆开分析。

English:
The vertical state is augmented to `X_z = [z, v_z, b_baro]^T`.

中文理解：
`z` 轴不是只估计高度和竖直速度，还额外估计 barometer bias。这样 barometer 的偏置不会直接把高度状态拉歪。

## 2. Prediction

English:
The IMU callback performs the prediction step.

中文理解：
预测完全靠 IMU 来推进状态。也就是说，在没有新传感器修正时，状态会根据上一次状态和 IMU 输入继续往前推。

English:
We rotate the horizontal acceleration into the world frame using the current yaw.

中文理解：
IMU 测到的是机体系加速度，但我们要估计世界坐标系里的 `x/y`，所以必须先用当前 yaw 做旋转。

English:
For the vertical axis, we first remove gravity.

中文理解：
`z` 轴的 IMU 加速度先减去重力，再用于预测，否则高度会一直被重力项污染。

English:
The yaw is predicted from the IMU angular velocity, and the angle is wrapped after each update.

中文理解：
yaw 不是直接用 IMU orientation，而是用角速度积分得到，并且每次更新后都做 angle wrapping，避免角度跳变。

## 3. Correction Models

English:
The correction step is separated by sensor.

中文理解：
不同传感器各自异步到来，各自触发一次 correction。这个设计和 ROS 的 callback 形式完全一致。

English:
GPS updates position, sonar updates height, barometer updates `z + bias`, and magnetometer updates yaw.

中文理解：
GPS 主要负责位置，sonar 负责近地高度，barometer 负责有偏差的高度信息，magnetometer 负责 yaw 修正。每个传感器都只修自己最有意义的状态。

English:
We keep the Joseph-form covariance update for numerical stability.

中文理解：
普通协方差更新更简洁，但 Joseph form 数值上更稳，不容易因为计算误差把协方差弄坏。

## 4. Improvements

English:
In early runs, sonar helped near the ground but became unreliable higher up.

中文理解：
sonar 不是一直没用，而是“近地面有用，高处容易害人”。所以最后方案不是删掉 sonar，而是做 gating。

English:
We only accept the sonar update when the predicted height is at most `3.8 m` and the innovation is at most `1.0 m`.

中文理解：
只在两个条件同时满足时才信 sonar：当前预测高度不能太高，测量和预测不能差太离谱。这个逻辑是为了拒绝明显不可信的 sonar 更新。

English:
Treating the barometer as a direct measurement of `z` gave a biased height estimate.

中文理解：
如果把 barometer 直接当成高度真值，bias 会直接进入 `z` 状态，导致高度整体偏掉。

English:
The filter can absorb the slow offset into the bias state instead of pushing it into the altitude estimate.

中文理解：
加入 bias state 以后，barometer 的慢偏差会被单独吸收到 `b_baro` 里，而不是污染高度本身。

English:
Our main problem in the horizontal plane was lag.

中文理解：
`x/y` 方向的核心问题不是发散，也不是剧烈震荡，而是“稳定但落后”。这个判断决定了后面改进的方向。

English:
The estimator computes a planar pseudo-velocity from two consecutive GPS positions.

中文理解：
GPS 本身不给速度，所以我们用相邻两次 GPS 位置差分出一个伪速度，再拿它去修正 `v_x` 和 `v_y`。

English:
We forward-compensate the GPS position when the GPS message is slightly stale.

中文理解：
如果 GPS 稍微滞后，我们先用当前估计速度把它往前补一点，再做修正，这样可以减少“用旧位置修正当前状态”带来的 lag。

## 5. Tuning and Results

English:
The final values were chosen to improve the observed height error and horizontal lag, not to copy one lucky run.

中文理解：
参数不是挑一次最好看的 run，而是挑重复实验里比较稳定、能解释得通的值。

English:
The main conclusion is that `z` and yaw are already quite accurate, while `x` and `y` are still the main limitation.

中文理解：
最后的结论一定要讲清楚：你最成功的是 `z` 和 yaw，不要硬说整个 estimator 已经全面完美；`x/y` 仍然是剩余短板。

## 6. One-Paragraph Summary

中文可直接复述：
我在 estimator 上保留了 handout 的分轴 simplified Kalman filter 结构，没有改成 full 3D EKF。主要问题一开始集中在两个地方：`z` 轴会被不可靠的 sonar 和带偏差的 barometer 拉坏，`x/y` 则主要表现为 lag。对应地，我做了四个关键改进：sonar gating、barometer bias state、GPS pseudo-velocity correction、GPS forward compensation。最终结果表明 `z` 和 yaw 已经比较准确，系统能稳定支持整段任务，但平面方向仍有一定 lag，这也是当前 estimator 的主要限制。
