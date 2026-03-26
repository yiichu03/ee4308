# Lab 2 Canvas Questions

This note is for preparing the Canvas `L2 Questions` quiz.  
The answers below are written to match the current implementation in [estimator.cpp](/home/liuyi/projects/ee4308_course/src/ee4308_drone/src/estimator.cpp#L137) and the tuned parameters in [proj2.yaml](/home/liuyi/projects/ee4308_course/src/ee4308_bringup/params/proj2.yaml#L33).

## Question 1

### Original English
Determine the expression for $a_{z,k}$.

### 中文翻译
求出 $a_{z,k}$ 的表达式。

### English Answer
The expression is

```text
a_{z,k} = u_{z,k} - g
```

where $u_{z,k}$ is the IMU `z`-axis linear acceleration measurement and $g$ is the gravitational acceleration.

In my code, this is implemented as:

```cpp
double az = msg.linear_acceleration.z - GRAVITY;
```

This sign was checked in simulation. When the drone is stationary, using `- GRAVITY` gives a much more reasonable `z` estimate and keeps the vertical velocity close to zero.

### 中文答案
表达式是：

```text
a_{z,k} = u_{z,k} - g
```

其中，$u_{z,k}$ 是 IMU 在 `z` 轴测到的线加速度，$g$ 是重力加速度。

在我的代码里，对应的是：

```cpp
double az = msg.linear_acceleration.z - GRAVITY;
```

这个符号我也用仿真验证过。无人机静止时，使用 `- GRAVITY` 得到的高度估计更合理，而且竖直速度会更接近 0。

## Question 2

### Original English
What are the matrix / vector sizes for $\mathbf{H}$, the innovation, the innovation covariance, and $\mathbf{K}$?

### 中文翻译
$\mathbf{H}$、innovation、innovation covariance 和 $\mathbf{K}$ 的矩阵 / 向量维度分别是多少？

### English Answer
For Lab 2, the vertical state is

```text
Xz = [z, z_dot]^T
```

so the state size is `2 x 1`, and the sonar measurement is a scalar.

Therefore:

- $\mathbf{H}$ is `1 x 2`
- innovation `(y - H Xz)` is `1 x 1`
- innovation covariance `(H P H^T + R)` is `1 x 1`
- $\mathbf{K}$ is `2 x 1`

This matches the code:

```cpp
Eigen::RowVector2d H;   // 1 x 2
Eigen::Vector2d K;      // 2 x 1
```

### 中文答案
在 Lab 2 里，竖直方向状态是：

```text
Xz = [z, z_dot]^T
```

所以状态维度是 `2 x 1`，而 sonar 量测是一个标量。

因此：

- $\mathbf{H}$ 是 `1 x 2`
- innovation `(y - H Xz)` 是 `1 x 1`
- innovation covariance `(H P H^T + R)` 是 `1 x 1`
- $\mathbf{K}$ 是 `2 x 1`

这也和代码写法一致：

```cpp
Eigen::RowVector2d H;   // 1 x 2
Eigen::Vector2d K;      // 2 x 1
```

## Question 3

### Original English
In order to determine $\sigma_{snr,z}^2$, one way is to make the drone stationary and collect about 100 samples from the sonar sensor and determine the variance from the samples. However, the drone may slowly drift upwards. In this situation, concisely describe your process of determining $\sigma_{snr,z}^2$ by using a best fit line and further tuning. An essay is **not** expected for this answer.

### 中文翻译
为了确定 $\sigma_{snr,z}^2$，一种方法是让无人机静止，并从 sonar 传感器采集大约 100 个样本，然后直接由样本求方差。  
但无人机可能会慢慢向上漂移。  
在这种情况下，请简要说明你如何通过 best fit line 和后续调参来确定 $\sigma_{snr,z}^2$。这个问题不需要写成作文。

### English Answer
My process is:

1. Keep the drone as stationary as possible and record about 100 sonar samples.
2. Fit a best fit line to sonar reading versus time, and treat that line as the slow drift trend.
3. Subtract the fitted line from the samples to get the residuals.
4. Compute the variance of the residuals and use it as the initial estimate of $\sigma_{snr,z}^2$.
5. Then tune `var_sonar` further in simulation. If the estimate follows sonar noise too much, increase `var_sonar`. If the `z` estimate drifts too much and the correction is too weak, decrease `var_sonar`.

In my experiment, the best fit line method gave an initial estimate of about

```text
var_sonar ≈ 0.000312907
```

After further tuning in simulation, I used:

```text
var_sonar = 0.03
```

Together with `var_imu_z = 20.0`, this gave a more stable `z` estimate.

### 中文答案
我的做法是：

1. 先让无人机尽量保持静止，记录大约 100 个 sonar 样本。
2. 对 sonar 读数随时间做一条 best fit line，把它看成缓慢漂移的趋势。
3. 用样本减去拟合线，得到残差。
4. 对残差求方差，把它作为 $\sigma_{snr,z}^2$ 的初始估计。
5. 然后再在仿真里继续微调 `var_sonar`。如果估计结果跟着 sonar 噪声抖得太厉害，就把 `var_sonar` 调大；如果 `z` 方向漂移明显、校正太弱，就把 `var_sonar` 调小。

在我的实验里，best fit line 得到的初始估计大约是：

```text
var_sonar ≈ 0.000312907
```

之后再经过仿真微调，我最后采用的是：

```text
var_sonar = 0.03
```

并且配合 `var_imu_z = 20.0`，这组参数得到的 `z` 轴估计更稳定。
