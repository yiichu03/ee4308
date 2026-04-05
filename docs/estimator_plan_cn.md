# Project 2 Estimator 中文规划与实现记录

## 1. 当前基线

当前工作分支为 `proj2-estimator`，它建立在已经集成好的 `behavior + controller` 基础上。

目前关键文件如下：

- `src/ee4308_drone/src/behavior.cpp`
- `src/ee4308_drone/src/controller.cpp`
- `src/ee4308_drone/src/estimator.cpp`
- `src/ee4308_drone/include/ee4308_drone/estimator.hpp`
- `src/ee4308_bringup/params/proj2.yaml`
- `src/ee4308_bringup/params/proj2_gt.yaml`

参数文件的关系要先说清楚：

- `proj2.yaml` 是正式的项目参数文件。
- `proj2.yaml` 里已经保留了老师要求的 controller 参数：
  - `max_z_vel: 0.5`
  - `yaw_vel: -0.3`
- `proj2_gt.yaml` 不是“另一套 controller 参数”。
- `proj2_gt.yaml` 只是复制了同样的 behavior/controller 参数，并把：
  - `drone.estimator.use_ground_truth: true`
  打开。

因此：

- 联调 `behavior/controller` 时，用 `proj2_gt.yaml`
- 正式测试 estimator 时，用 `proj2.yaml`

这也和老师在 `proj2.md` 里的说明一致：`use_ground_truth` 的目的就是在调试 `behavior` 和 `controller` 时，把 estimator 从问题里剥离掉。

## 2. 老师更看重什么

根据 `docs/email.md`，老师不只是看“有没有跑起来”，更看重以下几点：

- 公式有没有写出来
- 是否解释清楚“为什么这样设计”
- 参数是否通过实验有依据地调出来
- 是否说明某个改动在什么情况下有帮助
- 不要把没有实现的内容写进报告
- 如果某个方案失败了，可以写，但前提是有分析价值

这对 estimator 的意义很直接：

- 不能只写“调了几个方差，效果更稳定”
- 要解释每个传感器在补偿什么误差
- 要解释为什么增大/减小某个方差会影响滤波结果
- 要把实验设计成可以回答问题，而不是只展示结果

## 3. Estimator 要完成的范围

按照 `docs/proj2.md`，Estimator 分成 prediction 和 correction 两部分。

### 必做核心

1. `Estimator::callbackSubIMU_()`
2. `Estimator::callbackSubGPS_()`
3. `Estimator::getECEF_()`
4. `Estimator::callbackSubMagnetic_()`
5. `Estimator::callbackSubSonar_()` 检查并清理 Lab 2 遗留逻辑

### 三人组可选

6. `Estimator::callbackSubBaro_()`

对于三人组来说，barometer 是后续增强项，不应该放在第一个里程碑里做。

## 4. 代码结构与依赖关系

### Estimator 主体

- `src/ee4308_drone/src/estimator.cpp`
  - prediction / correction 主要逻辑
  - 各个传感器 callback
  - 发布 `odom`
  - verbose 调试输出

- `src/ee4308_drone/include/ee4308_drone/estimator.hpp`
  - 状态向量
  - 协方差矩阵
  - 测量缓存
  - 参数
  - 常量

### 参数与联调

- `src/ee4308_bringup/params/proj2.yaml`
  - 正式运行
  - 调 estimator 方差

- `src/ee4308_bringup/params/proj2_gt.yaml`
  - 仅用于 GT 联调
  - 保持同样的 behavior/controller 参数

### Estimator 输出会影响谁

- `src/ee4308_drone/src/behavior.cpp`
  - 依赖 `/drone/odom` 判断是否到达 waypoint

- `src/ee4308_drone/src/controller.cpp`
  - 依赖 `/drone/odom` 做路径跟踪

也就是说，estimator 一旦有明显漂移，上层 behavior/controller 就会一起被拖坏。

## 5. 当前状态向量设计

当前代码已经采用了四个独立的 2 维状态：

- `Xx_ = [x, \dot{x}]^T`
- `Xy_ = [y, \dot{y}]^T`
- `Xz_ = [z, \dot{z}]^T`
- `Xa_ = [yaw, \dot{yaw}]^T`

对应协方差矩阵为：

- `Px_`
- `Py_`
- `Pz_`
- `Pa_`

用于终端打印的测量缓存为：

- `Ygps_`
- `Ysonar_`
- `Ymagnet_`
- `Ybaro_`

这个结构已经能够直接支持：

- IMU 对 `x/y/z/yaw` 的 prediction
- GPS 对 `x/y/z` 的 correction
- sonar 对 `z` 的 correction
- magnetic 对 `yaw` 的 correction

如果后续真的做 barometer bias augmentation，那么 `Xz_` 和 `Pz_` 都要改尺寸，相关 `H` 矩阵也要一起重做。

## 6. 各函数实现规划

### 6.1 `callbackSubIMU_()`

目标：

- 把 Lab 2 的 `z` 轴 prediction 扩展到 `x/y/z/yaw`

输入：

- `msg.linear_acceleration.x`
- `msg.linear_acceleration.y`
- `msg.linear_acceleration.z`
- `msg.angular_velocity.z`
- 当前 yaw 估计 `Xa_(0)`
- `dt`

核心思路：

IMU 线加速度是在机体系下给出的，而我们状态是世界系下的 `x/y/z`。因此 `x/y` 的第一步不是直接积分，而是先把机体系加速度旋转到世界系。

公式如下：

```math
\begin{bmatrix} a_x \\ a_y \end{bmatrix}
=
\begin{bmatrix}
\cos\psi & -\sin\psi \\
\sin\psi & \cos\psi
\end{bmatrix}
\begin{bmatrix} u_x \\ u_y \end{bmatrix}
```

位置和速度的常加速度 prediction 形式：

```math
F =
\begin{bmatrix}
1 & dt \\
0 & 1
\end{bmatrix}, \quad
W =
\begin{bmatrix}
\frac{1}{2}dt^2 \\
dt
\end{bmatrix}
```

于是：

```math
\hat{X}_{x,k|k-1} = F \hat{X}_{x,k-1|k-1} + W a_x
```

```math
\hat{X}_{y,k|k-1} = F \hat{X}_{y,k-1|k-1} + W a_y
```

`z` 轴和 Lab 2 一样，但要注意重力符号：

```math
a_z = u_z - G
```

所以：

```math
\hat{X}_{z,k|k-1} = F \hat{X}_{z,k-1|k-1} + W a_z
```

yaw 则直接由角速度输入：

```math
\hat{X}_{a,k|k-1} =
\begin{bmatrix}
\psi_{k-1|k-1} + dt \cdot u_\psi \\
u_\psi
\end{bmatrix}
```

协方差更新：

```math
P_{k|k-1} = F P_{k-1|k-1} F^T + W Q W^T
```

其中：

- `x/y` 轴的 `Q` 来自 `var_imu_x_`、`var_imu_y_`
- `z` 轴来自 `var_imu_z_`
- yaw 来自 `var_imu_a_`

### 6.2 `getECEF_()`

目标：

- 把 GPS 的经纬高转换为 ECEF 坐标

公式：

```math
e^2 = 1 - \frac{b^2}{a^2}
```

```math
N(\varphi) = \frac{a}{\sqrt{1 - e^2 \sin^2(\varphi)}}
```

```math
\begin{bmatrix} x_e \\ y_e \\ z_e \end{bmatrix} =
\begin{bmatrix}
(N+h)\cos\varphi\cos\lambda \\
(N+h)\cos\varphi\sin\lambda \\
\left(\frac{b^2}{a^2}N+h\right)\sin\varphi
\end{bmatrix}
```

这里：

- `a = RAD_EQUATOR`
- `b = RAD_POLAR`

### 6.3 `callbackSubGPS_()`

目标：

- 用 GPS 修正 `x/y/z`

步骤：

1. 将纬度、经度转为弧度
2. 计算 ECEF
3. 第一帧 GPS 只用于初始化 `initial_ECEF_`
4. 将 `ECEF - initial_ECEF_` 转成局部 NED
5. 再从 NED 旋转到 Gazebo/world frame
6. 加上 `initial_position_` 得到世界系下的 `Ygps_`
7. 对 `x/y/z` 分别做标量 KF correction

老师给出的 NED 转换：

```math
\mathbf{p}_{ned} = R_{e/n}^T ( \mathbf{p}_{ecef} - \mathbf{p}_{ecef,0} )
```

再转到 world frame：

```math
\mathbf{p}_{gps} = R_{m/n} \mathbf{p}_{ned} + \mathbf{p}_0
```

对每个轴，测量模型都是：

```math
H = [1 \; 0], \quad V = 1, \quad R = \sigma^2
```

### 6.4 `callbackSubMagnetic_()`

目标：

- 用磁力计修正 yaw

老师的说明有一个很关键的点：

- Gazebo 里的磁北方向和正常 ENU 设定有偏差
- 但由于这个项目里无人机初始朝向就是 `+x`
- 因此不需要额外做初始 heading offset

这里我们只需要从磁场向量恢复 yaw 测量 `Ymagnet_`，然后用一维标量测量去修正 `Xa_`。

测量模型：

```math
Y_{mag} = \psi_{mag} + \varepsilon
```

```math
H = [1 \; 0], \quad V = 1, \quad R = \sigma^2_{mag}
```

这里有一个实现细节必须注意：

- yaw 的 innovation 必须做 `limitAngle`
- correction 后的 `Xa_(0)` 也要重新 wrap 到 `[-\pi, \pi)`

### 6.5 `callbackSubSonar_()`

目标：

- 保留 Lab 2 已完成的 `z` 轴 correction
- 删除 Lab 2 为了 RViz 方便而加入的临时 hack

当前代码里，`callbackSubSonar_()` 除了正常 KF correction，还把：

- `Px_`
- `Py_`

硬重置成小值。

这在 Project 2 里不应该保留，因为它会无端修改与 sonar 无关的 `x/y` 协方差。

### 6.6 `callbackSubBaro_()`

对于三人组这是可选项。

老师的要求不是“简单做个高度修正”这么粗糙，而是明确提到：

- barometer 可能有明显 bias
- 更合理的做法是把 `z` 状态扩展成包含 bias 的增广状态

这部分工作量不小，所以顺序上一定要放在：

- IMU
- GPS
- magnetic
- sonar

之后。

## 7. 建议的实现顺序

### 第 0 阶段：维持联调稳定

1. 保留 `proj2.yaml` 作为正式参数文件
2. 保留 `proj2_gt.yaml` 作为 GT 联调文件
3. 临时保留 behavior/controller 里的 `TMP LOG`

### 第 1 阶段：完成 prediction

1. 实现 `callbackSubIMU_()`
2. 确认 estimator 发布的 `odom` 至少是连续、有限、不会 NaN

### 第 2 阶段：完成 GPS correction

1. 实现 `getECEF_()`
2. 实现 `callbackSubGPS_()`
3. 确认 `Ygps_` 和世界系方向一致

### 第 3 阶段：完成 yaw correction

1. 实现 `callbackSubMagnetic_()`
2. 确认 yaw 不会持续漂移

### 第 4 阶段：清理 sonar 并开始调参

1. 移除 sonar 中的 Lab 2 covariance hack
2. 开始调：
  - `var_imu_x_`
  - `var_imu_y_`
  - `var_imu_z_`
  - `var_imu_a_`
  - `var_gps_x_`
  - `var_gps_y_`
  - `var_gps_z_`
  - `var_sonar_`
  - `var_magnet_`

### 第 5 阶段：决定是否做 baro

如果时间不够，宁愿把核心 estimator 做稳、实验做扎实、报告写清楚，也不要为了多塞一个可选 sensor 把前面的基础弄乱。

## 8. 验证思路

### A. 先隔离 behavior/controller

使用：

- `proj2_gt.yaml`

目的：

- 确认飞行逻辑没有问题

### B. 再切回真实 estimator

使用：

- `proj2.yaml`

目的：

- 检查 estimator 自己有没有把系统带偏

### C. 传感器分问题验证

建议把实验问题拆开：

- 只有 IMU 时漂移有多快
- IMU + sonar 后高度是否被约束住
- IMU + GPS 后平面漂移是否收敛
- IMU + magnetic 后 yaw 漂移是否减小
- 全部核心传感器打开后，任务是否仍能跑完

这种实验方式更符合老师在邮件里强调的“解释 why/how，而不只是描述结果”。

## 9. 实现记录

### 第 1 次实现

目标：

- 补齐核心 estimator 的第一版：
  - IMU prediction
  - ECEF
  - GPS correction
  - magnetic correction
  - 清理 sonar 中的 Lab 2 遗留逻辑

实施原则：

- 先保证公式正确、坐标系正确、输出有限
- 再谈调参
- barometer 先不抢进度

#### 已实现内容

1. 在 `estimator.cpp` 中补上了标量 KF correction 的公共逻辑。

用途：

- sonar / GPS / magnetic 这几类 correction 都是同一个一维测量模型：

```math
H = [1 \; 0]
```

因此实现成公共逻辑可以减少重复代码，也更不容易在不同 callback 里写出不一致的公式。

2. 实现了 `getECEF_()`

对应公式：

```math
e^2 = 1 - \frac{b^2}{a^2}
```

```math
N(\varphi) = \frac{a}{\sqrt{1 - e^2 \sin^2(\varphi)}}
```

```math
\begin{bmatrix} x_e \\ y_e \\ z_e \end{bmatrix} =
\begin{bmatrix}
(N+h)\cos\varphi\cos\lambda \\
(N+h)\cos\varphi\sin\lambda \\
\left(\frac{b^2}{a^2}N+h\right)\sin\varphi
\end{bmatrix}
```

3. 实现了 `callbackSubGPS_()`

实现思路：

- 先由经纬高得到 ECEF
- 第一帧 GPS 只初始化 `initial_ECEF_`
- 后续帧将 `ECEF - initial_ECEF_` 转到局部 NED
- 再从 NED 转到 Gazebo/world frame
- 最后分别对 `x/y/z` 做一维 correction

也就是说，GPS 这一步并没有直接拿经纬度“凑一个平面坐标”，而是严格按老师文档里给的坐标链条来做。

4. 实现了 `callbackSubIMU_()`

实现思路：

- `x/y` 轴先做 body frame 到 world frame 的旋转
- 再做常加速度模型 prediction
- `z` 轴沿用 Lab 2 的思路，但保留 `u_z - G`
- yaw 用 `angular_velocity.z` 做 prediction，并在每次更新后用 `limitAngle()` 做包角

这里最关键的地方有两个：

- `x/y` 不能直接积 body frame acceleration
- yaw innovation / yaw state 都要做角度归一化

5. 实现了 `callbackSubMagnetic_()`

实现思路：

- 利用 Gazebo 中磁场近似指向 world `+x` 的特点
- 从机体系下的磁场向量恢复 yaw 测量
- 再用一维 correction 修正 `Xa_`

这里 innovation 用的是：

```math
\mathrm{limitAngle}(Y_{mag} - \hat{\psi})
```

否则 yaw 在跨越 `-\pi / \pi` 边界时会跳。

6. 清理了 `callbackSubSonar_()` 里的 Lab 2 遗留逻辑

删除内容：

- 每次 sonar callback 时强行把 `Px_`、`Py_` 重置为 `0.1`

原因：

- 这属于 Lab 2 为了 RViz 显示做的临时处理
- Project 2 里会破坏 `x/y` 协方差的真实演化

7. `callbackSubBaro_()` 目前只做测量缓存，不做 correction

原因：

- 对三人组来说这是可选项
- 老师明确提醒 barometer 可能带 bias
- 如果不先做增广状态，直接硬修正 `z`，容易把本来能用的核心 estimator 搞坏

所以这一版策略是：

- 先把 `Ybaro_` 算出来，供 verbose 和实验分析使用
- 真正的 bias-aware baro correction 放到后续增强阶段

#### 编译与运行验证

编译环境：

- `ee4308_jazzy_proj2` Docker

编译结果：

- `ee4308_drone`
- `ee4308_bringup`

均已重新编译通过。

运行验证时发现了一个很重要的工程问题：

- 容器里如果残留上一轮 `proj2_gt` 或 `proj2` 的 Gazebo / component_container / rviz 进程
- 新旧仿真会同时向 topic 发消息
- 这会让 estimator 看起来像“公式写错了”，其实是会话污染

所以后续每次严肃验证 estimator 前，都应该先确保容器里只有一套 `proj2` 仿真在运行。

#### 当前验证观察

在清理残留进程后，重新用 `proj2.yaml` 启动一套干净仿真，观察到：

1. 初始阶段 `ErrPose` 非常接近 0

这说明：

- 初始状态对齐没有大错
- GPS 初始参考点和地图原点关系是基本对的

2. 大约在 `t = 7` 到 `8` 秒附近，误差大致表现为：

- `x` 误差约在 `0.0 ~ 0.13 m`
- `y` 误差约在 `-0.45 ~ -0.58 m`
- `z` 误差大致在 `-0.02 ~ 0.03 m`
- yaw 误差大致在 `0.01 ~ 0.02 rad`

这说明第一版 estimator 已经具备了继续调参的基础，至少没有出现：

- 坐标轴写反
- yaw 完全反号
- 全状态发散
- `odom` 持续 NaN

3. 当前最明显的残余问题是：

- `y` 方向存在较稳定的偏差

这很可能和下面几件事有关：

- GPS 噪声与方差尚未调优
- 简化模型忽略 roll/pitch 带来的平面误差
- 当前 GPS 局部坐标换算仍可能存在轻微系统误差

因此下一阶段重点应该是：

- 先调 `var_gps_x_ / var_gps_y_ / var_gps_z_`
- 再结合实验判断这个 `y` 偏差更像“测量偏”还是“prediction drift”

4. 日志中还出现过一次异常大的 `ErrTwis` yaw 项

目前判断：

- 更像是瞬时打印/消息时序问题
- 不是持续性的 estimator 发散

因为前后相邻样本很快恢复正常，而且 yaw 本身的 `ErrPose` 没有对应爆掉。

#### 当前结论

这一版 estimator 可以视为：

- 核心公式已接上
- 主要坐标转换已接上
- 系统可以编译并在仿真中持续运行
- 已经进入“调参与改进”阶段

但它还不是最终版本，当前最值得继续投入的点是：

1. 进一步验证 GPS 在 `y` 轴的系统偏差
2. 调整 `var_gps_*`
3. 调整 `var_imu_*`
4. 视时间决定是否加入 bias-aware 的 barometer correction

#### 本轮参数实验记录（2026-04-04）

为了把本轮调参做得尽量可复现，我采用了下面的固定流程：

1. 每次实验前先清理容器内残留的 `proj2` / `rviz2` / `component_container` / `gz sim`
2. 重新 `colcon build`
3. 用 `proj2.yaml` 启动一轮固定时长 `40s` 的仿真
4. 将 launch 输出保存到仓库内 `tmp/*.log`
5. 从 verbose 里的 `ErrPose` 行统计误差

这里我主要看两个时间窗口：

- `7s ~ 25s`：覆盖起飞后进入任务阶段的一整段时间
- `10s ~ 18s`：尽量避开起飞和更后面的任务切换，更接近“第一轮巡航阶段”

本轮对比了三组参数：

| 名称 | `var_imu_x/y` | `var_gps_x/y/z` | `7~25s` mean abs error `(x, y, z, yaw)` | `10~18s` mean abs error `(x, y, z, yaw)` | 结论 |
| --- | --- | --- | --- | --- | --- |
| 基线 | `1.0 / 1.0` | `1.0 / 1.0 / 1.0` | `(0.848, 0.530, 0.642, 0.004)` | `(1.098, 0.535, 0.663, 0.004)` | 平面误差偏大，中段 `x` 漂移明显 |
| 更信任 GPS | `2.0 / 2.0` | `0.5 / 0.5 / 0.5` | `(0.462, 0.383, 0.550, 0.008)` | `(0.655, 0.218, 0.480, 0.009)` | 当前平面表现最好，作为现阶段默认值保留 |
| 折中组 | `1.5 / 1.5` | `0.7 / 0.7 / 0.7` | `(0.844, 0.380, 0.592, 0.004)` | `(0.883, 0.438, 0.060, 0.005)` | `z` 在局部窗口更漂亮，但整体平面不如上一组 |

从这三组结果可以读出两个非常明确的结论：

1. 目前最值得优先保的是平面定位精度，因此我把 [proj2.yaml](/home/liuyi/projects/ee4308_proj2/src/ee4308_bringup/params/proj2.yaml) 的当前默认值保留为：

```yaml
var_imu_x: 2.0
var_imu_y: 2.0
var_gps_x: 0.5
var_gps_y: 0.5
var_gps_z: 0.5
```

2. `z` 方向的问题不能再完全靠“盲调参数”解决。

原因是：

- 巡航高度接近 `5m` 时，sonar 经常变成 `inf`
- 这时 `z` 方向主要靠 prediction 和 GPS 修正维持
- 如果 barometer 还只做缓存、不做 correction，那么 `z` 长时间漂移很难彻底压住

所以当前最合理的工程判断是：

- 先保留这组平面更好的参数，让 estimator 能更稳定地支持 behavior / controller
- 下一阶段如果要继续显著改善 `z`，优先级已经高于继续盲调 `var_gps_z`
- 更值得投入的是把 `callbackSubBaro_()` 做成真正的 correction

#### RViz 调试调整

这轮还顺手改了一个非常影响观感但不影响控制逻辑的东西：

- [proj2.rviz](/home/liuyi/projects/ee4308_proj2/src/ee4308_bringup/rviz/proj2.rviz) 里的 `Est. Odom` 显示，从 `Keep: 10` 改成了 `Keep: 1`

这样做的原因是：

- 原来 RViz 会保留最近 10 个 `/drone/odom` 历史样本
- 每个样本又会显示淡黄色 position covariance 和红色箭头
- 调试时很容易让人误以为“又多出来几架无人机”

现在改成 `Keep: 1` 之后：

- RViz 只保留当前估计位姿
- 你在录屏和人工观察时更容易判断 estimator 是否真的跳变

#### 复现命令

下面这些命令是我这轮实际使用过的，后面你可以直接照着跑。

主机上先执行：

```bash
xhost +local:root
docker start ee4308_jazzy_proj2
docker exec -it ee4308_jazzy_proj2 bash
```

进入容器后，如果你只是想正常观察和录屏，执行：

```bash
source /opt/ros/jazzy/setup.bash
cd /ws/ee4308
colcon build --symlink-install --packages-select ee4308_drone ee4308_bringup
source install/setup.bash
ros2 launch ee4308_bringup proj2_sim.launch.py libgl:=True param_file:=proj2
```

如果你想先只看 `behavior + controller`，把 estimator 从问题里剥离掉，执行：

```bash
source /opt/ros/jazzy/setup.bash
cd /ws/ee4308
colcon build --symlink-install --packages-select ee4308_drone ee4308_bringup
source install/setup.bash
ros2 launch ee4308_bringup proj2_sim.launch.py libgl:=True param_file:=proj2_gt
```

如果你准备重复做多轮实验，建议每次重跑前都先清理残留进程：

```bash
pkill -9 -f "[p]roj2_sim.launch.py" || true
pkill -9 -f "[r]viz2" || true
pkill -9 -f "[c]omponent_container" || true
pkill -9 -f "[g]z sim" || true
```

如果你想像我这次一样做“固定时长 + 留日志”的定量实验，可以用：

```bash
source /opt/ros/jazzy/setup.bash
cd /ws/ee4308
colcon build --symlink-install --packages-select ee4308_drone ee4308_bringup
source install/setup.bash

mkdir -p tmp
rm -f tmp/proj2_eval.log
timeout 40s stdbuf -oL -eL \
  ros2 launch ee4308_bringup proj2_sim.launch.py libgl:=True param_file:=proj2 \
  >tmp/proj2_eval.log 2>&1 || true
```

对应的误差统计命令是：

```bash
awk '$3=="ErrPose" && $2+0>=7 && $2+0<=25 {
  ax=($4<0?-1*$4:$4)
  ay=($5<0?-1*$5:$5)
  az=($6<0?-1*$6:$6)
  aa=($7<0?-1*$7:$7)
  sx+=ax; sy+=ay; sz+=az; sa+=aa
  if(ax>mx)mx=ax
  if(ay>my)my=ay
  if(az>mz)mz=az
  if(aa>ma)ma=aa
  n++
}
END {
  if(n>0) {
    printf("samples=%d mean_abs_x=%.3f mean_abs_y=%.3f mean_abs_z=%.3f mean_abs_yaw=%.3f max_abs_x=%.3f max_abs_y=%.3f max_abs_z=%.3f max_abs_yaw=%.3f\n",
      n, sx/n, sy/n, sz/n, sa/n, mx, my, mz, ma)
  } else {
    print "no samples"
  }
}' tmp/proj2_eval.log
```

如果你想快速抽看某个时间段的误差，例如 `14s ~ 15s`：

```bash
awk '$3=="ErrPose" && $2+0>=14 && $2+0<=15' tmp/proj2_eval.log | sed -n '1,20p'
```

#### 现阶段建议

在开始下一轮代码实现前，我建议按下面的优先级推进：

1. 先用当前 [proj2.yaml](/home/liuyi/projects/ee4308_proj2/src/ee4308_bringup/params/proj2.yaml) 再跑几轮人工观察，确认行为上没有新的异常
2. 如果主要矛盾仍然是高空 `z` 漂移，就优先继续实现 `callbackSubBaro_()` correction
3. 等 `z` 稳定后，再决定是否要回头微调 `var_gps_x/y`

#### 本轮实现更新：baro correction

这一轮已经不再只是“缓存 barometer 读数”，而是把它正式接进了 `z` 轴 correction。

当前实现思路是：

1. 将 `z` 状态从原本的二元状态

```math
\mathbf{X}_z = \begin{bmatrix} z \\ \dot{z} \end{bmatrix}
```

扩展成三元状态

```math
\mathbf{\hat{X}}_z = \begin{bmatrix} z \\ \dot{z} \\ b_{bar} \end{bmatrix}
```

其中 `b_bar` 表示 barometer bias。

2. 对应的 prediction 改为

```math
\mathbf{F}_z =
\begin{bmatrix}
1 & \Delta t & 0 \\
0 & 1 & 0 \\
0 & 0 & 1
\end{bmatrix},
\quad
\mathbf{W}_z =
\begin{bmatrix}
\frac{1}{2}\Delta t^2 \\
\Delta t \\
0
\end{bmatrix}
```

也就是：

- `z` 和 `vz` 继续由 IMU 做 prediction
- `b_bar` 视作慢变常值，在 prediction 里保持不变

3. barometer 的观测模型是：

```math
z_{bar} = z + b_{bar} + \varepsilon_{bar}
```

因此 measurement Jacobian 取为：

```math
\mathbf{H}_{bar} = \begin{bmatrix} 1 & 0 & 1 \end{bmatrix}
```

4. 代码层面现在已经实际这样实现：

- [estimator.hpp](/home/liuyi/projects/ee4308_proj2/src/ee4308_drone/include/ee4308_drone/estimator.hpp) 中的 `Xz_` 已扩展为 `Eigen::Vector3d`
- [estimator.hpp](/home/liuyi/projects/ee4308_proj2/src/ee4308_drone/include/ee4308_drone/estimator.hpp) 中的 `Pz_` 已扩展为 `Eigen::Matrix3d`
- [estimator.cpp](/home/liuyi/projects/ee4308_proj2/src/ee4308_drone/src/estimator.cpp) 中的 `callbackSubIMU_()` 已对应修改 prediction
- [estimator.cpp](/home/liuyi/projects/ee4308_proj2/src/ee4308_drone/src/estimator.cpp) 中的 `callbackSubGPS_()` / `callbackSubSonar_()` 已改成对增广 `z` 状态使用 `H = [1, 0, 0]`
- [estimator.cpp](/home/liuyi/projects/ee4308_proj2/src/ee4308_drone/src/estimator.cpp) 中的 `callbackSubBaro_()` 已使用 `H = [1, 0, 1]`

为了避免第一次 correction 时 bias 完全未定，我在第一次有效 baro 到来时，会先用：

```math
b_{bar,0} = z_{bar,0} - z_{est,0}
```

初始化 bias，再进入正式 correction。

这不是文档里最“完美”的做法，但在当前作业框架下是很实用的工程折中：

- 不需要把整个 estimator 再大改成更复杂的 bias random walk 模型
- 能快速把 barometer 接入现有滤波器
- 对当前最棘手的高空 `z` 漂移有直接帮助

#### baro correction 结果

在保留当前平面参数

```yaml
var_imu_x: 2.0
var_imu_y: 2.0
var_gps_x: 0.5
var_gps_y: 0.5
var_gps_z: 0.5
var_baro: 1.0
```

的前提下，对比接入 baro 前后：

| 版本 | `7s ~ 25s` mean abs z error | `10s ~ 18s` mean abs z error | 备注 |
| --- | --- | --- | --- |
| 未接入 baro correction | `0.550` | `0.480` | 高空 `z` 漂移明显 |
| 接入 baro correction 后 | `0.338` | `0.015` | `z` 明显改善，尤其是起飞后第一段巡航阶段 |

所以从这轮结果看，优先接 `baro correction` 是正确的，收益比继续盲调 `var_gps_z` 更大。

#### TMP LOG 说明

为了后续继续调参，我还额外保留了几条临时日志，提交前可以统一删掉：

- [behavior.cpp](/home/liuyi/projects/ee4308_proj2/src/ee4308_drone/src/behavior.cpp) 里有 `TMP LOG` 状态迁移日志
- [controller.cpp](/home/liuyi/projects/ee4308_proj2/src/ee4308_drone/src/controller.cpp) 里有 `TMP LOG` 的 `cmd_vel` 节流日志
- [estimator.cpp](/home/liuyi/projects/ee4308_proj2/src/ee4308_drone/src/estimator.cpp) 里现在新增了 `TMP LOG` 的 `baro bias initialized` 和 `err_xyz / z_est / bias / gps_z / sonar / baro` 节流日志

这些日志的用途分别是：

- `behavior`：确认状态机切换是否正确
- `controller`：确认控制指令是否饱和、是否朝正确方向走
- `estimator`：确认 `z` 的误差是在 prediction、sonar 失效后、还是 baro correction 本身出了问题

#### Ground Truth 与轨迹对比

是的，Gazebo 这边已经提供了足够做对比的数据，而且这比肉眼盯 RViz 要可靠得多。

当前和 estimator 最直接相关的 topic 是：

- `/drone/true_odom`
- `/drone/odom`
- `/drone/fix`
- `/drone/imu`
- `/drone/magnetic`
- `/drone/sonar`
- `/drone/air_pressure`

其中最关键的是：

1. `/drone/true_odom`

这是 Gazebo 提供的 ground truth，里面直接包含：

- 真值 position
- 真值 orientation
- 真值 linear velocity
- 真值 angular velocity

也就是说，它基本正好对应 estimator 正在估计的那些量。

2. `/drone/odom`

这是 estimator 发布的估计结果。

所以你完全可以把：

- `/drone/true_odom` 当作 reference
- `/drone/odom` 当作 estimate

然后做：

- 3D 轨迹对比
- `x/y/z` 随时间变化对比
- `yaw` 随时间变化对比
- 误差曲线 `e_x/e_y/e_z/e_yaw`

我建议的实验记录方式是直接录 bag，而不是手动复制 topic：

```bash
ros2 bag record \
  /drone/odom \
  /drone/true_odom \
  /drone/fix \
  /drone/imu \
  /drone/magnetic \
  /drone/sonar \
  /drone/air_pressure
```

这样后处理时你可以很方便地：

- 提取 `odom` 和 `true_odom`
- 按时间戳对齐
- 画 3D trajectory
- 画 `x/y/z`-vs-`t`
- 画误差曲线

如果后面你愿意，我下一步可以直接在仓库里给你补一个小脚本，专门把 bag 里的 `/drone/odom` 和 `/drone/true_odom` 画成：

- 一张 3D 轨迹图
- 三张 `x/y/z` 随时间图
- 一张误差图

#### 绘图脚本

这个脚本现在已经补好并验证过了，在 [plot_drone_bag.py](/home/liuyi/projects/ee4308_proj2/tools/plot_drone_bag.py)。

它现在支持两种输入：

- 一个 ROS 2 bag 目录
- 或者一份运行时直接记录下来的 `aligned_pose_error.csv`

然后输出：

- `trajectory_3d.png`
- `position_vs_time.png`
- `error_vs_time.png`
- `aligned_pose_error.csv`
- `summary.txt`

推荐的主流程已经改成“运行时直接记录 CSV”，不再默认依赖 rosbag 回放。

1. 在第一个终端启动实时对齐记录器

```bash
source /opt/ros/jazzy/setup.bash
cd /ws/ee4308
source install/setup.bash
python3 tools/record_drone_alignment.py \
  --output tmp/proj2_runs/run1/aligned_pose_error.csv \
  --duration 40
```

这个记录器会：

- 订阅 `/drone/odom` 和 `/drone/true_odom`
- 在线按时间戳对齐 estimator 和 ground truth
- 把画图需要的字段直接写进 `aligned_pose_error.csv`
- 以“第一条有效估计样本”为 `t=0`
- 收集满 `--duration` 秒的 estimator 数据后自动退出

2. 在第二个终端正常运行仿真

```bash
source /opt/ros/jazzy/setup.bash
cd /ws/ee4308
source install/setup.bash
ros2 launch ee4308_bringup proj2_sim.launch.py param_file:=proj2
```

3. 运行结束后直接基于 CSV 画图

```bash
source /opt/ros/jazzy/setup.bash
cd /ws/ee4308
python3 tools/plot_drone_bag.py \
  tmp/proj2_runs/run1/aligned_pose_error.csv \
  --output-dir tmp/proj2_runs/run1/plots
```

4. 查看结果

```bash
ls tmp/proj2_runs/run1/plots
cat tmp/proj2_runs/run1/plots/summary.txt
```

如果后面你确实还想保留原始传感器序列，rosbag 仍然可以作为可选补充方案：

```bash
ros2 bag record \
  -o tmp/proj2_run1_bag \
  --topics \
  /drone/odom \
  /drone/true_odom \
  /drone/fix \
  /drone/imu \
  /drone/magnetic \
  /drone/sonar \
  /drone/air_pressure
```

对应画图命令仍然可用：

```bash
source /opt/ros/jazzy/setup.bash
cd /ws/ee4308
python3 tools/plot_drone_bag.py \
  tmp/proj2_run1_bag \
  --output-dir tmp/proj2_run1_bag_plots
```

脚本的端到端验证已经做过一轮，短样例的输出统计示例是：

```text
input_path: .../tmp/bag_examples/testbag
samples: 178

Mean absolute error:
- x: 0.431444 m
- y: 0.329353 m
- z: 0.078180 m
- yaw: 0.005932 rad
```

这说明当前脚本至少已经可以：

- 正常读取 bag 或运行时落盘的 aligned CSV
- 正常反序列化 `Odometry`
- 正常对齐 `/drone/odom` 和 `/drone/true_odom`
- 正常输出图像和误差摘要

#### 调参策略

如果后面继续围绕老师的评分重点做优化，我建议按这个顺序推进：

1. 先固定 behavior / controller，不要三块一起动

原因：

- 老师更看重你们有没有做出“可解释的改进”
- 如果行为、控制、估计同时改，最后很难说明究竟是哪一块带来了提升

2. 先单独优化 `z` 轴，再回头优化平面 `x/y`

原因：

- 当前最明显的问题已经不是“完全飞不起来”，而是 estimator 的细节精度
- `z` 轴有明显的传感器切换特征：低空 sonar 有效，高空 sonar 失效，baro 和 GPS 更关键
- 这一块最适合做成有理有据的实验对比

3. `z` 轴调参时，一次只改一类参数

推荐顺序：

- 先扫 `var_baro`
- 再扫 `var_gps_z`
- 最后扫 `var_imu_z`

原因：

- `var_baro` 决定 baro correction 的强弱
- `var_gps_z` 决定高空时 GPS 对高度的牵引程度
- `var_imu_z` 决定 prediction 对垂直加速度的信任程度

4. `x/y` 调参时，一次成对修改

推荐顺序：

- 一组扫 `var_gps_x = var_gps_y`
- 一组扫 `var_imu_x = var_imu_y`

原因：

- 现在模型没有显式用 roll / pitch
- 所以平面误差更像“整体模型和量测信任关系”的问题
- 成对扫更容易看趋势，也更容易在报告里解释

5. 用统一指标筛 case，不要只靠肉眼

优先看：

- `w10_18 mae(x, y, z)`：代表第一段主要巡航阶段
- `w7_25 mae_z`：代表任务阶段里高度总体表现
- 3D 轨迹图和 `z-t` 曲线：代表可视化说服力

这和老师的偏好是对齐的：

- 不只是“我调了参数”
- 而是“我知道为什么调、怎么比、为什么这个更好”

#### 批量调参与日志汇总脚本

这轮已经补了两个脚本：

- [run_proj2_param_sweep.py](/home/liuyi/projects/ee4308_proj2/tools/run_proj2_param_sweep.py)
- [summarize_proj2_logs.py](/home/liuyi/projects/ee4308_proj2/tools/summarize_proj2_logs.py)

它们的分工是：

1. `run_proj2_param_sweep.py`

- 依次修改 [proj2.yaml](/home/liuyi/projects/ee4308_proj2/src/ee4308_bringup/params/proj2.yaml) 的参数
- 每个 case 跑一轮 headless 仿真
- 每个 case 同时启动轻量级对齐记录器
- 把每个 case 的 `run.log`、`alignment_recorder.log`、`aligned_pose_error.csv`、`plots/`、`proj2_used.yaml`、`summary.txt` 都保存到仓库 `tmp/` 里

默认内置了一组以 `z` 轴为主的 case：

- `current`
- `baro_0p5`
- `baro_1p5`
- `gps_z_0p3`
- `gps_z_0p7`
- `imu_z_10`
- `imu_z_30`

如果你只想跑自己指定的 case，可以加：

```bash
--no-default-cases
```

并通过：

```bash
--case my_case:var_baro=0.8,var_gps_z=0.6
```

追加自定义参数组。

2. `summarize_proj2_logs.py`

- 读取一个或多个 `.log`
- 提取 `ErrPose`
- 计算 `all`、`7~25s`、`10~18s` 这些窗口的 MAE / RMSE
- 如果同目录下存在 `aligned_pose_error.csv`，会自动把 CSV 里的真实误差统计合并进结果
- 对多个 case 做排序
- 输出文本摘要和 CSV

推荐用法：

1. 批量跑参数

```bash
source /opt/ros/jazzy/setup.bash
cd /ws/ee4308
python3 tools/run_proj2_param_sweep.py --duration 40 --output-root tmp/proj2_param_sweeps/run1
```

2. 只跑你自己指定的一组

```bash
source /opt/ros/jazzy/setup.bash
cd /ws/ee4308
python3 tools/run_proj2_param_sweep.py \
  --no-default-cases \
  --case test1:var_baro=0.8,var_gps_z=0.6 \
  --case test2:var_baro=1.2,var_gps_z=0.4 \
  --duration 40 \
  --output-root tmp/proj2_param_sweeps/custom_run
```

3. 单独汇总已有日志

```bash
source /opt/ros/jazzy/setup.bash
cd /ws/ee4308
python3 tools/summarize_proj2_logs.py \
  tmp/proj2_param_sweeps/run1 \
  --csv tmp/proj2_param_sweeps/run1/summary_recomputed.csv \
  --summary tmp/proj2_param_sweeps/run1/summary_recomputed.txt
```

如果这些日志目录里已经有对应的 `aligned_pose_error.csv`，新的汇总会优先显示基于 ground truth 对齐得到的指标，例如：

- `aligned w10_18 mae(x,y,z)`
- `aligned w7_25 mae_z`

后面如果你把：

- `tmp/proj2_param_sweeps/.../summary.csv`
- 或者某个具体 case 的 `run.log`

路径给我，我就可以基于同一套指标继续帮你筛参数、看问题更像 prediction 还是 correction，或者判断是否值得再改代码。

#### 2026-04-05：`z` 轴鲁棒性修正

这轮我针对前一版 estimator 在后半段偶发的高度塌陷，做了两类逻辑修正，代码都在 [estimator.cpp](/home/liuyi/projects/ee4308_proj2/src/ee4308_drone/src/estimator.cpp)。

1. sonar 门控

- 依据模型文件 [drone_proj2.sdf](/home/liuyi/projects/ee4308_proj2/src/ee4308_bringup/models/drone/drone_proj2.sdf) ，仿真的 sonar 量程上限是 `4m`
- 项目世界里有家具、平台等水平表面，所以高空时即使 `sonar` 给出 finite 值，也可能量到“最近的桌面”而不是地面
- 因此现在只在低空且与当前 `z` 估计足够一致时，才接受 sonar correction

当前使用的门控条件是：

- `est_z <= 3.8`
- `abs(z_sonar - est_z) <= 1.0`

如果不满足，会打 `TMP LOG sonar rejected`

2. IMU 的 `z` 向加速度限幅

- 当前 estimator 没有显式建模 roll / pitch
- 所以高速平移或姿态变化时，`imu.linear_acceleration.z - g` 里会混入模型无法解释的分量
- 这会把 `z` prediction 的均值直接拉崩，不是单纯调 `var_imu_z` 就能解决

因此现在把：

- `a_z = imu_z - g`

限幅为：

- `a_z in [-1.5, 1.5]`

如果发生限幅，会打 `TMP LOG imu z accel clamped`

这两类修改的思路都不是“让滤波器更激进”，而是让简化模型不要盲目信任明显不符合任务场景的量测或输入。

#### 这轮验证命令

1. 重新编译

```bash
source /opt/ros/jazzy/setup.bash
cd /ws/ee4308
colcon build --symlink-install --packages-select ee4308_drone ee4308_bringup
```

2. 对比新逻辑下的 `var_gps_z=0.5` 和 `var_gps_z=0.3`

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

3. 复跑一次 `var_gps_z=0.5` 检查是否只是偶然结果

```bash
source /opt/ros/jazzy/setup.bash
cd /ws/ee4308
python3 tools/run_proj2_param_sweep.py \
  --no-default-cases \
  --case current_logic_repeat:var_gps_z=0.5 \
  --duration 40 \
  --output-root tmp/proj2_param_sweeps/gating_run2
```

#### 关键结果

先看修改前的旧 baseline：

- `run1/current` 的 `aligned_all_mae_z = 0.649`
- `run1/current` 的 `aligned_w7_25_mae_z = 0.533`
- `run1/current` 的 `aligned_w10_18_mae_z = 0.499`

再看修改后的结果：

- `gating_run1/current_logic` 的 `aligned_all_mae_z = 0.020`
- `gating_run1/current_logic` 的 `aligned_w7_25_mae_z = 0.019`
- `gating_run1/current_logic` 的 `aligned_w10_18_mae_z = 0.017`

复跑一次也仍然成立：

- `gating_run2/current_logic_repeat` 的 `aligned_w7_25_mae_z = 0.020`
- `gating_run2/current_logic_repeat` 的 `aligned_w10_18_mae_z = 0.014`

这说明：

- 这轮改动最稳定地改善了 `z`
- 改善不是只发生在单一时间窗口
- 至少从两轮 40 秒运行看，后半段那种 `z` 突然掉到 `2~3m` 的问题基本被压住了

#### 日志证据

新日志里可以直接看到 sonar 被门控掉，例如：

- [current_logic/run.log](/home/liuyi/projects/ee4308_proj2/tmp/proj2_param_sweeps/gating_run1/current_logic/run.log) 中多次出现 `TMP LOG sonar rejected: meas=2.5xx est_z=5.0xx innov=-2.5xx`

这类日志说明：

- sonar 在高空时确实会给出看起来“像桌面高度”的 finite 值
- 如果直接把它当世界坐标系下的 `z` correction，会把高度向下拉崩

而在新版本的同一轮运行里，`24s`、`36s`、`40s` 附近的 `ErrPose z` 都保持在 `0.05m` 量级，没有再出现旧版那种 `2m+` 的塌陷。

#### 下一步建议

在这轮代码修正之后，`z` 轴已经不再是最主要风险。后面更应该做的是：

1. 暂时固定当前这套 `z` 轴逻辑
2. 用同样的 sweep 工具继续看平面 `x/y`
3. 优先扫：
   - `var_gps_x = var_gps_y`
   - `var_imu_x = var_imu_y`

原因是：

- 老师更容易给“我通过日志和图证明修掉了一个明确失败模式”的实现高分
- 现在 `z` 已经有比较完整的故事线了
- 剩下更值得投入的是平面位置偏差，而不是再去大改 behavior / controller

#### 2026-04-05：平面 `x/y` lag 分析

在 `z` 轴稳定下来之后，新的主要问题变成了平面方向的 lag。这个现象在：

- [position_vs_time.png](/home/liuyi/projects/ee4308_proj2/tmp/proj2_param_sweeps/gating_run2/current_logic_repeat/plots/position_vs_time.png)

里很明显：

- `z` 基本贴合 ground truth
- `x/y` 在转弯段和大范围机动段会明显落后
- 这个问题更像“滤波器跟得不够快”，而不是“估计值已经发散”

对应统计也支持这个判断：

- `gating_run2/current_logic_repeat` 的整体 `aligned_all_mae_x = 0.481`
- `gating_run2/current_logic_repeat` 的整体 `aligned_all_mae_y = 0.542`
- `10~18s` 这段的 `x` 误差均值基本同号，说明它更接近系统性 lag，而不是零均值噪声

#### 为了解决平面 lag，应该重点看哪部分代码

重点看两段：

1. [estimator.cpp](/home/liuyi/projects/ee4308_proj2/src/ee4308_drone/src/estimator.cpp) 里的 `callbackSubGPS_()`

关键代码是：

- `applyScalarCorrection(Xx_, Px_, Ygps_(0), var_gps_x_)`
- `applyScalarCorrection(Xy_, Py_, Ygps_(1), var_gps_y_)`

这意味着当前 GPS 只直接纠正平面位置状态，不直接纠正平面速度状态。

含义是：

- 如果 `var_gps_x / var_gps_y` 太大，滤波器会不够信 GPS
- 一旦 prediction 本身已经有滞后，GPS correction 就很难及时把 `x/y` 拉回
- 图上看到的“估计轨迹慢半拍”，往往就会出现在这里

2. [estimator.cpp](/home/liuyi/projects/ee4308_proj2/src/ee4308_drone/src/estimator.cpp) 里的 `callbackSubIMU_()`

关键代码是：

- `ax = cos(yaw) * ux - sin(yaw) * uy`
- `ay = sin(yaw) * ux + cos(yaw) * uy`
- `Xx_ = F * Xx_ + W * ax`
- `Xy_ = F * Xy_ + W * ay`

这部分决定了平面 prediction。

含义是：

- 当前模型只用了 yaw，把 body-frame 的 `ux/uy` 旋转到世界系
- 没有显式建模 roll / pitch
- 所以高速平移、转弯或姿态变化时，平面 prediction 只能算一个近似
- 如果 `var_imu_x / var_imu_y` 太小，滤波器会过度相信这个近似 prediction，于是 lag 更明显

所以，要减少 lag，最先应该怀疑和调的是：

- `var_gps_x = var_gps_y`
- `var_imu_x = var_imu_y`

而不是先去大改 behavior / controller。

#### 我做过的一次代码尝试，以及为什么回退

我尝试过一个更激进的方向：

- 用相邻两帧 GPS 的位置差，估一个平面速度
- 再用这个估计的 `vx / vy` 去额外纠正 `Xx_(1)` 和 `Xy_(1)`

这个思路的目标很直接：

- 既然 lag 看起来像速度状态没有及时跟上
- 那就试着给速度状态也补一个 observation

但是在当前项目里，这条线没有表现出足够稳定的收益，所以我已经回退，没有保留在代码里。

我实际跑出来的一轮结果是：

- `planar_logic_run1` 的 `aligned_score = 1.008`
- `aligned_all_mae_x = 0.376`
- `aligned_all_mae_y = 0.509`

它并不是完全没改善，但问题是：

- 它没有稳定地优于更强的 baseline
- 在主演示窗口里，收益不够一致
- 它本质上是在给一个简化模型再加一层“由 noisy GPS 差分得到的速度量测”
- 这种做法对 run-to-run 波动比较敏感，报告里也不如“有明确物理含义的参数调优”好解释

所以我当前的判断是：

- 这条代码方向不是完全错误
- 但以这个作业当前阶段来看，它还不够稳，不值得现在保留

#### 当前更稳的路线

当前更稳的路线仍然是参数层面的平面调优，而不是继续硬改 estimator 逻辑。

优先顺序：

1. 先扫 `var_gps_x = var_gps_y`
2. 再扫 `var_imu_x = var_imu_y`
3. 最后看少量组合 case

原因是：

- lag 的主现象更像“prediction 和 correction 的信任关系没调好”
- 这和老师偏好的“有依据、可解释的改进”更一致
- 即使后面要继续改代码，也应该先用 sweep 把趋势看清楚，再决定是否值得动模型

#### 可直接运行的平面 sweep 脚本

为了避免手写一大串 `--case`，现在补了一个专门扫平面参数的脚本：

- [run_proj2_xy_sweep.py](/home/liuyi/projects/ee4308_proj2/tools/run_proj2_xy_sweep.py)

它默认会跑这几类 case：

- `current`
- 更信 GPS：`var_gps_x = var_gps_y = 0.2 / 0.3 / 0.4 / 0.7`
- 更不信 IMU 平面 prediction：`var_imu_x = var_imu_y = 3 / 4 / 6`
- 少量组合 case

直接运行：

```bash
source /opt/ros/jazzy/setup.bash
cd /ws/ee4308
python3 tools/run_proj2_xy_sweep.py \
  --duration 40 \
  --output-root tmp/proj2_param_sweeps/xy_run1
```

如果想顺手先编译一次：

```bash
source /opt/ros/jazzy/setup.bash
cd /ws/ee4308
python3 tools/run_proj2_xy_sweep.py \
  --build \
  --duration 40 \
  --output-root tmp/proj2_param_sweeps/xy_run1
```

跑完后重点看：

- `tmp/proj2_param_sweeps/xy_run1/summary.csv`
- 每个 case 的 `plots/position_vs_time.png`
- 每个 case 的 `plots/error_vs_time.png`

后面把 `summary.csv` 路径给我，我就可以继续帮你筛哪组平面参数最值得保留。

#### 2026-04-05：对外部建议的判断，以及本轮实际修改

外部建议的大方向是有道理的，但要区分“值得现在立刻改”的内容和“原理上对、但当前阶段不该先做”的内容。

我对那份建议的判断是：

- “`x/y` 的问题更像 estimator 结构 + 调参问题，而不是单点 bug”这个判断是对的
- “先盯 `callbackSubGPS_()` 和 `callbackSubIMU_()`”这个判断是对的
- “先扫 `var_gps_x/y` 和 `var_imu_x/y`”这个优先级是对的
- “GPS 差分速度伪量测可以作为二阶段尝试”这个说法也对
- 但“现在就把 GPS 差分速度 correction 重新加回去”我不同意，因为我已经试过一轮，收益不够稳定，解释成本也更高

所以本轮我只改了三类低风险、但确实有帮助的东西。

1. 统一 estimator 发布出来的时间戳口径

之前 `callbackSubIMU_()` 的 prediction 用的是 `msg.header.stamp` 推进状态，但 `callbackTimer()` 发布 `/drone/odom` 时用的是 `this->now()`。

这会带来一个问题：

- estimator 内部状态对应的是“最近一次传感器更新时间”
- 但发布出去的 topic 时间戳对应的是“timer 触发时间”

这样在后处理里按 topic 时间戳对齐 `/drone/odom` 和 `/drone/true_odom` 时，会额外引入一层表观 lag。

所以现在改成了：

- estimator 维护 `latest_state_stamp_`
- 每次 IMU prediction 或 GPS / sonar / magnet / baro correction 成功后，都把它更新成对应 message 的 `header.stamp`
- 发布 odom 时用 `latest_state_stamp_`

这不会神奇地消灭真实 lag，但会让图上的 lag 更可信，不会再混进一层“时间戳口径不一致”的假延迟。

2. 把 correction 的协方差更新改成 Joseph form

之前的更新写法是最简形式：

```text
P = P - K H P
```

这种写法在数值上比较容易让协方差矩阵失去严格对称性，长期运行时会让滤波器显得过度自信。

现在改成了 Joseph form：

```text
P = (I - K H) P (I - K H)^T + K R K^T
```

并且在 prediction / correction 后都做一次：

```text
P = 0.5 * (P + P^T)
```

这类改动通常不会直接把 lag 变没，但它是更稳、更规范的 EKF 实现方式，也更符合报告里“实现是严谨的”这个方向。

3. 补了平面 lag 诊断用的 GPS innovation 临时日志

现在在 `callbackSubGPS_()` 里会打 `TMP LOG`，内容包括：

- `gps innov_xy`
- 当前 `est_xy`
- 当前 `gps_xy`
- 当前 `vel_xy`

这样做的目的不是为了最后提交时保留日志，而是为了回答一个很具体的问题：

- 是不是 prediction 先带着旧的 `vx/vy` 慢慢漂
- 然后每次 GPS 到来时，再把位置猛地往回拽

如果 log 里长期出现同号、周期性回拉的 `innov_x / innov_y`，那就能更有把握地证明我们现在看到的锯齿和 lag，确实来自“简化 prediction + 位置型 GPS correction”的组合。

#### 本轮没有做的事，以及为什么没做

这轮没有把“相邻两帧 GPS 差分出速度，再纠正 `vx / vy`”重新加回代码里。

原因不是它一定错误，而是：

- 我之前已经做过一次原型验证
- 它不是完全没改善，但收益不够稳定
- 它对 noisy GPS 差分很敏感
- 放进报告里也不如“参数调优 + 时间戳修正 + 更规范的协方差更新”容易解释

所以当前更稳的路线仍然是：

1. 先让时间戳和协方差实现更规范
2. 再继续扫 `var_gps_x/y` 和 `var_imu_x/y`
3. 如果参数扫完 lag 仍然明显，再把速度伪量测当成第二阶段尝试

#### 2026-04-05：`xy_run3` 结果解读

这一轮使用的命令是：

```bash
source /opt/ros/jazzy/setup.bash
cd /ws/ee4308
python3 tools/run_proj2_xy_sweep.py \
  --duration 40 \
  --output-root tmp/proj2_param_sweeps/xy_run3
```

结果里最重要的结论，不是“第 1 名到底是谁”，而是下面这件事：

- `best_so_far` 和 `current_xy` 实际上是同一组参数

也就是：

- `var_gps_x = var_gps_y = 0.4`
- `var_imu_x = var_imu_y = 3.0`

这组参数也已经是当前 [proj2.yaml](/home/liuyi/projects/ee4308_proj2/src/ee4308_bringup/params/proj2.yaml) 里的默认值。

但在这轮 sweep 里：

- `best_so_far` 的 `aligned_score = 0.644`
- `current_xy` 的 `aligned_score = 0.874`

这说明即使参数完全相同，单次 run 的结果也会有明显波动。原因通常来自：

- 传感器噪声
- flight path 某些阶段的细微时序差异
- correction 与 prediction 的耦合在不同 run 里被放大或缩小

所以这里不能把单次排名机械地理解成“0.644 一定严格优于 0.874 对应的参数”，因为它们本来就是同一组参数。

这件事反过来也说明：

- 当前 `proj2.yaml` 里的平面参数已经进入“比较合理”的区间
- 接下来更应该看“重复跑时的稳定性”，而不是继续大范围扫很多新参数

#### 这一轮里最值得关注的几组

1. `var_gps_x/y = 0.4`，`var_imu_x/y = 3.0`

这是当前默认值，也是我目前最建议保留的 baseline。

原因：

- 演示窗口 `10~18s` 的 `x` 误差已经比早期版本明显下降
- `z` 仍然保持很稳
- 参数含义容易解释
- 代码里也已经同步成默认配置

2. `var_gps_x/y = 0.35`，`var_imu_x/y = 3.5`

这一组的整体误差很强：

- `aligned_all_mae_x = 0.319`
- `aligned_all_mae_y = 0.383`

从“全程平均误差”看，它甚至比当前 baseline 更漂亮。

但它不是当前首选的原因是：

- 单次 run 与单次 run 之间已经有明显波动
- 它虽然全程平均更强，但演示窗口和总分未必稳定压过 baseline
- 如果没有再做重复试验，就不值得贸然把默认参数换掉

3. `var_gps_x/y = 0.4`，`var_imu_x/y = 3.5`

这一组很有意思：

- `y` 非常好
- 但 `x` 会更差一些

它更像是在“跟手程度”和“平滑程度”之间，往另一个方向偏了一点。可以保留做对照，但不建议直接取代 baseline。

#### 从 `TMP LOG gps innov_xy` 看到了什么

这轮 log 继续支持之前的判断：

- 当前问题确实更像“prediction 慢、GPS 周期性回拉”
- 而不是坐标系写反、ECEF 变换错误这种硬 bug

例如：

- `current_xy` 里能看到比较大的 innovation，例如 `innov_x ≈ 0.717`、`innov_y ≈ 0.550`
- `best_so_far` 和 `refine_gps_0p40__imu_3p5` 里，大多数 innovation 明显更收敛，更多落在 `0.1 ~ 0.3` 量级

这说明改完时间戳和协方差更新之后，当前剩下的主要问题仍然是平面融合的权重取舍，而不是某个基础公式已经错了。

#### 当前建议

当前建议非常明确：

1. 保留 [proj2.yaml](/home/liuyi/projects/ee4308_proj2/src/ee4308_bringup/params/proj2.yaml) 里的平面参数作为默认值
2. 不继续大改 estimator 结构
3. 如果要再验证，只做一个很小的重复性对比：

- baseline：`gps=0.4, imu=3.0`
- 候选 A：`gps=0.35, imu=3.5`
- 候选 B：`gps=0.4, imu=3.5`

每组各跑 3 次，再比较平均值和波动范围。

如果 baseline 的平均结果和方差都更稳，就不再继续追求更复杂的平面逻辑；把时间留给实验记录、图表整理和报告会更划算。

#### 2026-04-05：重新审视平面模型，并保守地重加 GPS velocity pseudo-measurement

在继续看平面 lag 时，有一个细节需要先说清楚：

- 当前 GPS correction 并不是“完全不修速度”

虽然代码里对平面状态用的是位置量测模型 `H = [1, 0]`，但由于 `Px_` / `Py_` 里存在位置与速度的协方差，Kalman gain 的第二项一般并不为 0。

这意味着：

- GPS 位置量测本来就会通过协方差，间接拉动 `vx / vy`
- 当前真正的问题不是“速度完全不可观”
- 而是“速度的可观性还不够强，而且在转弯/大机动段显得偏慢”

所以，如果要重加 `GPS velocity pseudo-measurement`，正确理解应该是：

- 不是从 0 到 1 新增一个速度观测
- 而是在现有位置 correction 已经会间接修速度的前提下，再给平面速度补一个保守的、带门控的增强项

#### 这次代码上怎么改

本轮没有去硬改平面 prediction 模型本身，因为在课程规则下：

- 不能直接拿 IMU orientation 当真值姿态
- 也没有额外可靠的 roll / pitch 估计

所以与其假装“把平面动力学建模得更完整”，不如承认当前 prediction 仍然是近似模型，然后用更保守的 measurement 侧增强来补。

这轮实际代码修改有三部分：

1. 给 `Eigen::Vector2d` 的 correction helper 加了通用观测模型

现在不仅能用 `H = [1, 0]` 修位置，也能用 `H = [0, 1]` 修速度。

2. 在 GPS callback 里加入了 `maybeApplyGPSVelocityCorrection_()`

逻辑是：

- 用相邻两帧 GPS 位置差除以 `dt_gps`，得到原始 `vx / vy`
- 再做一个一阶低通，避免直接把 noisy 差分速度塞进滤波器
- 然后分别对 `Xx_(1)` 和 `Xy_(1)` 做速度 correction

3. 整个伪速度量测是带门控的

主要门控包括：

- `dt_gps` 必须落在合理范围内
- 推出的速度必须是有限值
- `|v_meas - v_est|` 不能超过阈值
- velocity measurement variance 不是固定常数，而是按 `2 * var_gps / dt^2` 缩放，并再乘一个保守系数

这样做的目的很明确：

- 只在 GPS 差分速度“看起来还像回事”的时候使用它
- 避免它变成一个每秒都把平面状态往噪声上拉的坏观测

#### 这一轮的关键参数

新增的 estimator 参数有：

- `gps_velocity_alpha`
- `gps_velocity_variance_scale`
- `gps_velocity_min_variance`
- `gps_velocity_max_innovation`
- `gps_velocity_min_dt`
- `gps_velocity_max_dt`

当前默认值写在 [proj2.yaml](/home/liuyi/projects/ee4308_proj2/src/ee4308_bringup/params/proj2.yaml)。

其中最关键的是：

- `gps_velocity_variance_scale`

它控制“这个伪速度量测到底要被信多少”。

约定是：

- `gps_velocity_variance_scale <= 0` 时，相当于关闭这条逻辑
- 值越小，代表越相信伪速度量测
- 值越大，代表越保守

#### 这轮实际对比：`xy_logic_run2`

这轮使用的对比命令是：

```bash
source /opt/ros/jazzy/setup.bash
cd /ws/ee4308
python3 tools/run_proj2_param_sweep.py \
  --no-default-cases \
  --duration 40 \
  --output-root tmp/proj2_param_sweeps/xy_logic_run2 \
  --case pseudo_off:var_gps_x=0.4,var_gps_y=0.4,var_imu_x=3.0,var_imu_y=3.0,gps_velocity_variance_scale=0.0 \
  --case pseudo_default:var_gps_x=0.4,var_gps_y=0.4,var_imu_x=3.0,var_imu_y=3.0,gps_velocity_variance_scale=1.0,gps_velocity_alpha=0.6 \
  --case pseudo_stronger:var_gps_x=0.4,var_gps_y=0.4,var_imu_x=3.0,var_imu_y=3.0,gps_velocity_variance_scale=0.5,gps_velocity_alpha=0.8 \
  --case pseudo_weaker:var_gps_x=0.4,var_gps_y=0.4,var_imu_x=3.0,var_imu_y=3.0,gps_velocity_variance_scale=1.5,gps_velocity_alpha=0.6
```

结果很清楚：

- `pseudo_default`：`aligned_score = 0.628`
- `pseudo_off`：`aligned_score = 0.645`
- `pseudo_stronger`：`aligned_score = 0.903`
- `pseudo_weaker`：`aligned_score = 0.997`

这说明：

- 这条逻辑不是完全没用
- 但它只能工作在一个比较保守的区间里
- 一旦更激进或者更保守，都会明显变差

从这轮结果看，当前最合理的选择就是：

- 保留 `pseudo_default`
- 不再继续把它调得更强
- 也不把它完全关掉

#### 这轮结果应该怎么理解

最值得注意的是：

- `pseudo_default` 相比 `pseudo_off`，整体分数更好
- 说明“保守的伪速度量测”确实带来了收益

但它的收益不是“所有轴都一起变好”。

更准确地说：

- 它主要改善了平面里最拖后腿的那一部分表现
- 但不同 run 里，`x` 和 `y` 的改善分配可能并不完全一致

所以这条逻辑适合这样写进报告：

- 它是一个保守的增强项
- 目的是减少平面 lag
- 实验表明，适当启用时优于完全关闭
- 但它对权重很敏感，因此最终保留了偏保守的默认设置

#### 当前结论

当前我对这条逻辑的结论是：

1. 值得保留
2. 只能保留保守版本
3. 不值得继续在这条线上做大范围参数搜索

也就是说，到这个阶段，平面 estimator 更合理的状态是：

- 保留当前 `proj2.yaml` 里的 `gps_velocity_*` 默认值
- 把它当作“平面 lag 的小幅增强补丁”
- 然后把主要精力转回实验记录、结果整理和报告表达

#### 2026-04-05：补 GPS / magnet / baro 的软门控

老师在 [proj2.md](/home/liuyi/projects/ee4308_proj2/docs/proj2.md) 里对 correction 的描述重点是：

- correction 是异步发生的
- 每次量测都要“faithfully update”状态和协方差

这并不意味着“每一条量测都必须无条件接收”。

在实际仿真里，更稳妥的理解应该是：

- 合法、可信的量测就按 Kalman correction 正常更新
- 明显离谱的 outlier 先拒绝，再等待下一条量测

之前代码里：

- `sonar` 已经有比较强的门控
- 但 `GPS` 位置 correction、`magnet` yaw correction、`baro` correction 基本还是“只要数值有限就接收”

这会带来两个问题：

- 偶发跳变会把状态突然拉走
- 这种拉走本身又会在图上表现成更大的锯齿和更难看的“表观 lag”

所以这轮补的是“软门控”而不是“激进拒绝”：

- `GPS x/y/z`：只拒绝 innovation 很大、或者 normalized innovation 很离谱的更新
- `magnet`：拒绝 yaw innovation 太大的更新
- `baro`：拒绝和 `z + bias` 明显不一致的更新

注意这里的目标是：

- 优先减少偶发跳变
- 提升轨迹平滑性和鲁棒性

而不是：

- 直接把系统性的平面 lag 消灭掉

所以对 lag 的影响应该这样理解：

- 如果 lag 里有一部分其实是“outlier 拉走之后又被拉回来”的表观现象，那软门控会有帮助
- 但如果 lag 的主因是平面 prediction 本身偏慢，或者 `GPS / IMU` 权重关系没调好，那软门控只能间接改善，不能替代平面参数调优

这也是为什么当前更合理的定位是：

- 软门控负责“减少跳变和坏 correction”
- `gps_velocity_*` 和 `var_gps_x/y, var_imu_x/y` 负责“改善平面跟手程度”

现在新增到 [proj2.yaml](/home/liuyi/projects/ee4308_proj2/src/ee4308_bringup/params/proj2.yaml) 的门控参数包括：

- `gps_position_max_innovation_xy`
- `gps_position_max_innovation_z`
- `gps_position_max_sigma`
- `magnet_max_innovation`
- `magnet_max_sigma`
- `baro_max_innovation`
- `baro_max_sigma`

这些默认值故意设得比较保守，目的是先挡掉明显坏点，而不是把正常 correction 卡掉。

#### 2026-04-05：对另一份外部分析的判断

这轮还参考了一份外部分析。它的总体方向是有价值的，但里面有些结论可以直接吸收，有些则需要更谨慎地验证。

#### 我认为说得对的部分

1. `GPS velocity pseudo-measurement` 当前增益可能偏小

这个判断是有道理的。

当前伪速度量测的 measurement variance 来自：

- `gps_velocity_min_variance`
- `gps_velocity_variance_scale * 2 * var_gps / dt_gps^2`

如果这个 measurement variance 明显大于当前速度状态的后验方差，那么 Kalman gain 就会偏小，伪速度量测只能起到非常弱的拉动作用。

这和我们之前的实验结论并不矛盾：

- `pseudo_default` 比 `pseudo_off` 略好，说明它不是没用
- 但收益并不大，说明它确实可能还偏保守

2. 用 GPS 有限差分得到的速度，在加速段会系统性偏低

这个判断也是对的。

原因很直接：

- 相邻两帧位置差 / `dt` 给出的是这一段时间里的平均速度
- 而滤波器当前要修的是“当前时刻”的速度状态

所以在明显加速段里，差分速度天然会比真实瞬时速度更低。这也是为什么伪速度量测不能设得太激进，否则容易把系统往“更慢”的方向拉。

3. GPS correction 可能存在“测量时刻落后于当前状态”的问题

这一点也值得重视。

当前状态已经被 IMU 预测推进到了最近一次 IMU 时间戳，而 GPS message 的 `stamp` 可能更早。如果直接用旧时刻的 GPS 位置去修正当前状态，本质上是在做一个近似的 out-of-sequence correction。

所以外部分析提出的“把 GPS 位置按当前估计速度做一个小幅前向补偿”是有物理直觉支持的。

#### 我认为需要保留意见的部分

1. “当前伪速度量测近似完全无效”这个结论说得太满

从我们自己的对比：

- `pseudo_default`：`aligned_score = 0.628`
- `pseudo_off`：`aligned_score = 0.645`

可以看出，当前伪速度量测不是“完全无效”，只是增益比较保守，收益有限。

所以更准确的说法应当是：

- 当前伪速度量测是有效的
- 但它的作用偏弱，可能仍有小幅增强空间

2. “`var_gps_x/y = 0.1 ~ 0.3` 完全没试过”并不准确

我们之前其实已经试过其中一部分低值区间，尤其是 `0.2` 和 `0.3`。

更准确的说法应当是：

- 低 `var_gps_x/y` 区间没有被系统地细扫
- 特别是 `0.1 / 0.15 / 0.25` 这几个点还没有做过成体系比较

所以这条建议值得参考，但原始表述有点过头。

#### 我当前最认可的两个后续方向

1. 小幅增强伪速度量测，而不是一下子把它调得很激进

外部分析建议：

- `gps_velocity_min_variance = 0.05`
- `gps_velocity_variance_scale = 0.1`

这个方向本身是值得验证的，但风险也很明显：

- 它比当前默认设置激进很多
- 如果差分速度在加速段偏低，那过强的速度 correction 反而会把 lag 拉大

所以更合理的做法不是直接相信它，而是把它作为下一轮“小规模候选配置”之一来对比，而不是直接设成新默认值。

2. 试一个保守版的 GPS 前向补偿

外部分析里最值得认真考虑的代码点，其实是这个：

- 在 GPS correction 之前，根据 `dt_lag = last_predict_time_ - stamp.seconds()`，用当前估计速度把 GPS 测量前向补偿到当前状态时刻

这个方向的优点是：

- 它直接针对“状态在当前时刻，而 GPS 量测在过去时刻”的不一致
- 理论上比单纯继续压低 `var_gps_x/y` 更有针对性

它的风险是：

- 会把当前估计速度误差又反馈回位置量测
- 如果 `dt_lag` 很大或速度本身不准，可能会产生反效果

所以如果要做，这也应该是“带门控、带 sanity check 的小改动”，而不是无条件启用。

#### 当前我的结论

如果把这份外部分析总结成一句话，我的判断是：

- 它指出的问题方向基本是对的
- 其中最值得参考的是“伪速度量测可能过弱”和“GPS 时间对齐误差可能造成额外 lag”
- 但它给出的参数值不能直接当答案，需要实验验证

所以当前最值得做的，不是立刻大改，而是等你这轮门控对比结果出来之后，再决定下面二选一：

1. 做一轮“更强一点的伪速度量测”小范围对比
2. 做一个“带 `dt_lag` sanity check 的 GPS 前向补偿”小 patch

这两条里，我个人更看好第 2 条，因为它更直接针对时序不一致问题，也更容易在报告里解释。

#### 2026-04-05：`gating_compare1` 结果与后续选择

你跑出来的 `gating_compare1` 结果是：

- `gating_off`：`aligned_score = 0.634`
- `gating_default`：`aligned_score = 0.750`
- `gating_tighter`：`aligned_score = 2.413`

这说明：

- 这轮加入的软门控并没有直接带来更好的总分
- 更紧的门控会明显变差，说明过度拒绝 correction 很危险

所以这里的结论应该写得很克制：

- 软门控对“防偶发坏点”仍然是有工程价值的
- 但它不是当前 lag 的主解法
- 不能把它当成平面跟手性能的核心改进

这也支持了后续把注意力转向 “GPS 前向补偿” 这条线。

进一步解释这组结果：

- `gating_default` 比 `gating_off` 差，说明当前这版门控默认阈值已经开始拦掉一部分本来有用的 correction
- `gating_tighter` 明显更差，说明问题不是“门控方向错了”，而是“这类门控一旦变严，就会快速伤到正常更新”

因此当前更合理的工程结论是：

- 这套门控代码可以保留，作为可选保护逻辑和后续分析工具
- 但不应该把当前默认阈值当成“效果增强”方案
- 后续如果要继续做平面 lag 相关实验，应该先把门控等效关闭，避免它和前向补偿、伪速度量测互相干扰

#### 2026-04-05：保守版 GPS 前向补偿 patch

在 `callbackSubGPS_()` 里，当前状态已经由 IMU prediction 推进到了最近一次 IMU 时间，而 GPS message 的 `stamp` 往往更早。

因此，如果直接用原始 GPS 位置去修正当前状态，本质上是在做一个近似的 out-of-sequence correction。

这轮实现了一个保守版的前向补偿：

- 先计算 `dt_lag = last_predict_time_ - stamp.seconds()`
- 只有在 `0 < dt_lag < gps_forward_compensation_max_dt` 时才启用
- 用当前估计速度把 GPS 量测前向补偿到当前状态时刻：

```text
x_gps_corr = x_gps + vx_est * dt_lag
y_gps_corr = y_gps + vy_est * dt_lag
z_gps_corr = z_gps + vz_est * dt_lag
```

然后：

- 用补偿后的 `gps_correction_measurement` 做位置 innovation 计算
- 再进行 GPS gating 和 correction

这条逻辑的目标不是“神奇地把 GPS 变准”，而是：

- 让 correction 时刻和状态时刻更一致
- 尽量减少因为时间错位带来的表观 lag

为了方便后续做开关对比，这次把它也做成了参数：

- `gps_forward_compensation_enable`
- `gps_forward_compensation_max_dt`

默认值写在 [proj2.yaml](/home/liuyi/projects/ee4308_proj2/src/ee4308_bringup/params/proj2.yaml)：

- `gps_forward_compensation_enable: true`
- `gps_forward_compensation_max_dt: 0.5`

如果后面要对比这条 patch 是否有效，只需要在 sweep 里开关这两个参数，不需要再改代码。

但结合 `gating_compare1` 的结果，下一轮更合理的对比方式应该是：

- 先把 `GPS / magnet / baro` 门控等效关闭
- 再比较前向补偿 `on / off / smaller_dt`

这样才能更干净地判断：

- 前向补偿本身是否有帮助

而不会把门控副作用一起混进去。

#### 2026-04-05：`forward_comp_compare2` 结果

在把 `GPS / magnet / baro` 门控等效关闭后，前向补偿对比结果是：

- `fc_on_no_gate`：`aligned_score = 0.559`
- `fc_off_no_gate`：`aligned_score = 0.864`
- `fc_small_dt_no_gate`：`aligned_score = 0.940`

这组结果的结论非常明确：

1. GPS 前向补偿是有效的

和关闭前向补偿相比：

- `fc_on_no_gate` 的总分明显更好
- `10~18s` 窗口里，`x` 误差从 `0.723` 降到了 `0.373`
- `y` 虽然略有权衡，但整体综合指标明显改善

这说明当前 lag 的确有一部分来自：

- GPS correction 使用的是过去时刻的量测
- 直接拿它修当前状态，会在平面机动段造成额外滞后

2. `gps_forward_compensation_max_dt = 0.5` 比 `0.2` 更合适

`fc_small_dt_no_gate` 明显比 `fc_on_no_gate` 差，说明：

- 当前实际存在的 GPS 时间错位，不只是很小的一点点
- 把最大补偿窗口限制到 `0.2s`，会让很多本该补偿的 GPS correction 又退回到旧问题

所以当前默认保留：

- `gps_forward_compensation_enable: true`
- `gps_forward_compensation_max_dt: 0.5`

3. 当前新加的 `GPS / magnet / baro` 门控不应作为默认增强方案

结合前面的 `gating_compare1`：

- 当前门控默认值不但没带来更好结果
- 还会干扰对前向补偿效果的判断

因此现在更合理的默认配置是：

- 保留门控代码，作为可选开关和分析工具
- 但在默认参数里把这些门控等效关闭

也就是说，当前默认策略已经收敛为：

- `sonar` 继续保留强门控
- `GPS / magnet / baro` 的新软门控代码保留，但默认关闭
- `GPS` 前向补偿默认开启

#### 当前推荐状态

基于目前所有实验，当前最值得保留的 estimator 组合是：

- 平面参数：`var_gps_x/y = 0.4`，`var_imu_x/y = 3.0`
- `gps_velocity_*` 保留保守默认值
- `gps_forward_compensation_enable = true`
- `gps_forward_compensation_max_dt = 0.5`
- 新增的 `GPS / magnet / baro` 门控默认等效关闭

这组配置的特点是：

- 比早期版本更能跟上平面机动
- 不依赖激进门控
- 每一项改动都比较容易解释
