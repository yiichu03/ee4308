# EE4308 Lab 2 调参与结果查看指南

## 目的

这份文档只关注两件事：

- 如何调 `lab2` 需要的参数
- 如何判断你的 `callbackSubIMU_()` 和 `callbackSubSonar_()` 是否基本正确

## 先明确要看的对象

Lab 2 的核心状态只有竖直方向：

- 位置：`z`
- 速度：`z_dot`

对应代码里的变量：

- `Xz_(0)`：估计的高度 `z`
- `Xz_(1)`：估计的竖直速度 `z_dot`
- `Pz_`：这两个状态的协方差
- `Ysonar_`：声呐测到的高度

## 主要调哪些参数

参数文件在：

`src/ee4308_bringup/params/proj2.yaml`

Lab 2 最重要的是这两个：

- `var_imu_z`
- `var_sonar`

含义可以简单理解为：

- `var_imu_z` 越大：越不信任 IMU 的竖直加速度
- `var_sonar` 越大：越不信任 sonar 的高度观测

## 基本运行流程

在仓库根目录：

```bash
cd /home/liuyi/projects/ee4308_course
colcon build --symlink-install
source install/setup.bash
ros2 launch ee4308_bringup proj2_sim.launch.py
```

如果只是改了 `proj2.yaml`，通常可以直接重新启动，不必重新编译。

## 终端里主要看什么

估计器会持续打印多行状态。对 Lab 2，重点看这几项：

- `Pose`
- `Twist`
- `ErrPose`
- `ErrTwis`
- `Sonar`

### `Pose`

这里重点看第 3 个数，也就是 `z`。

你希望它：

- 不要明显发散
- 不要持续单方向漂移
- 大致接近 `Sonar` 里显示的高度

### `Twist`

这里重点看第 3 个数，也就是 `z` 速度。

如果无人机没有明显上下运动，它应该接近 `0`。  
如果一直偏大、偏小，或者持续累积，通常是 IMU 预测有问题。

### `ErrPose`

这里重点看第 3 个数，即：

```text
true_z - estimated_z
```

你希望它：

- 数值不大
- 不持续变大
- 能围绕某个较小范围波动

### `ErrTwis`

这里重点看第 3 个数，即：

```text
true_zdot - estimated_zdot
```

如果这个值长期不接近 0，说明速度估计有问题，通常先检查：

- 重力补偿符号
- `var_imu_z`

### `Sonar`

这是声呐观测值。  
如果 `Pose` 的高度与 `Sonar` 长期差很多，说明校正太弱或者预测出了偏差。

## 如何判断重力补偿符号对不对

`callbackSubIMU_()` 里最关键的一步是：

```text
a_z = u_z ± g
```

在这个项目里，正确实现应当让无人机静止时：

- `z_dot` 接近 0
- `z` 不会持续快速上漂或下漂

如果你把符号写反，常见现象是：

- `Twist` 的 `z` 速度明显不对
- `Pose` 的 `z` 高度持续漂
- `ErrPose` 和 `ErrTwis` 越来越大

所以，最先检查的不是“公式好不好看”，而是运行后静止状态下 `z` 和 `z_dot` 是否稳定。

## `var_sonar` 怎么调

### 直观理解

- `var_sonar` 小：滤波器更相信 sonar
- `var_sonar` 大：滤波器更不相信 sonar

### 现象与调整方向

如果出现以下现象：

- 高度估计很抖，几乎跟着 sonar 原始噪声上下跳  
  说明 `var_sonar` 可能太小，应适当增大

- 高度估计过于迟钝，明明 sonar 已经稳定，但估计值还长期偏着  
  说明 `var_sonar` 可能太大，应适当减小

### 一个实用调法

1. 先给 `var_sonar` 一个中间值
2. 启动仿真观察 `Pose.z` 与 `Sonar`
3. 如果 `Pose.z` 太抖，就增大一点
4. 如果 `Pose.z` 修正太慢，就减小一点
5. 每次只改一点，重新启动后比较效果

## `var_imu_z` 怎么调

### 直观理解

- `var_imu_z` 小：更相信 IMU 预测
- `var_imu_z` 大：更少相信 IMU，更多依赖 sonar 校正

### 现象与调整方向

如果出现以下现象：

- sonar 一旦短时间不可用，估计值很快漂走  
  可能是 IMU 预测本身偏差大，或者 `var_imu_z` 太小

- 明明 IMU 预测没那么差，但估计值几乎完全被 sonar 拉着走  
  可能是 `var_imu_z` 太大

### 一个实用调法

1. 先让重力补偿符号正确
2. 先把 `var_sonar` 调到一个合理范围
3. 再调 `var_imu_z`
4. 观察没有明显上下运动时，`z_dot` 是否接近 0，`z` 是否稳定

## 如何估计 `var_sonar`

Canvas 第 3 题其实就在问这个过程。  
一个够用的实验流程是：

1. 让无人机尽量静止
2. 连续记录约 100 个 sonar 高度样本
3. 因为平台可能慢慢上漂，所以不要直接对原始高度算方差
4. 先对“高度-时间”做一条 best fit line
5. 用每个样本减去拟合线，得到 residual
6. 用 residual 的 sample variance 作为 `var_sonar` 初值
7. 再回到仿真里做少量微调

## 推荐的查看命令

查看 topic：

```bash
ros2 topic list
```

查看 estimator 输出：

```bash
ros2 topic echo /drone/odom --once
```

查看 sonar 数据：

```bash
ros2 topic echo /drone/sonar --once
```

查看 IMU 数据：

```bash
ros2 topic echo /drone/imu --once
```

查看频率：

```bash
ros2 topic hz /drone/imu
ros2 topic hz /drone/sonar
ros2 topic hz /drone/odom
```

查看参数：

```bash
ros2 param get /drone/estimator var_imu_z
ros2 param get /drone/estimator var_sonar
```

## 一份简短的自检标准

如果你的实现基本正确，通常会看到：

- 仿真启动后 drone 不会因为 estimator 自己“飞掉”
- `Pose.z` 与 `Sonar` 数值接近
- `Twist.z` 在静止时接近 0
- `ErrPose.z` 和 `ErrTwis.z` 不会持续发散
- 调整 `var_sonar` 后，估计结果会出现可解释的变化

如果你看到的是：

- 高度快速发散
- 速度一直累积
- 调参数几乎没有效果

那就优先回头检查：

1. `callbackSubIMU_()` 里重力补偿符号
2. `F`、`W` 的公式
3. `callbackSubSonar_()` 里 innovation 和 Kalman gain 的写法
4. 参数文件是否真的被加载到了 `/drone/estimator`
