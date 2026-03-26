# EE4308 Lab 2 调参与结果查看指南

## 目的

这份文档只关注两件事：

- 如何调 `lab2` 需要的参数
- 如何判断你的 `callbackSubIMU_()` 和 `callbackSubSonar_()` 是否基本正确

## 如果你使用 `ee4308_jazzy_proj2` 容器

你已经确认课程 Docker 是：

- 容器名：`ee4308_jazzy_proj2`
- 容器内工作区：`/ws/ee4308`
- Host 绑定目录：`/home/liuyi/projects/ee4308_proj2`

常用命令：

```bash
xhost +SI:localuser:root
docker start ee4308_jazzy_proj2
docker exec -it ee4308_jazzy_proj2 bash
cd /ws/ee4308
source /opt/ros/jazzy/setup.bash
```

如果容器里第一次运行 `proj2_sim.launch.py` 时提示找不到 `model://turtlebot3_house`，先安装：

```bash
apt-get update
apt-get install -y ros-jazzy-turtlebot3-gazebo
```

`lab1/proj1` 现在应使用另一只容器 `ee4308_jazzy_proj1`，不要和 `proj2` 混用。

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

如果你在 `ee4308_jazzy_proj2` 容器里操作，这个文件的容器内路径是：

`/ws/ee4308/src/ee4308_bringup/params/proj2.yaml`

Lab 2 最重要的是这两个：

- `var_imu_z`
- `var_sonar`

含义可以简单理解为：

- `var_imu_z` 越大：越不信任 IMU 的竖直加速度
- `var_sonar` 越大：越不信任 sonar 的高度观测

当前仓库默认值是：

- `var_imu_z: 1.0`
- `var_sonar: 1.0`

建议把这两个值当成 baseline，不要一开始就乱跳太大。

## 基本运行流程

如果你是在 Docker 容器里操作，优先使用容器内路径：

```bash
cd /ws/ee4308
colcon build --symlink-install
source install/setup.bash
ros2 launch ee4308_bringup proj2_sim.launch.py
```

如果只是改了 `proj2.yaml`，通常可以直接重新启动，不必重新编译。

如果你是在 `ee4308_jazzy_proj2` 容器里操作，则使用：

```bash
docker start ee4308_jazzy_proj2
docker exec -it ee4308_jazzy_proj2 bash
cd /ws/ee4308
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
ros2 launch ee4308_bringup proj2_sim.launch.py
```

如果你想把终端输出直接写进日志文件，也可以在容器里用：

```bash
cd /ws/ee4308
./bd.sh
./proj2_sim_log.sh
```

其中：

- `./bd.sh` 只是编译脚本，不会启动仿真
- `./proj2_sim_log.sh` 会启动仿真，并把输出写到 `log/lab2/`

不要在同一个终端里“同时”运行两个前台脚本。  
正确做法是：

1. 先运行 `./bd.sh`
2. 等它结束
3. 再运行 `./proj2_sim_log.sh`

如果你想批量扫描参数，而不是手动一次次改 `proj2.yaml`，可以用：

```bash
cd /ws/ee4308
./sweep_lab2_params.sh 20
./analyze_lab2_logs.sh log/lab2/sweep_<timestamp>
```

说明：

- `./sweep_lab2_params.sh 20`
  - 表示每组参数跑 20 秒
  - 当前默认会扫描一组粗网格的 `var_imu_z` 和 `var_sonar`
  - 每组运行日志都会写到 `log/lab2/sweep_<timestamp>/`

- `./analyze_lab2_logs.sh <日志目录>`
  - 会从日志里提取 `z` 轴相关指标
  - 把结果写到 `tmp/lab2_analysis/`
  - 主要看 `score` 越小越好

## 调参前先固定这几件事

调参时，最怕的是一次同时改太多东西。先固定下面几件事：

1. 不要一边调参数一边改 `callbackSubIMU_()` 和 `callbackSubSonar_()` 代码。
2. 每次都从同一个 launch 命令启动：

```bash
ros2 launch ee4308_bringup proj2_sim.launch.py
```

3. 每次至少观察 10 到 20 秒，不要只看前 1 到 2 秒。
4. 前几帧 `Sonar` 可能是 `nan`，先等它变成正常数值再判断效果。
5. 每次只改一个参数，或者只沿一个方向改，不要一口气同时把两个参数都改掉。

## 推荐的调参顺序

建议顺序是：

1. 先确认重力补偿符号没错。
2. 先把 `var_sonar` 调到合理范围。
3. 再调 `var_imu_z`。
4. 最后再做一次小范围联合微调。

原因很简单：

- `var_sonar` 主要决定 sonar 校正有多强。
- `var_imu_z` 主要决定 IMU prediction 的不确定度有多大。
- 如果一开始两个一起乱动，你很难判断问题到底出在 prediction 还是 correction。

## 一次完整的调参流程

下面这套流程是最实用的。

### 第 1 步：先跑 baseline

先保持：

- `var_imu_z = 1.0`
- `var_sonar = 1.0`

运行 10 到 20 秒，重点记 4 个现象：

- `Pose.z` 会不会持续单方向漂移
- `Twist.z` 会不会明显不接近 0
- `Pose.z` 和 `Sonar` 是很贴近，还是长期偏离
- `ErrPose.z` 是围绕小范围波动，还是越来越大

建议你每次都简单记一行：

```text
var_imu_z=?, var_sonar=?, Pose.z表现=?, Twist.z表现=?, ErrPose.z表现=?
```

### 第 2 步：先扫 `var_sonar`

先固定：

- `var_imu_z = 1.0`

然后只改 `var_sonar`。  
推荐按数量级试，不要一点点慢慢挪：

- `1.0`
- `0.1`
- `0.01`

如果觉得太激进，再回到中间值附近细调，例如：

- `0.2`
- `0.5`
- `2.0`

判断规则：

- 如果 `Pose.z` 抖得厉害，几乎跟着 `Sonar` 原始噪声一起跳
  - `var_sonar` 太小，应该增大

- 如果 `Pose.z` 明显比 `Sonar` 更平滑，但长期和 `Sonar` 偏很多
  - `var_sonar` 太大，应该减小

- 如果 `Pose.z` 基本跟得上 `Sonar`，但又没有明显抖动
  - 这个范围通常就比较合适

### 第 3 步：再扫 `var_imu_z`

固定上一步挑出来的 `var_sonar`，只改 `var_imu_z`。

同样建议先按数量级试：

- `0.1`
- `1.0`
- `10.0`

然后再在较好的区间里细调，例如：

- `0.2`
- `0.5`
- `2.0`
- `5.0`

判断规则：

- 如果 `Twist.z` 长期偏离 0，而且 `Pose.z` 有慢慢漂走的趋势
  - 通常说明 prediction 影响太强，可以先尝试增大 `var_imu_z`

- 如果估计结果过分依赖 sonar，表现得很抖，像是每次量测一来就被强行拉走
  - `var_imu_z` 可能太大，可以适当减小

- 如果 `Twist.z` 更接近 0，`ErrPose.z` 也不再明显累计
  - 说明这个方向通常是对的

### 第 4 步：联合微调

当你已经找到一个大致范围后，再做最后一轮小改动：

- 如果整体偏抖，优先增大 `var_sonar`
- 如果整体偏慢、偏钝，优先减小 `var_sonar`
- 如果主要问题是慢慢漂，优先增大 `var_imu_z`
- 如果主要问题是 correction 过头，优先减小 `var_imu_z`

联合微调时，不要再按 10 倍改。  
改动幅度尽量控制在：

- 乘 `2`
- 或除 `2`

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

## 怎么判断该往哪个方向改

如果你不确定该改哪一个，就直接按下面这个对照表判断：

- `Pose.z` 很抖，和 `Sonar` 一样抖：
  - 先增大 `var_sonar`

- `Pose.z` 很稳，但和 `Sonar` 长期偏很多：
  - 先减小 `var_sonar`

- `Twist.z` 长期不接近 0，并且 `Pose.z` 有慢慢漂的趋势：
  - 先增大 `var_imu_z`

- 调大 `var_imu_z` 以后，`Pose.z` 开始变得过于依赖 sonar、看起来更吵：
  - 说明你调过头了，回退一点

- 两个参数怎么改都没明显效果：
  - 优先检查代码公式，而不是继续盲调

## 一个 10 分钟内能做完的实操版本

如果你只想先把作业调到“够用”，可以直接按这个顺序做：

1. 用默认值 `1.0 / 1.0` 跑一遍。
2. 把 `var_sonar` 改成 `0.1` 再跑一遍。
3. 如果太抖，就回到 `0.5` 或 `1.0`；如果不太抖且更贴近量测，就继续保留较小值。
4. 在确定的 `var_sonar` 下，把 `var_imu_z` 试成 `10.0`。
5. 如果 `Twist.z` 和 `ErrPose.z` 比之前更稳定，就保留更大的 `var_imu_z`；如果更吵，就往回退到 `5.0`、`2.0` 一类。
6. 最后再只改一次较小步长，把结果收敛到你觉得最稳定的组合。

这个流程的目的不是求“理论最优”，而是快速得到一个：

- 不发散
- 不明显漂移
- `Pose.z` 和 `Sonar` 基本一致
- `Twist.z` 基本接近 0

的可交作业参数。

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
