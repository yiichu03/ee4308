# EE4308 Lab 2 运行手册

## 环境要求

根据实验说明，推荐环境为：

- Ubuntu 24.04 LTS
- ROS 2 Jazzy

参考文件：`docs/lab2.md`

## 已确认的本机 Docker 环境

你机器上已经有一个可用的课程容器：

- 容器名：`ee4308_jazzy_proj2`
- 镜像：`osrf/ros:jazzy-desktop-full-noble`
- 容器内工作目录：`/ws/ee4308`
- Host 挂载目录：`/home/liuyi/projects/ee4308_proj2 -> /ws/ee4308`

另外，旧的 `lab1/proj1` 环境现在也有自己的独立容器：

- 容器名：`ee4308_jazzy_proj1`
- Host 挂载目录：`/home/liuyi/projects/ee4308_proj1 -> /ws/ee4308`

旧的 `ee4308_jazzy_legacy_oldpath` 只是迁移前的备份容器，不建议继续使用。

## 使用 `ee4308_jazzy_proj2` 的基本命令

如果你要在容器里打开 Gazebo / RViz 图形界面，先在宿主机终端运行：

```bash
xhost +SI:localuser:root
```

启动容器：

```bash
docker start ee4308_jazzy_proj2
```

进入容器：

```bash
docker exec -it ee4308_jazzy_proj2 bash
```

进入容器后，工作区目录是：

```bash
cd /ws/ee4308
```

进入容器后先 source ROS 2 Jazzy：

```bash
source /opt/ros/jazzy/setup.bash
```

如果容器里还没有 TurtleBot3 Gazebo 模型资源，先安装一次：

```bash
apt-get update
apt-get install -y ros-jazzy-turtlebot3-gazebo
```

然后再编译：

```bash
cd /ws/ee4308
colcon build --symlink-install
source install/setup.bash
```

启动仿真：

```bash
ros2 launch ee4308_bringup proj2_sim.launch.py
```

## 工作区目录

下面的命令默认你在本仓库根目录执行：

```bash
cd /home/liuyi/projects/ee4308_proj2
```

## 构建与运行

编译工作区：

```bash
colcon build --symlink-install
```

每次打开新终端后都要重新 source：

```bash
source install/setup.bash
```

启动仿真：

```bash
ros2 launch ee4308_bringup proj2_sim.launch.py
```

如果你在 VirtualBox 中运行：

```bash
ros2 launch ee4308_bringup proj2_sim.launch.py libgl:=True
```

## Lab 2 相关文件

- `src/ee4308_drone/src/estimator.cpp`
- `src/ee4308_drone/include/ee4308_drone/estimator.hpp`
- `src/ee4308_bringup/params/proj2.yaml`

Lab 2 只要求完成竖直方向状态估计的这两部分：

- `Estimator::callbackSubIMU_()`
- `Estimator::callbackSubSonar_()`

## 开发时常用命令

修改 C++ 代码后重新编译：

```bash
colcon build --symlink-install
source install/setup.bash
```

如果你只改了 `proj2.yaml` 参数，通常不需要重新编译，只需要重新启动：

```bash
source install/setup.bash
ros2 launch ee4308_bringup proj2_sim.launch.py
```

## ROS 2 检查命令

查看所有 topic：

```bash
ros2 topic list
```

查看估计器输出一次：

```bash
ros2 topic echo /drone/odom --once
```

查看 IMU 输入一次：

```bash
ros2 topic echo /drone/imu --once
```

查看 Sonar 输入一次：

```bash
ros2 topic echo /drone/sonar --once
```

查看发布频率：

```bash
ros2 topic hz /drone/imu
ros2 topic hz /drone/sonar
ros2 topic hz /drone/odom
```

查看 estimator 参数：

```bash
ros2 param list /drone/estimator
ros2 param get /drone/estimator var_imu_z
ros2 param get /drone/estimator var_sonar
ros2 param get /drone/estimator verbose
```

## 快速自检

启动仿真后，先看这几件事：

1. turtle 会开始运动，drone 在初始时刻应基本保持静止。
2. 终端会持续打印 `Pose`、`Twist`、`ErrPose`、`ErrTwis`、`Sonar`。
3. `Pose` 里的 `z` 与 `Sonar` 的高度应该大致接近。
4. 当无人机没有明显上下运动时，`Twist` 里的 `z` 速度应接近 0。
5. 如果 `z` 或 `z` 速度持续漂移，优先检查 `callbackSubIMU_()` 里的重力补偿符号。
6. 如果估计值抖动太明显、过分跟着 sonar 跳动，增大 `var_sonar`。
7. 如果估计值修正太弱、长时间漂移，减小 `var_sonar`。

## Lab 2 常见工作流程

1. 修改 `src/ee4308_drone/src/estimator.cpp`
2. 重新编译工作区
3. `source install/setup.bash`
4. 启动仿真
5. 观察 estimator 终端输出
6. 在 `src/ee4308_bringup/params/proj2.yaml` 中调 `var_imu_z` 和 `var_sonar`
7. 重新启动并比较效果

## Docker 备注

当前这个仓库里没有 `Dockerfile` 或 `docker-compose` 文件，所以你之前配过的 EE4308 Docker 环境大概率不在这个仓库目录里。

你现在进行 `lab2/proj2` 时，最常用的容器就是 `ee4308_jazzy_proj2`：

```bash
docker start ee4308_jazzy_proj2
docker exec -it ee4308_jazzy_proj2 bash
```

进入后使用：

```bash
cd /ws/ee4308
source /opt/ros/jazzy/setup.bash
```

### 查找可能的容器

查看正在运行的容器：

```bash
docker ps
```

查看所有容器，包括已经停止的：

```bash
docker ps -a
```

用表格形式看，更容易扫名字：

```bash
docker ps -a --format 'table {{.Names}}\t{{.Image}}\t{{.Status}}\t{{.RunningFor}}'
```

筛选可能和课程有关的容器：

```bash
docker ps -a --format '{{.Names}}\t{{.Image}}' | grep -Ei 'ee4308|ros|jazzy|ubuntu'
```

### 查找可能的镜像

查看镜像：

```bash
docker images
```

用表格形式查看：

```bash
docker images --format 'table {{.Repository}}\t{{.Tag}}\t{{.ID}}\t{{.CreatedSince}}'
```

筛选可能和课程有关的镜像：

```bash
docker images --format '{{.Repository}}:{{.Tag}}\t{{.ID}}' | grep -Ei 'ee4308|ros|jazzy|ubuntu'
```

### 检查某个候选容器

查看完整信息：

```bash
docker inspect <container_name_or_id>
```

只看它使用的镜像：

```bash
docker inspect -f '{{.Config.Image}}' <container_name_or_id>
```

查看启动命令：

```bash
docker inspect -f '{{.Path}} {{join .Args " "}}' <container_name_or_id>
```

### 重新进入某个容器

启动一个已停止容器：

```bash
docker start <container_name_or_id>
```

进入容器：

```bash
docker exec -it <container_name_or_id> bash
```

如果容器里没有 `bash`：

```bash
docker exec -it <container_name_or_id> sh
```

### 查看日志

```bash
docker logs <container_name_or_id>
```

只看最近一些日志：

```bash
docker logs --tail 100 <container_name_or_id>
```

### 在家目录里找 Docker 配置

查找你以前可能写过的 Docker 文件：

```bash
find ~ -maxdepth 4 \( -iname 'Dockerfile*' -o -iname 'docker-compose*.yml' -o -iname 'docker-compose*.yaml' -o -path '*/.devcontainer/*' \) 2>/dev/null
```

查找和 EE4308 / ROS / Jazzy 相关的本地文本：

```bash
rg -n -i 'ee4308|ros2|jazzy|ubuntu 24|ubuntu24|gazebo|rviz' ~ 2>/dev/null
```

### 如果找到了错误的容器

停止一个正在运行的容器：

```bash
docker stop <container_name_or_id>
```

删除一个已停止容器：

```bash
docker rm <container_name_or_id>
```

删除一个镜像：

```bash
docker rmi <image_id_or_name>
```

如果你不能确定哪个才是课程环境，不要急着删。

## 提交提醒

根据 Lab 2 说明，需要提交的是：

- Canvas 上 3 道问答题的答案
- 一个按要求打包的代码 zip 文件

说明文档没有要求单独提交：

- 实验报告
- 截图
- 视频
- 单独的实验结果文档
