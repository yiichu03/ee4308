## 终端 A：启动 SLAM 仿真

A1）进入容器:
```
xhost +local:root
docker start -ai ee4308_jazzy
```
A2）在容器里启动 headless SLAM
```
export LIBGL_ALWAYS_SOFTWARE=1
export MESA_LOADER_DRIVER_OVERRIDE=llvmpipe
export GALLIUM_DRIVER=llvmpipe

cd /ws/ee4308
source install/setup.bash
export TURTLEBOT3_MODEL=burger
ros2 launch ee4308_bringup proj1_sim_slam.launch.py

```
## 终端 B：键盘遥控（teleop）

B1）进入同一个容器（不要开新的容器）
```
docker exec -it ee4308_jazzy bash
```
B2）在容器里启动 teleop
```
cd /ws/ee4308
source install/setup.bash
export TURTLEBOT3_MODEL=burger
ros2 run turtlebot3_teleop teleop_keyboard
```
要在终端B按wad就可以

## 保存map
```
source /ws/ee4308/install/setup.bash

ros2 run nav2_map_server map_saver_cli -f /ws/ee4308/src/ee4308_bringup/maps/proj1_sim

cd /ws/ee4308
colcon build --symlink-install
```

## Simulating Navigation with Nav2
```
cd /ws/ee4308
source install/setup.bash
export TURTLEBOT3_MODEL=burger

ros2 launch ee4308_bringup proj1_sim.launch.py
```
## 改完代码编译：
只编译相关包（更快）：
```
cd /ws/ee4308
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select ee4308_turtle
source install/setup.bash
```
补充建议：
这种流程里，改 C++ 代码要编译；只改 proj1.yaml 参数通常不需要编译，重启 launch 即可。
建议开一个“终端 C”专门 build，A/B 专门运行节点，避免混用。
如果出现“改了代码但行为没变”，先确认重启了 launch；还不行再用一次干净构建：rm -rf build install log && colcon build --symlink-install。
你保存 map 后再 colcon build 一般不是必须步骤（除非同时改了源码）。








