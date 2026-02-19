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

export LIBGL_ALWAYS_SOFTWARE=1
export MESA_LOADER_DRIVER_OVERRIDE=llvmpipe
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

ros2 launch ee4308_bringup proj1_sim.launch.py headless:=True
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

# 一些报错：
报错之一如下（主要体现在gazebo没有办法正确加载）：
```
[gazebo-1] kmsro: driver missing
[gazebo-1] libEGL warning: egl: failed to create dri2 screen
[gazebo-1] kmsro: driver missing
[gazebo-1] libEGL warning: egl: failed to create dri2 screen
```
C1) 容器场景（最常见）
在宿主机（有桌面那个）先执行一次：
xhost +si:localuser:root
启动容器时确保带上（示例）：
-e DISPLAY=$DISPLAY
-v /tmp/.X11-unix:/tmp/.X11-unix:rw
（推荐）--device=/dev/dri
容器内再确认：
echo $DISPLAY
ls /tmp/.X11-unix
```
root@liuyi:/ws/ee4308# exit
exit
(base) liuyi@liuyi:~/projects/ee4308$ xhost +si:localuser:root
localuser:root being added to access control list
(base) liuyi@liuyi:~/projects/ee4308$ docker start -ai ee4308_jazzy
root@liuyi:/ws/ee4308# echo $DISPLAY
ls /tmp/.X11-unix
:0
X0
root@liuyi:/ws/ee4308# export LIBGL_ALWAYS_SOFTWARE=1
export MESA_LOADER_DRIVER_OVERRIDE=llvmpipe
export GALLIUM_DRIVER=llvmpipe

export LIBGL_ALWAYS_SOFTWARE=1
export MESA_LOADER_DRIVER_OVERRIDE=llvmpipe
cd /ws/ee4308
source install/setup.bash
export TURTLEBOT3_MODEL=burger
ros2 launch ee4308_bringup proj1_sim_slam.launch.py
[INFO] [launch]: All log files can be found below /root/.ros/log/2026-02-13-02-31-18-324019-liuyi-25
```





