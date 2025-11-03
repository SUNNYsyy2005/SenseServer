# SenseServer

高颢嘉

# 注意事项

本代码为 SenseLabRobo 服务器端，用于多机全局建图与发送指令。本代码只适合部署在个人开发环境或服务器环境中，请勿将此代码部署在实车中。

# 下载与配置：

```bash
cd ~/
git clone https://github.com/gaohaojia/SenseServer
cd SenseServer
sudo apt update
sudo apt install ros-humble-desktop-full python3-rosdep ros-humble-pcl* ros-humble-foxglove-bridge -y
sudo rosdep init
rosdep update
rosdep install -r --from-paths src --ignore-src --rosdistro $ROS_DISTRO -y
```

# 配置服务器

修改 ~/SenseServer/src/robot_communication/launch/robot_communication.launch.py 的 network_ip 为服务器 ip，network_port 为服务器端口（一般不需要更改端口）。

# 编译

目前只测试过在 x86 架构上编译本代码。

```bash
bash ~/SenseServer/toBuild.sh
```

# 运行

将 [robot_count] 替换成机器人数量（机器人的最大 id＋1，当前支持的范围为 1-5）。

```bash
bash ~/SenseServer/run.sh [robot_count]
```

**注意**本仓库**robot**目录下为**single_robot_explore**的`src/communication`的代码