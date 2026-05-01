# 逐际动力 WF_TRON1A 机器人 — 激光里程计与导航开发

 本仓库整体以 GNU General Public License v3 (GPL-3.0) 授权；详情见 /LICENSE。仓库中包含 limxdynamics 的 Apache-2.0 代码（保留于 src/limxsdk-lowlevel/LICENSE）。

## 概览
本仓库记录并包含在 WF_TRON1A 差速驱动机器人平台上，为解决差速里程计在异常工况（如单轮打滑/卡住）下引起的大位姿误差所做的开发工作。核心思路是：外接二维激光雷达（雷神 N10p 系列），使用基于激光扫描的 RF2O（Range Flow-based Odometry）算法估计里程计增量，替代或辅助原车载差速里程计，从而提高导航的鲁棒性与精度。

README 主要内容：
- 如何编译与快速启动（软件使用方式）
- 激光里程计（RF2O）算法简介与实现要点
- 节点参数与运行示例
- 在仿真与真机测试中得到的效果摘要（结论化说明）
- 仓库结构与子模块说明

（注：仓库内包含若干子模块（motion SDK、可视化、驱动等），有关真机的硬件操作请参考逐际动力WF系列机器人硬件操作文档；此 README 主要介绍软件使用和算法实现与效果。）

## 快速开始（软件构建与运行）
本工程建议在Ubuntu22.04 + ROS2 humble系统下开发

1. 依赖与工具安装（onnxruntime等）  
```bash
# 下载onnxruntime v1.10.0（适配ROS2 Humble）
wget https://github.com/microsoft/onnxruntime/releases/download/v1.10.0/onnxruntime-linux-x64-1.10.0.tgz
# 解压文件
tar xvf onnxruntime-linux-x64-1.10.0.tgz
# 复制依赖至系统目录，确保系统可识别
sudo cp -a onnxruntime-linux-x64-1.10.0/include/* /usr/include
sudo cp -a onnxruntime-linux-x64-1.10.0/lib/* /usr/lib
# 安装Gazebo与ROS2的控制接口
sudo apt install ros-humble-gazebo-ros2-control
# 安装机器人运动控制工具
sudo apt install ros-humble-rqt-robot-steering -y
```

2. 克隆仓库并进入工作区
```bash
mkdir -p ~/limx_ws/src
cd ~/limx_ws/src
git clone https://github.com/F-Morry/limx_ws.git
cd ~/limx_ws
```

3. 编译及机器人型号设置
```bash
source /opt/ros/humble/setup.bash
# 编译工程（Release模式，优化编译速度与运行性能）
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release

# 机器人型号与训练环境配置（适配WF_TRON1A）
# 设置机器人型号为WF_TRON1A（替换默认的PF_P441C）
echo 'export ROBOT_TYPE=WF_TRON1A' >> ~/.bashrc && source ~/.bashrc
# 设置训练环境为isaaclab（替换默认的isaacgym）
echo 'export RL_TYPE=isaaclab' >> ~/.bashrc && source ~/.bashrc  
```

4. 启动Gazebo仿真
```bash
source install/setup.bash
# 启动WF_TRON1A仿真环境
ros2 launch robot_hw pointfoot_hw_sim.launch.py
```

5. 启动 RF2O 里程计节点
```bash
# 启动仓库内的 rf2o launch（示例）
ros2 launch rf2o_laser_odometry rf2o_laser_odometry.launch.py 
```

6. （可选）启动 IMU 时间戳同步转换工具（若出现IMU时间戳与雷达时间戳不匹配，则需要做 IMU/时间戳预处理，仿真中可忽略，实机需根据实际情况分析）
```bash
# 假设 imustamp 单位为微秒，则 scale = 1e-6
ros2 run imu_sync_converter imu_converter_node --ros-args -p imustamp_scale:=0.000001 -p alpha:=0.05
```

## 算法简介 — RF2O（激光基里程计）
RF2O（Range Flow-based Odometry）是一种针对平面（2D）激光扫描序列估计传感器平移与旋转的方法。核心思想：
- 对每个激光点建立“距离流 / range flow”约束，将点的相对运动写成传感器线速度和角速度的函数；
- 将所有点的几何约束组合，最小化一个鲁棒损失函数以求解机体在两个连续扫描之间的运动增量；
- 该方法计算量小、实时性好（文献与实现报告在单核上能达到约 0.9 ms/帧 的计算开销），对低成本 2D 雷达平台十分适用。

更多学术细节与参考：
- 文章：Planar Odometry from a Radial Laser Scanner. A Range Flow-based Approach. ICRA 2016（论文与详细推导请参考原论文及实现仓库）

实现要点（基于仓库实现与节点源码）
- 节点名为 `CLaserOdometry2DNode`，位于包 `rf2o_laser_odometry`。
- 主要输入：连续的 sensor_msgs/LaserScan（默认 topic `/scan`）。
- 输出：nav_msgs/Odometry（默认 topic `/odom_rf2o`）并可发布 TF（默认开启）。
- 节点初始化需要知道激光在 base_link 下的位姿（通过 TF 获取），或通过外部初始位姿话题输入初始姿态。
- 算法在第一帧做参数初始化，之后每帧调用 odometryCalculation 进行估计并发布里程计/TF。

## 节点参数（默认值来自源码）
以下参数均可通过 ROS 参数设定或在 launch 中传入：

- laser_scan_topic (string, default: "/scan")
  - 激光雷达扫描输入 topic
- odom_topic (string, default: "/odom_rf2o")
  - 发布里程计的 topic
- base_frame_id (string, default: "base_link")
  - 机器人基坐标系
- odom_frame_id (string, default: "odom")
  - 里程计参考坐标系
- publish_tf (bool, default: true)
  - 是否发布 odom -> base_link 的 TF
- init_pose_from_topic (string, default: "/base_pose_ground_truth")
  - （可选）从指定 topic 读取初始里程计作为初始姿态；若不指定则默认以零位姿初始化
- freq (double, default: 10.0)
  - 节点主循环频率（Hz）

运行时注意：
- 节点会在收到第一帧扫描后从 TF 中读取激光到 base_link 的变换并初始化内部结构（函数 setLaserPoseFromTf）。因此确保 TF 树在节点启动后可查询到激光到 base_link 的变换。
- 若使用 best_effort QoS（典型于部分驱动发布），请确保网络/节点 QoS 与驱动匹配以避免丢帧。

## 输入/输出说明
- 输入
  - sensor_msgs/LaserScan — 激光扫描数据（高频）
  - （可选）nav_msgs/Odometry — 用于初始化的“地面真实位姿”
- 输出
  - nav_msgs/Odometry — RF2O 估计的位姿与速度（topic: "/odom_rf2o"，可由参数修改）
  - TF（odom -> base_link） — 若 publish_tf = true，则发布对应转换

## 在仿真与实机测试中的实现效果分析
通过与Simple2D_odometry算法对比我们发现，RF2O算法具有较强的鲁棒性很稳定性，在遇到机器人机身晃动，雷达扫描平面俯仰角发生较大变化时仍然能够较为精准的里程计输出， 
同时该算法也解决了原生差速里程计的缺陷，不足之处在于该雷达里程计算法受限于雷达信息的发布频率导致输出的里程计轨迹不够平滑，后续工作中可以将原生里程计与雷达里程计进行结合  
输出更为平滑和精准的里程计


### 测试数据与图像  
下面分别展示里程计算法在仿真和实机上的效果，采用Rosbag收集运动数据，并在Plotjuggler进行可视化  
其中仿真中的对比里程计为odom里程计，实机中对比的里程计为机器人原生差速里程计
- 仿真轨迹对比
  
<img width="1280" height="728" alt="sim_rf2ovsodom_xy" src="https://github.com/user-attachments/assets/bee00121-8f3f-4901-842a-8ab1a085e949" />
  
- 仿真轨迹x、y坐标对比
<img width="1280" height="728" alt="sim_rf2ovsodom_xay" src="https://github.com/user-attachments/assets/fb6f86f4-23e1-43f7-bfc2-3b0f1073421f" />

- 实机轨迹对比  
<img width="1280" height="909" alt="real_rf2ovsdiff_xy" src="https://github.com/user-attachments/assets/4bbeba53-698c-4b3c-96be-b63e862b4c61" />
  
- 实机轨迹x、y坐标对比
<img width="1280" height="902" alt="real_rf2ovsdiff_xay" src="https://github.com/user-attachments/assets/af095ca4-0ba5-4ebb-889b-2bef86e40080" />

  
## 仓库结构（与关键子模块）
- src/rf2o_laser_odometry/ — RF2O 算法实现（包含 C++ 源码、launch、README、LICENSE）
  - 关键文件：`src/CLaserOdometry2D.cpp`、`src/CLaserOdometry2DNode.cpp`（节点实现与发布逻辑）
- src/imu_sync_converter/ — IMU 时间戳转换与同步工具（用于在不同时间单位间转换与滤波）
  - 关键文件：`src/imu_converter_node.cpp`
- src/limxsdk-lowlevel/ — LimX 运动控制 SDK（示例、接口与使用说明，包含 ROS1/Noetic 的安装说明）
- src/lidar/ — 雷神n10p雷达驱动与消息定义（包括 lslidar 驱动）
- src/robot-description/ — 机器人建模文件（mesh、urdf、xacro文件等）
- src/robot-visualization/ — 可视化/调试工具（包含 ROS1/ROS2 支持的可视化工具）
- src/tron1-rl-deploy-ros2/ — 仿真与实机机器人控制及仿真启动节点（可同时启动slam、里程计及导航节点）
---
