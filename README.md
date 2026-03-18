## 项目子工程基本介绍

1 ur_driver          # ur机械臂ros驱动，包含urdf模型、moveit运动控制等.

2 ur_description       # 包含urdf模型、config等.

3 ros2_robotiq_gripper、serial            # gripper 2f-85夹爪驱动.

4 robotiq_description  # 夹爪urdf模型、config配置.

5 dual_ur_moveit_config  # ur双臂config、moveit等.

## 示例：双臂控制
首先，需要检查双臂的ip地址是否与PC在同一网段，并且分别记录左右机械臂的ip，此项目中为192.168.1.101和192.168.1.50.如果不一致，则需要修改/ur_driver/ur_robot_driver/launch/dual_ur_control.launch.py中的robot_ip。其次，需要给两个机械臂安装External Control，并且配置端口分别为60002和50002。最后就可以分别执行一下指令与机械臂建立通信与控制。

1 开启双臂ros_driver

ros2 launch ur_robot_driver dual_ur_control.launch.py  

2 双臂测试

注：如果启动过程中rviz2中机器人是黑的关闭多启几次

ros2 launch dual_ur_moveit_config dual_ur_moveit.launch.py

3 双臂同时独立测试

ros2 launch dual_ur_pnc dual_ur_separate_test

## 示例：双夹爪控制
1 开启双臂ros_driver

```bash
ros2 launch ur_robot_driver dual_ur_control.launch.py
```

2 开启双夹爪服务

```bash
ros2 launch robotiq_2f_gripper_hardware robotiq_2f_gripper_launch.py
```

3 双夹爪测试

```bash
ros2 launch robotiq_2f_gripper_hardware gripper_test_launch.py
```

如果想在终端直接测试可用下面的命令

```bash
ros2 action send_goal /left/robotiq_2f_gripper_action robotiq_2f_gripper_msgs/action/MoveTwoFingerGripper "{target_position: 0.05, target_speed: 0.1, target_force: 0.1}"
```

## 示例：单个夹爪驱动控制

1 开启右臂ros_driver(将dual_ur_control.launch.py 中最后的left_with_namespace注释掉)

ros2 launch ur_robot_driver dual_ur_control.launch.py right_robot_ip:=192.168.1.50 right_ur_type:=ur3e right_use_tool_communication:=true right_tool_voltage:=24 right_tool_parity:=0 right_tool_baud_rate:=115200 right_tool_stop_bits:=1 right_tool_rx_idle_chars:=1.5 right_tool_tx_idle_chars:=3.5 right_tool_device_name:=/tmp/ttyUR  



right_use_tool_communication:=true right_tool_voltage:=24 right_tool_parity:=0 right_tool_baud_rate:=115200 right_tool_stop_bits:=1  right_tool_rx_idle_chars:=1.5 right_tool_tx_idle_chars:=3.5 right_tool_device_name:=/tmp/ttyUR 这些参数都是跟夹爪通信相关的,需要传

2 右边夹爪测试

./build/robotiq_hardware_tests/full_test 

## 示例：摄像头标定





## 示例：pnp任务规划

1 驱动控制 

1.1 机械臂驱动

```bash
ros2 launch ur_robot_driver dual_ur_control.launch.py  
```

1.2 夹爪驱动

```bash
ros2 launch robotiq_2f_gripper_hardware robotiq_2f_gripper_launch.py
```

1.3 开启rviz

```bash
ros2 launch dual_ur_moveit_config dual_ur_moveit.launch.py
```

1.4.1 启动 pnp demo

```bash
ros2 launch dual_ur_pnc dual_ur_pnc.launch.py
```

或者启动

1.4.2 启动 pnp 服务端

```bash
ros2 launch robot_action robot_action.launch.py 
```

2 仿真模式

2.1 开启gazebo rviz

```bash
ros2 launch ur_description test_gazebo_moveit.launch.py
```

2.2 pnp demo

```bash
ros2 launch dual_ur_pnc dual_ur_pnc.launch.py
```

## 示例：仿真模式
1 开启gazebo仿真机械臂(不要联网)

ros2 launch ur_description test_gazebo.launch.py

如果出现Libgazebo_ros2_control相关报错以及controller_manager无法启动，运行source /usr/share/gazebo/setup.bash(或者setup.sh)

2 开启moveit运动控制

ros2 launch ur_description test_moveit.launch.py
（这里不用ros2 launch dual_ur_moveit_config dual_ur_moveit.launch.py可能原因
ur控实际机械臂是ros2_controllers.yaml+controllers.yaml    ros2_controllers.yaml与controllers.yaml中控制器不一样 ur双臂的可能有些差异
ur控gazebo中机械臂ros2_controllers.yaml+moveit_controllers.yaml   ros2_controllers.yaml与moveit_controllers.yaml中控制器一样，正常应该都是一样的）

3 启动action server

ros2 launch robot_action robot_action.launch.py

4 左臂移动到操作点

ros2 action send_goal /control_robot common_msgs/action/CtrlCmd "{cmd_type: 1, info: 'Move Left arm to position', actor: 1, gripper_cmd: 1, force_position_control: 0, sport_mode: 1, position_limit: 0, joints_angle: [-2.60, -106.88, -75.38, -118.26, -2.04, 210.87]}"

## 示例：力控模式

1 开启双臂ros_driver

```bash
ros2 launch ur_robot_driver dual_ur_control.launch.py
```

2 开启力控

```bash
ros2 launch robot_wrench_frame mechanical_arm_control.launch.py
```

注意面板上速度不要调太快30%左右，参考系以为left_base_link_inertia 
或者right_base_link_inertia ,可在demon中查看使用方法

***

查看控制器

ros2 control list_controllers -c /left/controller_manager


ros2 control switch_controllers -c /left/controller_manager --stop  scaled_joint_trajectory_controller

ros2 control switch_controllers -c /left/controller_manager --start cartesian_compliance_controller

## 示例：坐标变换
1 静态发布ee_link和camera_optical_link

```bash
ros2 launch static_broadcaster_listener static_transform_publisher.launch.py
```
生成tf树命令
```bash
ros2 run tf2_tools view_frames
```

## 示例：与决策端联调双臂运动

1 机械臂驱动

```bash
ros2 launch ur_robot_driver dual_ur_control.launch.py  
```

2 启动moveit

```bash
ros2 launch dual_ur_moveit_config dual_ur_moveit.launch.py
```

3 夹爪驱动

```bash
ros2 launch robotiq_2f_gripper_hardware robotiq_2f_gripper_launch.py
```

4 启动action server
```bash
ros2 launch robot_action robot_action.launch.py
```

5 手动请求
```bash
左臂
移动到home点
ros2 action send_goal /control_robot common_msgs/action/CtrlCmd "{cmd_type: 1, info: 'Move Left arm to position', actor: 1, gripper_cmd: 1, force_position_control: 0, sport_mode: 1, position_limit: 0, joints_angle: [0.0, -90.0, 0.0, -90.0, 0.0, 0.0]}"

移动到操作点
ros2 action send_goal /control_robot common_msgs/action/CtrlCmd "{cmd_type: 1, info: 'Move Left arm to position', actor: 1, gripper_cmd: 1, force_position_control: 0, sport_mode: 1, position_limit: 0, joints_angle: [-2.60, -106.88, -75.38, -118.26, -2.04, 210.87]}"

移动到抓取点,垂直 0.5361, -0.40955, 1.00801, 0.7107, 0.70343, -0.00466, 0.00655
ros2 action send_goal /control_robot common_msgs/action/CtrlCmd "{cmd_type: 1, info: 'Move Left arm to position', actor: 1, gripper_cmd: 1, pose: {header: {stamp: {sec: 0, nanosec: 0}, frame_id: 'world'}, pose: {position: {x: 0.5361, y: -0.40955, z: 1.00801}, orientation: {x: 0.7107, y: 0.70343, z: -0.00466, w: 0.00655}}}, force_position_control: 0, sport_mode: 0, position_limit: 0}"


移动到观察点
ros2 action send_goal /control_robot common_msgs/action/CtrlCmd "{cmd_type: 1, info: 'Move Left arm to position', actor: 1, gripper_cmd: 1, force_position_control: 0, sport_mode: 1, position_limit: 0, joints_angle: [-38.82, -14.06, -98.42, -103.75, 91.11, 2.33]}"

打开左夹爪
ros2 action send_goal /control_robot common_msgs/action/CtrlCmd "{cmd_type: 1, info: 'Open Left gripper', actor: 3, gripper_cmd: 1}"

关闭左夹爪
ros2 action send_goal /control_robot common_msgs/action/CtrlCmd "{cmd_type: 1, info: 'Close Left gripper', actor: 3, gripper_cmd: 0}"


右臂
观察点1
ros2 action send_goal /control_robot common_msgs/action/CtrlCmd "{cmd_type: 1, info: 'Move Right arm to position', actor: 2, gripper_cmd: 1, force_position_control: 0, sport_mode: 1, position_limit: 0, joints_angle: [16.10, -83.26, 82.68, -89.71, -84.11, 5.50]}"

观察点2
ros2 action send_goal /control_robot common_msgs/action/CtrlCmd "{cmd_type: 1, info: 'Move Right arm to position', actor: 2, gripper_cmd: 1, force_position_control: 0, sport_mode: 1, position_limit: 0, joints_angle: [5.50, -73.18, 15.49, -59.54, -54.04, 304.61]}"

移动到home点
ros2 action send_goal /control_robot common_msgs/action/CtrlCmd "{cmd_type: 1, info: 'Move Right arm to position', actor: 2, gripper_cmd: 1, force_position_control: 0, sport_mode: 1, position_limit: 0, joints_angle: [0.0, -90.0, 0.0, -90.0, 0.0, 0.0]}"

抓取点，可以到
ros2 action send_goal /control_robot common_msgs/action/CtrlCmd "{cmd_type: 1, info: 'Move Right arm to position', actor: 2, gripper_cmd: 1, pose: {header: {stamp: {sec: 0, nanosec: 0}, frame_id: 'world'}, pose: {position: {x: -0.34251, y: -0.31638, z: 0.84172}, orientation: {x: 0.0, y: 1.0, z: 0.0, w: 0.0}}}, force_position_control: 0, sport_mode: 0, position_limit: 0}"



预插入点，可以到 -0.26899, -0.34054, 1.1274, 0.50714, -0.49272, 0.51378, -0.48586
ros2 action send_goal /control_robot common_msgs/action/CtrlCmd "{cmd_type: 1, info: 'Move Right arm to position', actor: 2, gripper_cmd: 1, pose: {header: {stamp: {sec: 0, nanosec: 0}, frame_id: 'world'}, pose: {position: {x: -0.26899, y: -0.34054, z: 1.1274}, orientation: {x: 0.50714, y: -0.49272, z: 0.51378, w: -0.48586}}}, force_position_control: 0, sport_mode: 0, position_limit: 0}"

打开右夹爪
ros2 action send_goal /control_robot common_msgs/action/CtrlCmd "{cmd_type: 1, info: 'Open Right gripper', actor: 4, gripper_cmd: 1}"

关闭右夹爪
ros2 action send_goal /control_robot common_msgs/action/CtrlCmd "{cmd_type: 1, info: 'Close Right gripper', actor: 4, gripper_cmd: 0}"
```

# test tongyigedian
ros2 action send_goal /control_robot common_msgs/action/CtrlCmd "{cmd_type: 1, info: 'Move Right arm to position', actor: 2, gripper_cmd: 1, pose: {header: {stamp: {sec: 0, nanosec: 0}, frame_id: 'world'}, pose: {position: {x: -0.1028, y: -0.29453, z: 1.241}, orientation: {x: -0.49982, y: 0.5002, z: -0.49945, w: 0.50054}}}, force_position_control: 0, sport_mode: 0, position_limit: 0}"

ros2 action send_goal /control_robot common_msgs/action/CtrlCmd "{cmd_type: 1, info: 'Move Right arm to position', actor: 2, gripper_cmd: 1, force_position_control: 0, sport_mode: 1, position_limit: 0, joints_angle: [0.0, -33.61, -48.52, 34.74, 0.0, 227.48]}"

## trac_ik求解器
1 使用trac_ik要在容器中安装nlopt 
git clone https://kkgithub.com/stevengj/nlopt.git 

cd nlopt/  

mkdir build 

cd build 

cmake -DCMAKE_INSTALL_PREFIX=/usr .. 

make 

sudo make install 

2 将trac_ik下载到planning_control下
https://kkgithub.com/ravnicas/trac_ik.git

3 更改dual_arm_moveit_config的config中kinematics.yaml，将kdl_kinematics_plugin/KDLKinematicsPlugin改为trac_ik_kinematics_plugin/TRAC_IKKinematicsPlugin


## 解决速度限制问题
```
sudo apt-get install ros-humble-ur-client-library 这种安装方式会出现下面速度限制问题，用上面安装方式可以解决速度限制问题
Velocity 54.112 required to reach the received target [-3641786652204.927734, -1140532155447.430664, 1166022624219.684814, 10909127314.458231, -64363636502.848495, -9701334695426.744141] within 0.002 seconds is exceeding the joint velocity limits

解决：
https://kkgithub.com/UniversalRobots/Universal_Robots_ROS2_Driver/issues/1235
sudo apt-get remove ros-humble-ur-client-library 卸载安装的ur-client-library
git clone -b improve_limit_check https://kkgithub.com/urfeex/Universal_Robots_Client_Library.git 文件名改成ur_client_library放到ur_driver下
colcon build –-packages-select ur_client_library --cmake-args -DCMAKE_BUILD_TYPE=Release
```

## 工程pull下来要在byd_servo下创建include/byd_servo文件夹，不然编译会报错



# test 线性插值（LERP）+ 球面插值（Slerp）函数
```
bool success = move_group_node->move_small_pose("left", goal->pose.pose, keep_orientation, 0.5, 1) == 0;

ros2 action send_goal /control_robot common_msgs/action/CtrlCmd "{cmd_type: 1, info: 'Move Left arm to position', actor: 1, gripper_cmd: 1, pose: {header: {stamp: {sec: 0, nanosec: 0}, frame_id: 'world'}, pose: {position: {x: 0.5361, y: -0.40955, z: 1.00801}, orientation: {x: 0.7107, y: 0.70343, z: -0.00466, w: 0.00655}}}, force_position_control: 0, sport_mode: 0, position_limit: 1}"
```

# build
```
colcon build --cmake-args -DBUILD_TESTING=False
```