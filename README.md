moveit assistant 、libmoveit.so库版本匹配问题解决：回退版本。

```bash
wget http://snapshots.ros.org/humble/2025-06-18/ubuntu/pool/main/r/ros-humble-rviz-common/ros-humble-rviz-common_11.2.17-1jammy.20250617.234657_amd64.deb

http://ec2-35-174-230-37.compute-1.amazonaws.com/humble/2024-12-05/ubuntu/pool/main/r/ros-humble-moveit-visual-tools/


sudo dpkg -i ros-humble-rviz-common_11.2.17-1jammy.20250617.234657_amd64.deb
```




# build
```bash
colcon build --cmake-args -DBUILD_TESTING=False --symlink-install
```
# run
```bash
source install/setup.bash


ros2 launch dual_arm_moveit_config sim.launch.py


ros2 launch motion_plan motion_plan.launch.py


ros2 run motion_plan jointprinter
```

# bug点记录
1. 注意gazebo显示模型必须`file://$(find ...)`而不是`package://`

2. 单独gazebo classic启动文件无需启动ros2 control node

3. 启动gazebo classic仿真时，需要添加`<gazebo>`标签的`<plugin>`标签的`<plugin filename="libgazebo_ros_control.so">`，`name`必须设置为`gazebo_ros_control`，
必须加入控制器yaml文件

4. gazebo和moveit共同启动，想正常模仿夹爪，夹爪的控制器在`ros2_control`必须有mimic的关联的joint，
且需要在urdf中添加`<mimic>`的虚拟joint列表避免刷屏报错

5. 用`movegroup`节点获取末端EEF位姿可能不如tf准确，因为`movegroupinterface`是局部机械臂的运动链

6. 启动文件中的spawn_entity需要超时参数，否则会报错