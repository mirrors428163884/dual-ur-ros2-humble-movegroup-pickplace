
## 项目简介
这是一个基于 ROS2 Humble 的机器人运动规划与控制系统，集成了 MoveIt! 运动规划框架和 Gazebo 仿真环境。

## 依赖版本问题解决
### Moveit Assistant 与 libmoveit.so 库版本匹配问题
由于版本不兼容导致的问题，解决方案为回退到特定版本：

```bash
wget http://snapshots.ros.org/humble/2025-06-18/ubuntu/pool/main/r/ros-humble-rviz-common/ros-humble-rviz-common_11.2.17-1jammy.20250617.234657_amd64.deb

http://ec2-35-174-230-37.compute-1.amazonaws.com/humble/2024-12-05/ubuntu/pool/main/r/ros-humble-moveit-visual-tools/

sudo dpkg -i ros-humble-rviz-common_11.2.17-1jammy.20250617.234657_amd64.deb
```

## 构建与运行

### 构建
```bash
colcon build --cmake-args -DBUILD_TESTING=False --symlink-install
```

### 运行
```bash
source install/setup.bash

# 启动 MoveIt 仿真环境
ros2 launch dual_arm_moveit_config sim.launch.py

# 启动 Gazebo 仿真
ros2 launch dual_arm_moveit_config sim_moveit_gazebo_all.launch.py

# 启动运动规划节点
ros2 launch motion_plan motion_plan.launch.py

ros2 run motion_plan stack

```

## 已知问题及解决方案

1. **Gazebo 模型路径格式** / *Gazebo model path format*
   - 注意：Gazebo 显示模型必须使用 `file://$(find ...)` 而不是 `package://` / *Note: Gazebo display models must use `file://$(find ...)` instead of `package://`*

2. **Gazebo Classic 单独启动** / *Gazebo Classic standalone launch*
   - 单独 Gazebo Classic 启动文件无需启动 ROS2 Control Node / *Standalone Gazebo Classic launch files don't need to start ROS2 Control Node*

3. **Gazebo Classic 插件配置** / *Gazebo Classic plugin configuration*
   - 启动 Gazebo Classic 仿真时，需要添加 `<gazebo>` 标签的 `<plugin>` 标签的 `<plugin filename="libgazebo_ros_control.so">`，[name](ur_hand_eye_calibration/flexbe_behavior_engine/flexbe_core/flexbe_core/behavior.py#L0-L0) 必须设置为 `gazebo_ros_control`，必须加入控制器 yaml 文件 / *When starting Gazebo Classic simulation, add `<plugin>` tag in `<gazebo>` with `<plugin filename="libgazebo_ros_control.so">`, the [name](ur_hand_eye_calibration/flexbe_behavior_engine/flexbe_core/flexbe_core/behavior.py#L0-L0) must be set to `gazebo_ros_control`, and controller yaml file must be included*

4. **Gazebo 与 MoveIt 联合启动** / *Gazebo and MoveIt combined launch*
   - 想正常模拟夹爪，夹爪的控制器在 `ros2_control` 必须有 mimic 的关联的 joint，且需要在 urdf 中添加 `<mimic>` 的虚拟 joint 列表避免刷屏报错 / *To properly simulate grippers, gripper controllers in `ros2_control` must have mimic associated joints, and virtual joint lists with `<mimic>` need to be added in urdf to avoid spam errors*

5. **末端执行器位姿获取** / *End-effector pose acquisition*
   - 用 `movegroup` 节点获取末端 EEF 位姿可能不如 tf 准确，因为 `movegroupinterface` 是局部机械臂的运动链 / *Acquiring end-effector pose using `movegroup` node may not be as accurate as tf, because `movegroupinterface` is a local arm kinematic chain*

6. **启动文件超时参数** / *Launch file timeout parameter*
   - 启动文件中的 spawn_entity 需要超时参数，否则会报错 / *The spawn_entity in launch files needs timeout parameter, otherwise it will throw error*

7. **IFRA_LinkAttacher-humble 依赖** / *IFRA_LinkAttacher-humble dependency*
   - 注意添加 CMakeLists.txt 对其的依赖 / *Note to add its dependency in CMakeLists.txt*

8. **Link Attacher 附着点问题** / *Link Attacher attachment point issue*
   ```
   linkattacher_msgs.srv.AttachLink_Response(
     success=True, 
     message='ATTACHED: {MODEL , LINK} -> {dual_arm , left_robotiq_85_left_finger_tip_link} -- {target_plate1564897 , link}.'
   )
   ```
   URDF 结构中，[left_robotiq_85_base_link](motion_plan/urdf/dual_arm_with_camera.xacro#L350-L350) 是通过一个 fixed 关节连接到 [left_camera_baselink](motion_plan/urdf/dual_arm_with_camera.xacro#L348-L348) 的。Gazebo 的物理引擎在构建模型时，有时会优化掉这种纯固定的、非主运动链上的链接，导致 Model::GetLink() API 无法找到它。
   
   解决方案: 将附着点从夹爪的基座 (_base_link) 改为夹爪手指的尖端 (_finger_tip_link)。这些尖端链接是直接参与物理交互（碰撞和摩擦）的关键部分，因此 Gazebo 一定会将它们注册为有效的物理实体。
   
   / *In the URDF structure, [left_robotiq_85_base_link](motion_plan/urdf/dual_arm_with_camera.xacro#L350-L350) is connected to [left_camera_baselink](motion_plan/urdf/dual_arm_with_camera.xacro#L348-L348) through a fixed joint. The physics engine in Gazebo sometimes optimizes away these purely fixed links that are not on the main kinematic chain, causing the Model::GetLink() API to fail to find them.*
   
   / *Solution: Change the attachment point from the gripper base (_base_link) to the gripper finger tip (_finger_tip_link). These tip links are key parts that directly participate in physical interactions (collision and friction), so Gazebo will definitely register them as valid physical entities.*

9. **IFRA_LinkAttacher 重复附着限制** / *IFRA_LinkAttacher repeated attachment limitation*
   - 问题：`IFRA_LinkAttacher` 的源码只能附着一次，即在已有一次附着成功后，不能接着再次使用附着，无论是再次附着其他 link 还是当前 link。/ *Issue: The `IFRA_LinkAttacher` source code can only attach once; after one successful attachment, it cannot be used again to attach, whether to another link or the current link.*
   - 解决方案：需要修改源码 [IFRA_LinkAttacher-humble/ros2_LinkAttacher/src/gazebo_link_attacher.cpp](IFRA_LinkAttacher-humble/ros2_LinkAttacher/src/gazebo_link_attacher.cpp) / *Solution: Need to modify the source code at [IFRA_LinkAttacher-humble/ros2_LinkAttacher/src/gazebo_link_attacher.cpp](IFRA_LinkAttacher-humble/ros2_LinkAttacher/src/gazebo_link_attacher.cpp)*
   - 影响：在连续操作场景中，如需要多次抓取和放置物体时会出现问题 / *Impact: Problems will occur in continuous operation scenarios, such as multiple pick-and-place operations*
   - 临时解决方法：重启仿真环境以重置附着状态 / *Temporary workaround: Restart the simulation environment to reset the attachment state*

10. **serial 包依赖关系** / *Serial package dependency*
   - 依赖：`serial` 包被依赖于 `ros2_robotiq_gripper` / *Dependency: The `serial` package is required by `ros2_robotiq_gripper`*
   - 用途：用于与 Robotiq 夹爪进行串口通信 / *Usage: Used for serial communication with Robotiq grippers*
   - 注意：在安装和编译 `ros2_robotiq_gripper` 时必须确保 `serial` 包已正确安装 / *Note: Ensure the `serial` package is correctly installed when installing and compiling `ros2_robotiq_gripper`*
   - 安装方法：`sudo apt-get install ros-humble-serial` 或从源码编译 / *Installation: `sudo apt-get install ros-humble-serial` or compile from source*

## 待办事项 / TODO

- [x] **双臂协同规划算法** / *Dual-arm cooperative planning algorithm*
- [x] **rviz和简单的gazebo仿真** / *RViz and basic Gazebo simulation*
- [ ] **视觉感知模块集成（gazebo camera）** / *Vision perception module integration (Gazebo camera)*
- [ ] **轨迹平滑优化** / *Trajectory smoothing optimization*
- [ ] **用户界面开发** / *User interface development*
- [ ] **错误处理机制完善** / *Error handling mechanism improvement*
- [ ] **性能监控工具** / *Performance monitoring tools*
- [ ] **单元测试补充** / *Unit test supplementation*
- [ ] **文档完善** / *Documentation completion*
- [ ] **部署脚本自动化** / *Deployment script automation*


## 版权声明 / Copyright Notice

© 2026 [mirrors]. 保留所有权利. / *All rights reserved by [Your Organization Name], 2026*

本项目遵循 GNU GENERAL PUBLIC LICENSE Version 3 (GPLv3) 许可证. 详见 LICENSE 文件. / *This project follows GNU GENERAL PUBLIC LICENSE Version 3 (GPLv3). See LICENSE file for details.*

### 许可证要点 / License Key Points:
- **自由使用** / *Free to use*: 可以自由运行、研究、分享和修改软件 / *You can run, study, share, and modify the software*
- **源码开放** / *Open source*: 分发修改版本时必须提供源码 / *Modified versions must include source code*
- **相同许可** / *Same license*: 衍生作品必须使用相同的 GPLv3 许可证 / *Derivative works must be licensed under the same GPLv3*
- **无担保** / *No warranty*: 软件按"原样"提供，无任何担保 / *Software is provided "as is", without warranty*

### 第三方组件 / Third-party components:
- ROS2 Humble Hawksbill
- MoveIt! Motion Planning Framework
- Gazebo Classic Simulator
- Robotiq 85 Gripper Models

### 联系方式 / Contact:
[mirrors@您的邮箱地址] / *[mirrors@mirrors.com]*

---

<!-- **重要提醒**: 由于项目采用 GPLv3 许可证，任何基于此项目进行的修改或衍生作品都必须：
1. 保持相同的 GPLv3 许可证
2. 提供源代码访问权限
3. 包含原始版权声明和许可证声明 -->

*Important reminder: Since the project uses GPLv3 license, any modifications or derivative works based on this project must:*
1. *Maintain the same GPLv3 license*
2. *Provide source code access*
3. *Include original copyright notices and license statements*