#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <cstdlib> // 引入stdlib.h以使用std::atof
#include <iostream>
#include <sstream>
#include <std_srvs/srv/trigger.hpp>
#include <vector>
#include <stdexcept>
#include <set>
#include <cmath>
#include <moveit/trajectory_processing/time_optimal_trajectory_generation.h>
#include <moveit/trajectory_processing/iterative_spline_parameterization.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("motion_plan");

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);
    auto move_group_node = rclcpp::Node::make_shared("motion_plan", node_options);

    move_group_node->set_parameter(rclcpp::Parameter("use_sim_time", true));

    rclcpp::executors::SingleThreadedExecutor executor;
    // rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(move_group_node);
    std::thread([&executor](){ executor.spin(); }).detach();

    static const std::string PLANNING_GROUP = "dual_arm";
    // static const std::string JOINT_MODEL = "left_joint_7";

    static const std::string POSE_1 = "dual_pose1";
    static const std::string POSE_2 = "dual_pose2";

    moveit::planning_interface::MoveGroupInterface move_group(move_group_node, PLANNING_GROUP);
    // moveit::planning_interface::MoveGroupInterface left_group(move_group_node, "left_arm_hand");
    // moveit::planning_interface::MoveGroupInterface right_group(move_group_node, "right_arm_hand");
    // moveit::planning_interface::MoveGroupInterface left_gripper(move_group_node, "left_hand");  
    // moveit::planning_interface::MoveGroupInterface right_gripper(move_group_node, "right_hand");
    moveit::planning_interface::MoveGroupInterface leftarm_only(move_group_node, "left_arm");  
    moveit::planning_interface::MoveGroupInterface rightarm_only(move_group_node, "right_arm");

    std::string defaultPlannerId = move_group.getDefaultPlannerId(PLANNING_GROUP);
    std::string curPlannerId = move_group.getPlannerId();
    RCLCPP_INFO(LOGGER, "defaultPlannerId====%s \ncurPlannerId====%s", defaultPlannerId.c_str(),curPlannerId.c_str());
    move_group.setPlannerId("RRTstar");
    defaultPlannerId = move_group.getDefaultPlannerId(PLANNING_GROUP);
    curPlannerId = move_group.getPlannerId();
    std::string getPlanningPipelineId = move_group.getPlanningPipelineId();
    RCLCPP_INFO(LOGGER, "defaultPlannerId====%s \ncurPlannerId====%s\ngetPlanningPipelineId====%s", defaultPlannerId.c_str(),curPlannerId.c_str(),getPlanningPipelineId.c_str());


    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    const moveit::core::JointModelGroup *joint_model_group =
        move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

    // 获取关节的物理限制
    const moveit::core::JointModel* joint_model = joint_model_group->getJointModel("left_joint_7");
    double j7min_position = joint_model->getVariableBounds()[0].min_position_;
    double j7max_position = joint_model->getVariableBounds()[0].max_position_;
    RCLCPP_INFO(LOGGER, "--------------arm_joint_7 limits: min = %f, max = %f", j7min_position, j7max_position); 

    std::vector<double> joint_arm_to_gripper_target = move_group.getCurrentJointValues();
    std::vector<std::string> jointname = move_group.getJointNames();

    RCLCPP_INFO(LOGGER, "初始关节值----------------------:");
    for (size_t i = 0; i < jointname.size(); ++i)
    {
        RCLCPP_INFO(LOGGER, "%s: %f", jointname[i].c_str(), joint_arm_to_gripper_target[i]);
    }

    RCLCPP_INFO(LOGGER, "Available Planning Groups:");
    std::copy(move_group.getJointModelGroupNames().begin(), move_group.getJointModelGroupNames().end(),
              std::ostream_iterator<std::string>(std::cout, ", "));
    std::cout << std::endl;

// RCLCPP_INFO(LOGGER, "=============================== QQ ============================");
// move_group.setStartStateToCurrentState();
// std::vector<double> joint_angles_degrees = {0, -30, 0, -30, 0, -30, 0};
//     // 将关节角度从度转换为弧度
//     std::vector<double> joint_angles_radians;
//     for (double angle : joint_angles_degrees) {
//         double radian = angle * M_PI / 180.0;
//         joint_angles_radians.push_back(radian);
//     }
// leftarm_only.setJointValueTarget(joint_angles_radians);

//     // 规划动作
//     moveit::planning_interface::MoveGroupInterface::Plan gripper_plan1;
//     bool jsuccess1 = (leftarm_only.plan(gripper_plan1) == moveit::core::MoveItErrorCode::SUCCESS);

//     // 输出结果
//     RCLCPP_INFO(LOGGER, "==[NOTE]== Plan (gripper goal) %s", jsuccess1 ? "SUCCESS" : "FAILED");
//     if (!jsuccess1) {
//         RCLCPP_ERROR(LOGGER, "==[FAILED!!]== 创建规划plan失败，原因可能是\n1.指定的目标位姿不可达，超出了机械臂的移动范围，无法求出逆解。\n2.缝隙太小，无法有效采样。\n3.机器人配置空间中的障碍物设置或环境不适合进行有效的采样。\n");
//     } else{
//         RCLCPP_INFO(LOGGER, "==[DONE!!]== 创建规划plan成功,3秒后执行");
//         // rclcpp::sleep_for(std::chrono::seconds(3));
//         leftarm_only.execute(gripper_plan1);
//       RCLCPP_INFO(LOGGER, "< move_group.execute(gripper_plan1) work well... >");
//         leftarm_only.move();
//     }
// rclcpp::sleep_for(std::chrono::seconds(2));

// RCLCPP_INFO(LOGGER, "=============================== POSE MOVE TO OBJECT POSITION ============================");

//         move_group.setPlanningPipelineId("ompl_plan");
//         move_group.setPlannerId("RRTstar");
//         move_group.setStartStateToCurrentState();
//     // 定义目标姿态目标
//     geometry_msgs::msg::Pose target_pose; // 假设此姿态在代码中已被定义
//     // 定义起始姿态
//     geometry_msgs::msg::Pose start_pose = leftarm_only.getCurrentPose("left_hand_link8").pose; // 假设此姿态在代码中已被定义
// RCLCPP_INFO(LOGGER, "=============================== pose goal设置限制和姿态约束 ============================");

// /*
// 起始点0.49779; 0.50332; 0.49587; 0.50299
// 目标 0.14468; 0.69301; 0.14317; 0.6916
// */
//     target_pose.position.x = 0.5341;
//     target_pose.position.y = -0.0027201;
//     target_pose.position.z = 0.90373;
//     target_pose.orientation.x = 0.85326;
//     target_pose.orientation.y = 0.52008;
//     target_pose.orientation.z = -0.026163;
//     target_pose.orientation.w = -0.027884;
//     // target_pose.orientation = start_pose.orientation; // 这里需要定义start_pose    
    
//     // 0.5341; -0.0027201; 0.90373

//     // 0.85326; 0.52008; -0.026163; -0.027884
//     // 定义方向约束
//     moveit_msgs::msg::OrientationConstraint ocm;
//     ocm.link_name = "left_hand_link8";
//     ocm.header.frame_id = "base_link";

//     // 更新方向约束
//     ocm.orientation.w = 1.0;
//     ocm.absolute_x_axis_tolerance = 0.5;
//     ocm.absolute_y_axis_tolerance = 0.5;
//     ocm.absolute_z_axis_tolerance = 0.5;
//     ocm.weight = 1.0;

//     // 将方向约束添加到约束集
//     moveit_msgs::msg::Constraints test_constraints;
//     test_constraints.orientation_constraints.push_back(ocm);
//     leftarm_only.setPathConstraints(test_constraints);

//     // 设置目标姿态
//     leftarm_only.setPoseTarget(target_pose);

//     // 设置规划参数
//     move_group.setNumPlanningAttempts(3);
//     move_group.setReplanAttempts(5);
//     move_group.setMaxVelocityScalingFactor(0.3);
//     move_group.setMaxAccelerationScalingFactor(0.3);

//     // 规划动作
//     moveit::planning_interface::MoveGroupInterface::Plan my_plan;
//     bool success = (leftarm_only.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);

//     if (success) {
//         // 执行动作
//         move_group.execute(my_plan);
//     } else {
//         RCLCPP_ERROR(LOGGER, "Planning failed!");
//     }
    
//     move_group.clearPathConstraints();


//     rclcpp::shutdown();
//     return 0;
RCLCPP_INFO(LOGGER, "=============================== MOVE TO OBJECT POSITION ============================");

    // 设置起始状态
    move_group.setStartStateToCurrentState();

    // 设定关节目标
    move_group.setNamedTarget(POSE_1);
    // 设置规划尝试次数
    move_group.setNumPlanningAttempts(3);
    // 设置最大重新规划尝试次数（如遇到障碍物时最多尝试5次）
    move_group.setReplanAttempts(5);
    move_group.setMaxVelocityScalingFactor(0.2); // Cartesian_Path not support
    move_group.setMaxAccelerationScalingFactor(0.2);

    // 规划动作
    moveit::planning_interface::MoveGroupInterface::Plan my_plan1;
    bool success1 = (move_group.plan(my_plan1) == moveit::core::MoveItErrorCode::SUCCESS);

    // 输出结果
    RCLCPP_INFO(LOGGER, "==[NOTE]== Plan (joint goal1) %s", success1 ? "SUCCESS" : "FAILED");
    if (!success1) {
        RCLCPP_ERROR(LOGGER, "==[FAILED!!]== 创建规划plan失败，原因可能是\n1.指定的目标位姿不可达，超出了机械臂的移动范围，无法求出逆解。\n2.缝隙太小，无法有效采样。\n3.机器人配置空间中的障碍物设置或环境不适合进行有效的采样。\n");
        executor.cancel();
        return 0;
    } else{
        RCLCPP_INFO(LOGGER, "==[DONE!!]== 创建规划plan成功,3秒后执行");
        move_group.execute(my_plan1);
        RCLCPP_INFO(LOGGER, "<my plan1 move_group.execute(my_plan1) work well>");
        move_group.move();        
    }

    rclcpp::sleep_for(std::chrono::seconds(1));


RCLCPP_INFO(LOGGER, "=============================== ROTATE GRIPPER ============================");

    // moveit::core::RobotStatePtr current_state_arm =
    //     move_group.getCurrentState(10);

    // std::vector<double> joint_arm_to_gripper_target;
    // current_state_arm->copyJointGroupPositions(joint_model_group_arm,
    //                                             joint_arm_to_gripper_target);
    move_group.clearPoseTargets();
    move_group.setStartStateToCurrentState();
    // 获取当前关节值和关节名称
    jointname = move_group.getJointNames();
    joint_arm_to_gripper_target = move_group.getCurrentJointValues();

    // 计算夹爪旋转角度
    float target_theta_left = -90* M_PI / 180;
    float target_theta_right = -60* M_PI / 180;

    // 打印修改前的关节值
RCLCPP_INFO(LOGGER, "修改前的关节值:");
for (size_t i = 0; i < jointname.size(); ++i) {
    RCLCPP_INFO(LOGGER, "%s: %f", jointname[i].c_str(), joint_arm_to_gripper_target[i]);
}
    
    // 设置夹爪的目标姿态
    int cnt = 0;
    for (auto& elem : jointname) {
        if (elem == "left_joint_7"){
           // 旋转夹爪 
           if ((joint_arm_to_gripper_target[cnt] + target_theta_left) >= j7max_position){
            joint_arm_to_gripper_target[cnt] -= target_theta_left;
           }           
           else
           {
            joint_arm_to_gripper_target[cnt] += target_theta_left;
           }
        }

        if (elem == "right_joint_7"){
           // 旋转夹爪 
           if ((joint_arm_to_gripper_target[cnt] + target_theta_right) >= j7max_position){
            joint_arm_to_gripper_target[cnt] -= target_theta_right;
           }           
           else
           {
            joint_arm_to_gripper_target[cnt] += target_theta_right;
           }
           break;
        } 
        cnt++;
    }

// 打印修改后的关节值
RCLCPP_INFO(LOGGER, "修改后的关节值:");
for (size_t i = 0; i < jointname.size(); ++i) {
    RCLCPP_INFO(LOGGER, "%s: %f", jointname[i].c_str(), joint_arm_to_gripper_target[i]);
}

// 设置关节约束
// moveit_msgs::msg::Constraints constraints;
// moveit_msgs::msg::JointConstraint joint_constraint;

// joint_constraint.joint_name = "left_joint_7";
// joint_constraint.position = joint_arm_to_gripper_target[cnt]; // 当前的关节值
// joint_constraint.tolerance_above = 1; // 上限容差
// joint_constraint.tolerance_below = 1; // 下限容差
// joint_constraint.weight = 1.0; // 权重

// constraints.joint_constraints.push_back(joint_constraint);
// move_group.setPathConstraints(constraints);

    // 设定末端旋转夹爪的目标
    move_group.setJointValueTarget(joint_arm_to_gripper_target);

    // 规划动作
    moveit::planning_interface::MoveGroupInterface::Plan gripper_plan1;
    bool jsuccess1 = (move_group.plan(gripper_plan1) == moveit::core::MoveItErrorCode::SUCCESS);

    // 输出结果
    RCLCPP_INFO(LOGGER, "==[NOTE]== Plan (gripper goal) %s", jsuccess1 ? "SUCCESS" : "FAILED");
    if (!jsuccess1) {
        RCLCPP_ERROR(LOGGER, "==[FAILED!!]== 创建规划plan失败，原因可能是\n1.指定的目标位姿不可达，超出了机械臂的移动范围，无法求出逆解。\n2.缝隙太小，无法有效采样。\n3.机器人配置空间中的障碍物设置或环境不适合进行有效的采样。\n");
        executor.cancel();
        return 0;
    } else{
        RCLCPP_INFO(LOGGER, "==[DONE!!]== 创建规划plan成功,3秒后执行");
        // rclcpp::sleep_for(std::chrono::seconds(3));
        move_group.execute(gripper_plan1);
      RCLCPP_INFO(LOGGER, "< move_group.execute(gripper_plan1) work well... >");
        move_group.move();

    }

    rclcpp::sleep_for(std::chrono::seconds(3));


RCLCPP_INFO(LOGGER, "=============================== OPEN GRIPPER ============================");

    rclcpp::sleep_for(std::chrono::seconds(1));



RCLCPP_INFO(LOGGER, "=============================== CONNECT GO DOWN Cartesian ============================");

  move_group.setStartState(*move_group.getCurrentState());

geometry_msgs::msg::PoseStamped rightEEF_current_pose_down = rightarm_only.getCurrentPose("right_hand_link8");
geometry_msgs::msg::Pose target_right_EEFpose_down = rightEEF_current_pose_down.pose;
// 打印当前姿态的位置和姿态信息
    RCLCPP_INFO(LOGGER, "Current Right EEF Pose: ");
    RCLCPP_INFO(LOGGER, "Position: (x: %.4f, y: %.4f, z: %.4f)",
                target_right_EEFpose_down.position.x,
                target_right_EEFpose_down.position.y,
                target_right_EEFpose_down.position.z);
    RCLCPP_INFO(LOGGER, "Orientation: (x: %.4f, y: %.4f, z: %.4f, w: %.4f)",
                target_right_EEFpose_down.orientation.x,
                target_right_EEFpose_down.orientation.y,
                target_right_EEFpose_down.orientation.z,
                target_right_EEFpose_down.orientation.w);

// RCLCPP_INFO(LOGGER, "=============================== QQ ============================");

//     geometry_msgs::msg::Pose target_pose; // 假设此姿态在代码中已被定义
//     // 定义起始姿态
//     geometry_msgs::msg::Pose start_pose = leftarm_only.getCurrentPose("left_hand_link8").pose; // 假设此姿态在代码中已被定义
//     // 设置规划参数
//     move_group.setNumPlanningAttempts(3);
//     move_group.setReplanAttempts(5);
//     move_group.setMaxVelocityScalingFactor(0.5);
//     move_group.setMaxAccelerationScalingFactor(0.5);
// /*
// 起始点0.49779; 0.50332; 0.49587; 0.50299
// 目标 0.14468; 0.69301; 0.14317; 0.6916
// */
//     target_pose.position.x = 0.61523;
//     target_pose.position.y = -0.18777;
//     target_pose.position.z = 1.2839;
//     // target_pose.orientation.x = 0.70949;
//     // target_pose.orientation.y = 0.54082;
//     // target_pose.orientation.z = 0.22631;
//     // target_pose.orientation.w = 0.39105;

//     leftarm_only.setGoalOrientationTolerance(0.2);
//     leftarm_only.setPoseTarget(target_pose);



//     // 规划动作
//     moveit::planning_interface::MoveGroupInterface::Plan my_plan;
//     bool success = (leftarm_only.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);

//     if (success) {
//         // 执行动作
//         move_group.execute(my_plan);
//     } else {
//         RCLCPP_ERROR(LOGGER, "Planning failed!");
//     }
    
//     move_group.clearPathConstraints();


//     rclcpp::shutdown();
//     return 0;
// RCLCPP_INFO(LOGGER, "=============================== QQ ============================");



geometry_msgs::msg::Pose start_pose = rightarm_only.getCurrentPose("right_hand_link8").pose;
//   rclcpp::Time start = rclcpp::Clock().now();


    RCLCPP_INFO(LOGGER, "Current Right EEF Pose: ");
    RCLCPP_INFO(LOGGER, "Position: (x: %.4f, y: %.4f, z: %.4f)",
                start_pose.position.x,
                start_pose.position.y,
                start_pose.position.z);
    RCLCPP_INFO(LOGGER, "Orientation: (x: %.4f, y: %.4f, z: %.4f, w: %.4f)",
                start_pose.orientation.x,
                start_pose.orientation.y,
                start_pose.orientation.z,
                start_pose.orientation.w);

  std::vector<geometry_msgs::msg::Pose> waypoints_down;

//   target_right_EEFpose_down.position.z -= 0.1; // Go down
//   target_right_EEFpose_down.position.y -= 0.001; // Move up
//   target_right_EEFpose_down.position.x -= 0.12; // Move left
  start_pose.position.z -= 0.1; // Move left
  waypoints_down.push_back(start_pose); 


  for (std::size_t i = 0; i < waypoints_down.size(); ++i) {
      const auto& wp = waypoints_down[i]; // 获取 waypoint
      RCLCPP_INFO(LOGGER, "Waypoint %zu: [x: %.4f, y: %.4f, z: %.4f]",
                  i,
                  wp.position.x,
                  wp.position.y,
                  wp.position.z);
  }

  moveit_msgs::msg::RobotTrajectory trajectory_down;
  double jump_threshold = 0.0;
  double eef_step = 0.001;

  double fraction = rightarm_only.computeCartesianPath(
      waypoints_down, eef_step, jump_threshold, trajectory_down ,false);
  RCLCPP_INFO(LOGGER, "################ Cartesian path (%.2f%% achieved)", fraction * 100.0);


  if (fraction >= 0.8)
  {
    RCLCPP_INFO(LOGGER, "Achieved %f %% of Cartesian path", fraction * 100.);

    robot_trajectory::RobotTrajectory rt(rightarm_only.getRobotModel(), rightarm_only.getName());
    rt.setRobotTrajectoryMsg(*rightarm_only.getCurrentState(), trajectory_down);
        trajectory_processing::IterativeParabolicTimeParameterization iptp;
        // 下面这句限制速度0.1,加速度0.01
        iptp.computeTimeStamps(rt, 0.1, 0.01);
        rt.getRobotTrajectoryMsg(trajectory_down);    
    // trajectory_processing::TimeOptimalTrajectoryGeneration time_parameterization;
    // bool success = time_parameterization.computeTimeStamps(rt, 0.01,
    //                                                        0.01);
    // RCLCPP_INFO(LOGGER, "Computing time stamps %s", success ? "SUCCEEDED" : "FAILED");

    // // Store trajectory in current_plan_
    // auto current_plan_ = std::make_shared<moveit::planning_interface::MoveGroupInterface::Plan>();
    // rt.getRobotTrajectoryMsg(current_plan_->trajectory_);
    // current_plan_->planning_time_ = (rclcpp::Clock().now() - start).seconds();
    move_group.execute(trajectory_down);
    }

    else {
        executor.cancel();
        return 0;
    }
    // if (fraction > 0.80) move_group.execute(trajectory_down);
    // else {
    //     executor.cancel();
    //     return 0;
    // }
    rclcpp::sleep_for(std::chrono::seconds(2));


RCLCPP_INFO(LOGGER, "=============================== CLOSE GRIPPER ============================");

    rclcpp::sleep_for(std::chrono::seconds(1));


// RCLCPP_INFO(LOGGER, "*********************获取并打印所有可用的命名规划目标名称===================");
//     // 获取所有可用的命名目标名称
//     const std::vector<std::string>& named_targets = move_group.getNamedTargets();

//     // 打印所有命名目标的名称
//     RCLCPP_INFO(LOGGER, "Available Named Targets:");
//     for (const auto& target : named_targets) {
//         RCLCPP_INFO(LOGGER, "- %s", target.c_str());
//     }

//     // 遍历每个命名目标并打印其关节角度值
//     for (const auto& target : named_targets) {
//         // 获取指定命名目标的关节角度值
//         std::map<std::string, double> joint_values = move_group.getNamedTargetValues(target);

//         // 打印该命名目标的名称
//         RCLCPP_INFO(LOGGER, "\nJoint values for target '%s':", target.c_str());

//         // 打印每个关节的角度值
//         for (const auto& joint_value : joint_values) {
//             RCLCPP_INFO(LOGGER, "%s: %.4f", joint_value.first.c_str(), joint_value.second);
//         }
//     }
// RCLCPP_INFO(LOGGER, "*********************打印命名规划目标名称 完毕... ===================");

    

RCLCPP_INFO(LOGGER, "=============================== CONNECT GO UP Cartesian ============================");

  move_group.setStartState(*move_group.getCurrentState());

geometry_msgs::msg::PoseStamped rightEEF_current_pose_up = rightarm_only.getCurrentPose("right_hand_link8");
geometry_msgs::msg::Pose target_right_EEFpose_up = rightEEF_current_pose_up.pose;
// 打印当前姿态的位置和姿态信息
    RCLCPP_INFO(LOGGER, "Current Right EEF Pose: ");
    RCLCPP_INFO(LOGGER, "Position: (x: %.4f, y: %.4f, z: %.4f)",
                target_right_EEFpose_up.position.x,
                target_right_EEFpose_up.position.y,
                target_right_EEFpose_up.position.z);
    RCLCPP_INFO(LOGGER, "Orientation: (x: %.4f, y: %.4f, z: %.4f, w: %.4f)",
                target_right_EEFpose_up.orientation.x,
                target_right_EEFpose_up.orientation.y,
                target_right_EEFpose_up.orientation.z,
                target_right_EEFpose_up.orientation.w);


  std::vector<geometry_msgs::msg::Pose> waypoints_up;

  target_right_EEFpose_up.position.z += 0.1; // Go down

  waypoints_up.push_back(target_right_EEFpose_up); 


  for (std::size_t i = 0; i < waypoints_up.size(); ++i) {
      const auto& wp = waypoints_up[i]; // 获取 waypoint
      RCLCPP_INFO(LOGGER, "Waypoint %zu: [x: %.4f, y: %.4f, z: %.4f]",
                  i,
                  wp.position.x,
                  wp.position.y,
                  wp.position.z);
  }

  moveit_msgs::msg::RobotTrajectory trajectory_up;
  jump_threshold = 0.0;
  eef_step = 0.001;

  fraction = rightarm_only.computeCartesianPath(
      waypoints_up, eef_step, jump_threshold, trajectory_up ,false);
  RCLCPP_INFO(LOGGER, "################ Cartesian path (%.2f%% achieved)", fraction * 100.0);
    if (fraction > 0.8) move_group.execute(trajectory_up);
    else {
        executor.cancel();
        return 0;
    }
    rclcpp::sleep_for(std::chrono::seconds(2));



RCLCPP_INFO(LOGGER, "=============================== MOVE TO WORK POSE ============================");

    // move_group.setStartState(*move_group.getCurrentState());
    move_group.clearPoseTargets();
    move_group.setStartStateToCurrentState();

    // 设定关节目标
    move_group.setNamedTarget(POSE_2);

    // 规划动作
    moveit::planning_interface::MoveGroupInterface::Plan my_plan2;
    bool success2 = (move_group.plan(my_plan2) == moveit::core::MoveItErrorCode::SUCCESS);

    // 输出结果
    RCLCPP_INFO(LOGGER, "==[NOTE]== Plan (joint goal2) %s", success2 ? "SUCCESS" : "FAILED");
    if (!success2) {
        RCLCPP_ERROR(LOGGER, "==[FAILED!!]== 创建规划plan失败，原因可能是\n1.指定的目标位姿不可达，超出了机械臂的移动范围，无法求出逆解。\n2.缝隙太小，无法有效采样。\n3.机器人配置空间中的障碍物设置或环境不适合进行有效的采样。\n");
        executor.cancel();
        return 0;
    } else{
        RCLCPP_INFO(LOGGER, "==[DONE!!]== 创建规划plan成功,3秒后执行");
        move_group.execute(my_plan2);
        RCLCPP_INFO(LOGGER, "<my plan1 move_group.execute(my_plan2) work well>");
        move_group.move();        
    }
    rclcpp::sleep_for(std::chrono::seconds(1));


RCLCPP_INFO(LOGGER, "=============================== ROTATE GRIPPER ============================");

    move_group.clearPoseTargets();
    move_group.setStartStateToCurrentState();

    // 获取当前关节值和关节名称
    std::vector<std::string> jointname2 = move_group.getJointNames();
    joint_arm_to_gripper_target = move_group.getCurrentJointValues();

    // 计算夹爪旋转角度
    float target_theta2 = 3.75* M_PI / 180;

    // 打印修改前的关节值
RCLCPP_INFO(LOGGER, "修改前的关节值:");
for (size_t i = 0; i < jointname2.size(); ++i) {
    RCLCPP_INFO(LOGGER, "%s: %f", jointname[i].c_str(), joint_arm_to_gripper_target[i]);
}
    
    // 设置夹爪的目标姿态
    int cnt1 = 0;
    for (auto& elem : jointname2) {
        if (elem == "left_joint_7"){
           // 旋转夹爪 
           if ((joint_arm_to_gripper_target[cnt1] + target_theta2) >= j7max_position){
            joint_arm_to_gripper_target[cnt1] -= target_theta2;
           }           
           else
           {
            joint_arm_to_gripper_target[cnt1] += target_theta2;
           }
           break;
        } 
        cnt1++;
    }

// 打印修改后的关节值
RCLCPP_INFO(LOGGER, "修改后的关节值:");
for (size_t i = 0; i < jointname2.size(); ++i) {
    RCLCPP_INFO(LOGGER, "%s: %f", jointname[i].c_str(), joint_arm_to_gripper_target[i]);
}

    // 设定末端旋转夹爪的目标
    move_group.setJointValueTarget(joint_arm_to_gripper_target);

    // 规划动作
    moveit::planning_interface::MoveGroupInterface::Plan joint_plan2;
    jsuccess1 = (move_group.plan(joint_plan2) == moveit::core::MoveItErrorCode::SUCCESS);

    // 输出结果
    RCLCPP_INFO(LOGGER, "==[NOTE]== Plan (gripper goal) %s", jsuccess1 ? "SUCCESS" : "FAILED");
    if (!jsuccess1) {
        RCLCPP_ERROR(LOGGER, "==[FAILED!!]== 创建规划plan失败，原因可能是\n1.指定的目标位姿不可达，超出了机械臂的移动范围，无法求出逆解。\n2.缝隙太小，无法有效采样。\n3.机器人配置空间中的障碍物设置或环境不适合进行有效的采样。\n");
        executor.cancel();
        return 0;
    } else{
        RCLCPP_INFO(LOGGER, "==[DONE!!]== 创建规划plan成功,3秒后执行");
        // rclcpp::sleep_for(std::chrono::seconds(3));
        move_group.execute(joint_plan2);
        move_group.move();
      RCLCPP_INFO(LOGGER, "< move_group.execute(joint_plan2) work well... >");

    }

    rclcpp::sleep_for(std::chrono::seconds(1));


RCLCPP_INFO(LOGGER, "=============================== CONNECT LINE Cartesian ============================");

  move_group.setStartState(*move_group.getCurrentState());

    defaultPlannerId = move_group.getDefaultPlannerId(PLANNING_GROUP);

    move_group.setPlanningPipelineId("pilz_industrial_motion_planner");
    move_group.setPlannerId("LIN");
    getPlanningPipelineId = move_group.getPlanningPipelineId();
    curPlannerId = move_group.getPlannerId();
    RCLCPP_INFO(LOGGER, "defaultPlannerId====%s \ncurPlannerId====%s\ngetPlanningPipelineId====%s", defaultPlannerId.c_str(),curPlannerId.c_str(),getPlanningPipelineId.c_str());
    // 获取规划器参数
    std::map<std::string, std::string> planner_params = move_group.getPlannerParams(curPlannerId);
    // 打印规划器参数
    RCLCPP_INFO(LOGGER, "Planner Parameters:");
    for (const auto& param : planner_params) {
        RCLCPP_INFO(LOGGER, "  %s: %s", param.first.c_str(), param.second.c_str());
    }


geometry_msgs::msg::PoseStamped leftEEF_current_pose = leftarm_only.getCurrentPose("left_hand_link8");
geometry_msgs::msg::Pose target_left_EEFpose = leftEEF_current_pose.pose;
// 打印当前姿态的位置和姿态信息
    RCLCPP_INFO(LOGGER, "Current Right EEF Pose: ");
    RCLCPP_INFO(LOGGER, "Position: (x: %.4f, y: %.4f, z: %.4f)",
                target_left_EEFpose.position.x,
                target_left_EEFpose.position.y,
                target_left_EEFpose.position.z);
    RCLCPP_INFO(LOGGER, "Orientation: (x: %.4f, y: %.4f, z: %.4f, w: %.4f)",
                target_left_EEFpose.orientation.x,
                target_left_EEFpose.orientation.y,
                target_left_EEFpose.orientation.z,
                target_left_EEFpose.orientation.w);


  std::vector<geometry_msgs::msg::Pose> waypoints;

//   target_left_EEFpose.position.z += 0.03; // Go down
//   target_left_EEFpose.position.y -= 0.001; // Move up
  target_left_EEFpose.position.x -= 0.071; // Move left
  waypoints.push_back(target_left_EEFpose); 


  for (std::size_t i = 0; i < waypoints.size(); ++i) {
      const auto& wp = waypoints[i]; // 获取 waypoint
      RCLCPP_INFO(LOGGER, "Waypoint %zu: [x: %.4f, y: %.4f, z: %.4f]",
                  i,
                  wp.position.x,
                  wp.position.y,
                  wp.position.z);
  }

  moveit_msgs::msg::RobotTrajectory trajectory;
  jump_threshold = 0.0;
  eef_step = 0.001;

  fraction = leftarm_only.computeCartesianPath(
      waypoints, eef_step, jump_threshold, trajectory ,false);
  RCLCPP_INFO(LOGGER, "################ Cartesian path (%.2f%% achieved)", fraction * 100.0);
    if (fraction > 0.95) move_group.execute(trajectory);
    else {
        executor.cancel();
        return 0;
    }
rclcpp::sleep_for(std::chrono::seconds(2));

RCLCPP_INFO(LOGGER, "=============================== CONNECT OUT Cartesian ============================");

  move_group.setStartState(*move_group.getCurrentState());

geometry_msgs::msg::PoseStamped leftEEF_current_pose_out = leftarm_only.getCurrentPose("left_hand_link8");
geometry_msgs::msg::Pose target_left_EEFpose_out = leftEEF_current_pose_out.pose;
// 打印当前姿态的位置和姿态信息
    RCLCPP_INFO(LOGGER, "Current Right EEF Pose: ");
    RCLCPP_INFO(LOGGER, "Position: (x: %.4f, y: %.4f, z: %.4f)",
                target_left_EEFpose_out.position.x,
                target_left_EEFpose_out.position.y,
                target_left_EEFpose_out.position.z);
    RCLCPP_INFO(LOGGER, "Orientation: (x: %.4f, y: %.4f, z: %.4f, w: %.4f)",
                target_left_EEFpose_out.orientation.x,
                target_left_EEFpose_out.orientation.y,
                target_left_EEFpose_out.orientation.z,
                target_left_EEFpose_out.orientation.w);


  std::vector<geometry_msgs::msg::Pose> waypoints1;

//   target_left_EEFpose_out.position.y -= 0.001; // Move up
  target_left_EEFpose_out.position.x += 0.08; // Move left
  waypoints1.push_back(target_left_EEFpose_out); 


  for (std::size_t i = 0; i < waypoints1.size(); ++i) {
      const auto& wp = waypoints1[i]; // 获取 waypoint
      RCLCPP_INFO(LOGGER, "Waypoint %zu: [x: %.4f, y: %.4f, z: %.4f]",
                  i,
                  wp.position.x,
                  wp.position.y,
                  wp.position.z);
  }

  moveit_msgs::msg::RobotTrajectory trajectory_out;
  jump_threshold = 0.0;
  eef_step = 0.001;

  fraction = leftarm_only.computeCartesianPath(
      waypoints1, eef_step, jump_threshold, trajectory_out ,false);
  RCLCPP_INFO(LOGGER, "################ Cartesian path (%.2f%% achieved)", fraction * 100.0);
    if (fraction > 0.95) move_group.execute(trajectory_out);
    else {
        executor.cancel();
        return 0;
    }
rclcpp::sleep_for(std::chrono::seconds(2));

RCLCPP_INFO(LOGGER, "=============================== MOVE TO HOME POSE ============================");

    move_group.setPlanningPipelineId("ompl_plan");
    move_group.setPlannerId("RRTConnect");
    move_group.clearPoseTargets();
    move_group.setStartStateToCurrentState();

    // 设定关节目标
    move_group.setNamedTarget("home");

    // 规划动作
    moveit::planning_interface::MoveGroupInterface::Plan my_plan_home;
    bool success3 = (move_group.plan(my_plan_home) == moveit::core::MoveItErrorCode::SUCCESS);

    // 输出结果
    RCLCPP_INFO(LOGGER, "==[NOTE]== Plan (joint goal2) %s", success3 ? "SUCCESS" : "FAILED");
    if (!success3) {
        RCLCPP_ERROR(LOGGER, "==[FAILED!!]== 创建规划plan失败，原因可能是\n1.指定的目标位姿不可达，超出了机械臂的移动范围，无法求出逆解。\n2.缝隙太小，无法有效采样。\n3.机器人配置空间中的障碍物设置或环境不适合进行有效的采样。\n");
        executor.cancel();
        return 0;
    } else{
        RCLCPP_INFO(LOGGER, "==[DONE!!]== 创建规划plan成功,3秒后执行");
        move_group.execute(my_plan_home);
        RCLCPP_INFO(LOGGER, "<my plan1 move_group.execute(my_plan2) work well>");
        move_group.move();        
    }
    rclcpp::sleep_for(std::chrono::seconds(1));


RCLCPP_INFO(LOGGER, "=============================== TASK DONE!!! ============================");

    rclcpp::shutdown();
    return 0;
}