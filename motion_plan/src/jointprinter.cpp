#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <cstdlib>
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

#include <rclcpp_action/rclcpp_action.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>

// 全局日志器
static const rclcpp::Logger LOGGER = rclcpp::get_logger("dual_move");
using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;

// 定义 PI，用于角度转弧度
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/**
 * @brief 根据机械臂 ID 和 SRDF 中预定义的姿态名称执行运动规划并发送轨迹
 */
int to_srdf_NamedTarget(const std::string &id, const std::string &srdf_pose_id, double speed)
{
    auto node = rclcpp::Node::make_shared("move_group_client_node_" + id + "_named");
    
    RCLCPP_INFO(node->get_logger(), "[to_srdf_NamedTarget] 开始为机械臂 %s 规划目标姿态：%s", id.c_str(), srdf_pose_id.c_str());

    std::string group_name = id; 
    auto move_group = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node, group_name);

    move_group->setMaxVelocityScalingFactor(speed);
    move_group->setMaxAccelerationScalingFactor(speed);

    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    move_group->setStartStateToCurrentState();
    move_group->setNamedTarget(srdf_pose_id);

    const size_t moveit_max_iter = 5;
    moveit::core::MoveItErrorCode plan_ret;

    for (size_t test_iter = 0; test_iter < moveit_max_iter; ++test_iter)
    {
        RCLCPP_DEBUG(node->get_logger(), "第 %zu 次尝试进行路径规划...", test_iter + 1);
        plan_ret = move_group->plan(my_plan);
        if (plan_ret == moveit::core::MoveItErrorCode::SUCCESS)
        {
            RCLCPP_INFO(node->get_logger(), "路径规划成功！");
            break;
        }
        rclcpp::sleep_for(std::chrono::milliseconds(100));
    }

    if (plan_ret != moveit::core::MoveItErrorCode::SUCCESS)
    {
        RCLCPP_ERROR(node->get_logger(), "规划失败，在 %zu 次尝试后仍无法找到有效路径！", moveit_max_iter);
        return -1;
    }

    // 构建控制器动作服务器名称
    std::string controller_name;
    if (id.find("left") != std::string::npos) {
        controller_name = "/left_controller/follow_joint_trajectory";
    } else if (id.find("right") != std::string::npos) {
        controller_name = "/right_controller/follow_joint_trajectory";
    } else {
        RCLCPP_ERROR(node->get_logger(), "无效的机械臂 ID: %s", id.c_str());
        return -1;
    }

    auto action_client = rclcpp_action::create_client<FollowJointTrajectory>(node, controller_name);

    if (!action_client->wait_for_action_server(std::chrono::seconds(10))) {
        RCLCPP_ERROR(node->get_logger(), "动作服务器 %s 未响应！", controller_name.c_str());
        return -1;
    }

    RCLCPP_INFO(node->get_logger(), "准备向控制器 %s 发送轨迹...", controller_name.c_str());

    FollowJointTrajectory::Goal goal_msg;
    goal_msg.trajectory = my_plan.trajectory_.joint_trajectory;

    auto send_goal_options = rclcpp_action::Client<FollowJointTrajectory>::SendGoalOptions();
    send_goal_options.result_callback =
        [node](const rclcpp_action::ClientGoalHandle<FollowJointTrajectory>::WrappedResult & result) {
            switch (result.code) {
                case rclcpp_action::ResultCode::SUCCEEDED:
                    RCLCPP_INFO(node->get_logger(), "轨迹执行成功！");
                    break;
                case rclcpp_action::ResultCode::ABORTED:
                    RCLCPP_ERROR(node->get_logger(), "轨迹执行被中止！");
                    break;
                case rclcpp_action::ResultCode::CANCELED:
                    RCLCPP_ERROR(node->get_logger(), "轨迹执行被取消！");
                    break;
                default:
                    RCLCPP_ERROR(node->get_logger(), "未知的结果码！");
                    break;
            }
        };

    auto future_goal_handle = action_client->async_send_goal(goal_msg, send_goal_options);
    (void)future_goal_handle; // 避免未使用警告

    RCLCPP_INFO(node->get_logger(), "轨迹已发送至控制器，正在执行...");
    return 0;
}

/**
 * @brief 根据机械臂 ID 和指定的关节角度数组 (单位：度) 执行运动规划并发送轨迹
 *
 * @param id            机械臂标识符（如 "left", "right"）
 * @param joint_angles_deg 关节角度数组 (单位：度)，顺序需与活动关节顺序一致
 * @param speed         运动速度比例因子（0.0 ~ 1.0）
 * @return int          成功返回 0，失败返回 -1
 */
int to_joint_config(const std::string &id, const std::vector<double> &joint_angles_deg, double speed)
{
    auto node = rclcpp::Node::make_shared("move_group_client_node_" + id + "_joint");

    RCLCPP_INFO(node->get_logger(), "[to_joint_config] 开始为机械臂 %s 规划关节角度目标", id.c_str());

    std::string group_name = id;
    auto move_group = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node, group_name);

    // 获取 RobotModel 和 JointModelGroup 以验证关节数量
    auto robot_model = move_group->getRobotModel();
    if (!robot_model) {
        RCLCPP_ERROR(node->get_logger(), "无法获取 RobotModel！");
        return -1;
    }

    auto joint_model_group = robot_model->getJointModelGroup(group_name);
    if (!joint_model_group) {
        RCLCPP_ERROR(node->get_logger(), "找不到规划组: %s", group_name.c_str());
        return -1;
    }

    const std::vector<std::string>& active_joints = joint_model_group->getActiveJointModelNames();

    if (joint_angles_deg.size() != active_joints.size()) {
        RCLCPP_ERROR(node->get_logger(), "错误：提供的角度数量 (%zu) 与机械臂 %s 的活动关节数量 (%zu) 不匹配！",
                     joint_angles_deg.size(), id.c_str(), active_joints.size());
        RCLCPP_INFO(node->get_logger(), "期望的关节顺序为：");
        for (const auto& name : active_joints) {
            RCLCPP_INFO(node->get_logger(), "  - %s", name.c_str());
        }
        return -1;
    }

    // 设置速度限制
    move_group->setMaxVelocityScalingFactor(speed);
    move_group->setMaxAccelerationScalingFactor(speed);

    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    move_group->setStartStateToCurrentState();

    // 将角度转换为弧度
    std::vector<double> joint_angles_rad;
    joint_angles_rad.reserve(joint_angles_deg.size());
    for (double deg : joint_angles_deg) {
        joint_angles_rad.push_back(deg * M_PI / 180.0);
    }

    // 设置目标关节值
    bool set_target_success = move_group->setJointValueTarget(joint_angles_rad);
    if (!set_target_success) {
        RCLCPP_ERROR(node->get_logger(), "设置关节目标值失败，可能是数值超出限位或无效。");
        return -1;
    }

    const size_t moveit_max_iter = 5;
    moveit::core::MoveItErrorCode plan_ret;

    for (size_t test_iter = 0; test_iter < moveit_max_iter; ++test_iter)
    {
        RCLCPP_DEBUG(node->get_logger(), "第 %zu 次尝试进行路径规划...", test_iter + 1);
        plan_ret = move_group->plan(my_plan);
        if (plan_ret == moveit::core::MoveItErrorCode::SUCCESS)
        {
            RCLCPP_INFO(node->get_logger(), "路径规划成功！");
            break;
        }
        rclcpp::sleep_for(std::chrono::milliseconds(100));
    }

    if (plan_ret != moveit::core::MoveItErrorCode::SUCCESS)
    {
        RCLCPP_ERROR(node->get_logger(), "规划失败，在 %zu 次尝试后仍无法找到有效路径！", moveit_max_iter);
        return -1;
    }

    // 构建控制器动作服务器名称
    std::string controller_name;
    if (id.find("left") != std::string::npos) {
        controller_name = "/left_controller/follow_joint_trajectory";
    } else if (id.find("right") != std::string::npos) {
        controller_name = "/right_controller/follow_joint_trajectory";
    } else {
        RCLCPP_ERROR(node->get_logger(), "无效的机械臂 ID: %s", id.c_str());
        return -1;
    }

    auto action_client = rclcpp_action::create_client<FollowJointTrajectory>(node, controller_name);

    if (!action_client->wait_for_action_server(std::chrono::seconds(10))) {
        RCLCPP_ERROR(node->get_logger(), "动作服务器 %s 未响应！", controller_name.c_str());
        return -1;
    }

    RCLCPP_INFO(node->get_logger(), "准备向控制器 %s 发送关节轨迹...", controller_name.c_str());

    FollowJointTrajectory::Goal goal_msg;
    goal_msg.trajectory = my_plan.trajectory_.joint_trajectory;

    auto send_goal_options = rclcpp_action::Client<FollowJointTrajectory>::SendGoalOptions();
    send_goal_options.result_callback =
        [node](const rclcpp_action::ClientGoalHandle<FollowJointTrajectory>::WrappedResult & result) {
            switch (result.code) {
                case rclcpp_action::ResultCode::SUCCEEDED:
                    RCLCPP_INFO(node->get_logger(), "关节轨迹执行成功！");
                    break;
                case rclcpp_action::ResultCode::ABORTED:
                    RCLCPP_ERROR(node->get_logger(), "关节轨迹执行被中止！");
                    break;
                case rclcpp_action::ResultCode::CANCELED:
                    RCLCPP_ERROR(node->get_logger(), "关节轨迹执行被取消！");
                    break;
                default:
                    RCLCPP_ERROR(node->get_logger(), "未知的结果码！");
                    break;
            }
        };

    auto future_goal_handle = action_client->async_send_goal(goal_msg, send_goal_options);
    (void)future_goal_handle;

    RCLCPP_INFO(node->get_logger(), "关节轨迹已发送至控制器，正在执行...");
    return 0;
}

/**
 * @brief 主函数：初始化 ROS 2 上下文，执行双臂运动控制流程
 */
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    RCLCPP_INFO(LOGGER, "开始启动 dual_move 节点...");

    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);

    auto move_group_node = rclcpp::Node::make_shared("dual_move", node_options);
    move_group_node->set_parameter({"use_sim_time", true});

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(move_group_node);
    
    // 启动执行器线程
    std::thread([&executor]() { executor.spin(); }).detach();
    
    // 等待系统完全就绪
    rclcpp::sleep_for(std::chrono::seconds(5));

    // ==========================================
    // 测试部分 1: SRDF 命名姿态测试
    // ==========================================
    RCLCPP_INFO(LOGGER, ">>> 测试阶段 1: SRDF 命名姿态");
    
    RCLCPP_INFO(LOGGER, "正在执行左臂动作 (SRDF)...");
    if (to_srdf_NamedTarget("left", "left_pose1", 0.5) != 0) {
        RCLCPP_WARN(LOGGER, "左臂 SRDF 规划失败，继续后续测试...");
    }
    rclcpp::sleep_for(std::chrono::seconds(3));

    RCLCPP_INFO(LOGGER, "正在执行右臂动作 (SRDF)...");
    if (to_srdf_NamedTarget("right", "right_pose1", 0.5) != 0) {
        RCLCPP_WARN(LOGGER, "右臂 SRDF 规划失败，继续后续测试...");
    }
    rclcpp::sleep_for(std::chrono::seconds(3));

    // ==========================================
    // 测试部分 2: 指定关节角度配置测试 (单位：度)
    // ==========================================
    RCLCPP_INFO(LOGGER, ">>> 测试阶段 2: 指定关节角度配置 (单位：度)");

    // 示例：假设每个臂有 6 个活动关节（请根据实际机器人调整！）
    std::vector<double> left_arm_joints_deg = {0.0, -45.0, 90.0, 0.0, 45.0, 0.0}; 
    std::vector<double> right_arm_joints_deg = {0.0, 45.0, -90.0, 0.0, -45.0, 0.0};

    RCLCPP_INFO(LOGGER, "正在执行左臂动作 (关节角度模式)...");
    RCLCPP_INFO_STREAM(LOGGER, "目标角度：" << [&]() {
        std::stringstream ss;
        for(auto a : left_arm_joints_deg) ss << a << " ";
        return ss.str();
    }());
    
    if (to_joint_config("left", left_arm_joints_deg, 0.5) != 0) {
        RCLCPP_WARN(LOGGER, "左臂关节角度规划失败");
    }
    rclcpp::sleep_for(std::chrono::seconds(3));

    RCLCPP_INFO(LOGGER, "正在执行右臂动作 (关节角度模式)...");
    RCLCPP_INFO_STREAM(LOGGER, "目标角度：" << [&]() {
        std::stringstream ss;
        for(auto a : right_arm_joints_deg) ss << a << " ";
        return ss.str();
    }());

    if (to_joint_config("right", right_arm_joints_deg, 0.5) != 0) {
        RCLCPP_WARN(LOGGER, "右臂关节角度规划失败");
    }

    // ==========================================
    // 结束部分
    // ==========================================
    RCLCPP_INFO(LOGGER, "所有测试指令已下发。程序将在后台运行以接收回调结果。");
    RCLCPP_INFO(LOGGER, "Waiting... Press \"CTRL + C\" to shutdown.");

    rclcpp::Rate rate(0.5);
    while (rclcpp::ok())
    {
        rate.sleep();
    }

    RCLCPP_INFO(LOGGER, "关闭 ROS 2 系统...");
    rclcpp::shutdown();

    return EXIT_SUCCESS;
}