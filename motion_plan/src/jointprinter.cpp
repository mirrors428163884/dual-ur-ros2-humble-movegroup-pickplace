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
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h> 
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <urdf/model.h> // 需要安装 urdf 依赖
#include <rclcpp/parameter.hpp> // 确保包含此头文件

// 全局日志器
static const rclcpp::Logger LOGGER = rclcpp::get_logger("dual_move");
using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;

// 定义 PI，用于角度转弧度
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/**
 * @brief 打印指定机械臂当前的关节名称和角度（单位：度）
 */
void printCurrentJointStates(const rclcpp::Node::SharedPtr& main_node, const std::string& arm_id)
{
    // 创建临时节点用于订阅 /joint_states（需 TRANSIENT_LOCAL QoS）
    auto fetcher_node = rclcpp::Node::make_shared("state_fetcher_" + arm_id);
    auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();

    sensor_msgs::msg::JointState::SharedPtr latest_state = nullptr;
    std::mutex state_mutex;

    auto subscription = fetcher_node->create_subscription<sensor_msgs::msg::JointState>(
        "/joint_states", qos_profile,
        [&latest_state, &state_mutex](const sensor_msgs::msg::JointState::SharedPtr msg) {
            std::lock_guard<std::mutex> lock(state_mutex);
            latest_state = msg;
        }
    );

    // ✅ 使用主节点创建 MoveGroupInterface（确保 robot_description 可用）
    auto move_group = std::make_shared<moveit::planning_interface::MoveGroupInterface>(main_node, arm_id);
    const std::vector<std::string>& joint_names = move_group->getActiveJoints();

    // 等待 joint_states 数据
    auto start_time = fetcher_node->now();
    while (rclcpp::ok() && (fetcher_node->now() - start_time).seconds() < 2.0) {
        rclcpp::spin_some(fetcher_node);
        if (latest_state) break;
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    if (!latest_state) {
        RCLCPP_ERROR(LOGGER, "超时未收到 /joint_states 数据！");
        return;
    }

    // 映射关节值
    std::vector<double> joint_values(joint_names.size(), 0.0);
    for (size_t i = 0; i < joint_names.size(); ++i) {
        bool found = false;
        for (size_t j = 0; j < latest_state->name.size(); ++j) {
            if (latest_state->name[j] == joint_names[i]) {
                joint_values[i] = latest_state->position[j];
                found = true;
                break;
            }
        }
        if (!found) {
            RCLCPP_WARN(LOGGER, "关节 '%s' 未在 /joint_states 中找到。", joint_names[i].c_str());
        }
    }

    RCLCPP_INFO(LOGGER, "=== 机械臂 %s 当前关节状态 ===", arm_id.c_str());
    for (size_t i = 0; i < joint_names.size(); ++i) {
        double deg = joint_values[i] * 180.0 / M_PI;
        RCLCPP_INFO(LOGGER, "  %s: %.2f°", joint_names[i].c_str(), deg);
    }
}








/**
 * @brief 从 ROS 2 参数服务器加载碰撞体配置
 * 
 * 自动处理三种可能的情况：
 * 1. 标准路径：collision_object.id
 * 2. ros__parameters 前缀：ros__parameters.collision_object.id
 * 3. 节点名前缀 (当前问题)：dual_move.collision_object.id
 */
bool load_collision_object_params(
    const rclcpp::Node::SharedPtr& node,
    std::string& object_id,
    std::string& frame_id,
    std::string& shape,
    std::vector<double>& dimensions,
    geometry_msgs::msg::Pose& pose)
{
    try {
        // 1. 获取当前节点名称
        std::string node_name = node->get_name();
        RCLCPP_INFO(LOGGER, "当前节点名称：%s", node_name.c_str());

        // 2. 获取所有参数名用于分析
        auto param_names = node->list_parameters({}, 0).names;
        
        // 3. 确定正确的前缀
        std::string prefix = "";
        
        // 检查是否有 "ros__parameters" 前缀
        bool has_ros_params = false;
        for (const auto& name : param_names) {
            if (name.find("ros__parameters.collision_object") != std::string::npos) {
                has_ros_params = true;
                break;
            }
        }

        // 检查是否有 "节点名" 前缀 (例如 dual_move.collision_object)
        bool has_node_prefix = false;
        std::string node_prefix_candidate = node_name + ".collision_object";
        for (const auto& name : param_names) {
            if (name.find(node_prefix_candidate) != std::string::npos) {
                has_node_prefix = true;
                break;
            }
        }

        // 决定使用哪个前缀
        if (has_ros_params) {
            prefix = "ros__parameters.";
            RCLCPP_INFO(LOGGER, "检测到 'ros__parameters' 前缀模式。");
        } else if (has_node_prefix) {
            prefix = node_name + ".";
            RCLCPP_INFO(LOGGER, "检测到节点名 '%s' 前缀模式，将使用 '%s' 作为前缀。", node_name.c_str(), prefix.c_str());
        } else {
            RCLCPP_INFO(LOGGER, "未检测到特殊前缀，使用标准路径。");
        }

        // 4. 定义辅助 Lambda 来读取参数
        auto get_param_str = [&](const std::string& suffix) -> std::string {
            return node->get_parameter(prefix + suffix).as_string();
        };
        
        auto get_param_double = [&](const std::string& suffix) -> double {
            return node->get_parameter(prefix + suffix).as_double();
        };

        auto get_param_array = [&](const std::string& suffix) -> std::vector<double> {
            return node->get_parameter(prefix + suffix).as_double_array();
        };

        // --- 开始读取 ---
        // 注意：suffix 始终是相对路径 "collision_object.xxx"
        
        object_id = get_param_str("collision_object.id");
        frame_id = get_param_str("collision_object.frame_id");
        shape = get_param_str("collision_object.shape");
        
        dimensions = get_param_array("collision_object.dimensions");
        if (dimensions.size() != 3) {
            RCLCPP_ERROR(LOGGER, "尺寸数组长度错误，应为 3，实际为：%zu", dimensions.size());
            return false;
        }

        pose.position.x = get_param_double("collision_object.pose.position.x");
        pose.position.y = get_param_double("collision_object.pose.position.y");
        pose.position.z = get_param_double("collision_object.pose.position.z");
        
        pose.orientation.x = get_param_double("collision_object.pose.orientation.x");
        pose.orientation.y = get_param_double("collision_object.pose.orientation.y");
        pose.orientation.z = get_param_double("collision_object.pose.orientation.z");
        pose.orientation.w = get_param_double("collision_object.pose.orientation.w");

        // --- 成功日志 ---
        RCLCPP_INFO(LOGGER,
            "✅ 成功加载物体 '%s' (形状：%s) 在帧 '%s' 下。\n"
            "   尺寸：[%.3f, %.3f, %.3f]\n"
            "   位置：(%.3f, %.3f, %.3f)\n"
            "   朝向：(%.3f, %.3f, %.3f, %.3f)",
            object_id.c_str(), shape.c_str(), frame_id.c_str(),
            dimensions[0], dimensions[1], dimensions[2],
            pose.position.x, pose.position.y, pose.position.z,
            pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);

        return true;

    } catch (const rclcpp::exceptions::ParameterNotDeclaredException& e) {
        RCLCPP_ERROR(LOGGER, "❌ 参数未找到：%s", e.what());
        RCLCPP_INFO(LOGGER, "调试信息 - 当前所有包含 'collision_object' 的参数：");
        auto all_params = node->list_parameters({}, 0).names;
        for(const auto& p : all_params) {
            if(p.find("collision_object") != std::string::npos) {
                RCLCPP_INFO(LOGGER, "  - %s", p.c_str());
            }
        }
        return false;
    } catch (const std::exception& e) {
        RCLCPP_ERROR(LOGGER, "❌ 读取参数时发生错误：%s", e.what());
        return false;
    }
}

/**
 * @brief 创建一个 CollisionObject
 * 
 * 此函数使用从 YAML 加载的参数来构造物体。
 */
bool create_target_plate_collision_object(
    const rclcpp::Node::SharedPtr& node,
    moveit_msgs::msg::CollisionObject& collision_object)
{
    std::string object_id, frame_id, shape; // <-- 1. 在这里添加 'shape' 变量
    std::vector<double> dimensions;
    geometry_msgs::msg::Pose pose;

    // 2. 在调用时，将 'shape' 作为第四个参数传入
    if (!load_collision_object_params(node, object_id, frame_id, shape, dimensions, pose)) {
        return false;
    }

    // 1. 设置 ID 和参考系
    collision_object.id = object_id;
    collision_object.header.frame_id = frame_id;
    collision_object.operation = moveit_msgs::msg::CollisionObject::ADD;

    // 2. 定义几何形状 (假设是 Box)
    shape_msgs::msg::SolidPrimitive box;
    box.type = box.BOX;
    if (dimensions.size() == 3) {
        box.dimensions.resize(3);
        box.dimensions[box.BOX_X] = dimensions[0];
        box.dimensions[box.BOX_Y] = dimensions[1];
        box.dimensions[box.BOX_Z] = dimensions[2];
    } else {
        RCLCPP_ERROR(LOGGER, "YAML 中的 dimensions 参数必须包含3个值 (x, y, z)");
        return false;
    }

    collision_object.primitives.push_back(box);
    collision_object.primitive_poses.push_back(pose);
    return true;
}

/**
 * @brief 将已存在的物体附着到机械臂末端
 * 
 * 注意：此函数假设名为 object_name 的物体已经作为 world object 存在于规划场景中。
 */
bool attach_target_plate(const rclcpp::Node::SharedPtr& node, const std::string& arm_id, const std::string& object_name)
{
    auto move_group = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node, arm_id);
    const std::string ee_link = (arm_id == "left") ? "left_ee_link" : "right_ee_link";

    // 定义 Touch Links
    std::vector<std::string> touch_links = {
        ee_link,
        arm_id + "_tool0",
        arm_id + "_flange",
        arm_id + "_robotiq_85_left_finger_tip_link", 
        arm_id + "_robotiq_85_right_finger_tip_link"
    };

    RCLCPP_INFO(LOGGER, "Attaching '%s' to '%s'...", object_name.c_str(), ee_link.c_str());
    bool success = move_group->attachObject(object_name, ee_link, touch_links);
    
    if (success) {
        RCLCPP_INFO(LOGGER, "SUCCESS: Object '%s' is now attached.", object_name.c_str());
        move_group->setStartStateToCurrentState();
    } else {
        RCLCPP_ERROR(LOGGER, "FAILED: Could not attach object '%s'.", object_name.c_str());
    }

    return success;
}

/**
 * @brief 分离物体
 * 
 * 分离后，物体将自动变回 world object。
 */
bool detach_target_plate(const rclcpp::Node::SharedPtr& node, const std::string& arm_id, const std::string& object_name)
{
    auto move_group = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node, arm_id);

    RCLCPP_INFO(LOGGER, "Attempting to detach object '%s'...", object_name.c_str());
    bool success = move_group->detachObject(object_name);
    
    if (success) {
        RCLCPP_INFO(LOGGER, "SUCCESS: Object '%s' detached.", object_name.c_str());
        move_group->setStartStateToCurrentState();
    } else {
        RCLCPP_ERROR(LOGGER, "FAILED: Could not detach object '%s'.", object_name.c_str());
    }

    return success;
}




/**
 * @brief 根据机械臂 ID 和 SRDF 中预定义的姿态名称执行运动规划并发送轨迹
 */
int to_srdf_NamedTarget(const rclcpp::Node::SharedPtr& node, const std::string &id, const std::string &srdf_pose_id, double speed)
{
    RCLCPP_INFO(node->get_logger(), "[to_srdf_NamedTarget] 开始为机械臂 %s 规划目标姿态：%s", id.c_str(), srdf_pose_id.c_str());

    auto move_group = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node, id);
    move_group->setMaxVelocityScalingFactor(speed);
    move_group->setMaxAccelerationScalingFactor(speed);

    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    move_group->setStartStateToCurrentState();
    move_group->setNamedTarget(srdf_pose_id);

    const size_t moveit_max_iter = 5;
    moveit::core::MoveItErrorCode plan_ret;
    for (size_t test_iter = 0; test_iter < moveit_max_iter; ++test_iter) {
        plan_ret = move_group->plan(my_plan);
        if (plan_ret == moveit::core::MoveItErrorCode::SUCCESS) {
            RCLCPP_INFO(node->get_logger(), "路径规划成功！");
            break;
        }
        rclcpp::sleep_for(std::chrono::milliseconds(100));
    }

    if (plan_ret != moveit::core::MoveItErrorCode::SUCCESS) {
        RCLCPP_ERROR(node->get_logger(), "规划失败，在 %zu 次尝试后仍无法找到有效路径！", moveit_max_iter);
        return -1;
    }

    std::string controller_name = (id.find("left") != std::string::npos) 
        ? "/left_controller/follow_joint_trajectory" 
        : "/right_controller/follow_joint_trajectory";

    auto action_client = rclcpp_action::create_client<FollowJointTrajectory>(node, controller_name);
    if (!action_client->wait_for_action_server(std::chrono::seconds(10))) {
        RCLCPP_ERROR(node->get_logger(), "动作服务器 %s 未响应！", controller_name.c_str());
        return -1;
    }

    FollowJointTrajectory::Goal goal_msg;
    goal_msg.trajectory = my_plan.trajectory_.joint_trajectory;

    auto send_goal_options = rclcpp_action::Client<FollowJointTrajectory>::SendGoalOptions();
    send_goal_options.result_callback = [node](const auto & result) {
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

    action_client->async_send_goal(goal_msg, send_goal_options);
    RCLCPP_INFO(node->get_logger(), "轨迹已发送至控制器，正在执行...");
    return 0;
}

/**
 * @brief 根据机械臂 ID 和指定的关节角度数组 (单位：度) 执行运动规划并发送轨迹
 */
int to_joint_config(const rclcpp::Node::SharedPtr& node, const std::string &id, const std::vector<double> &joint_angles_deg, double speed)
{
    RCLCPP_INFO(node->get_logger(), "[to_joint_config] 开始为机械臂 %s 规划关节角度目标", id.c_str());

    auto move_group = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node, id);
    auto robot_model = move_group->getRobotModel();
    if (!robot_model) {
        RCLCPP_ERROR(node->get_logger(), "无法获取 RobotModel！");
        return -1;
    }

    auto joint_model_group = robot_model->getJointModelGroup(id);
    if (!joint_model_group) {
        RCLCPP_ERROR(node->get_logger(), "找不到规划组: %s", id.c_str());
        return -1;
    }

    const std::vector<std::string>& active_joints = joint_model_group->getActiveJointModelNames();
    if (joint_angles_deg.size() != active_joints.size()) {
        RCLCPP_ERROR(node->get_logger(), "错误：提供的角度数量 (%zu) 与活动关节数量 (%zu) 不匹配！", 
                     joint_angles_deg.size(), active_joints.size());
        return -1;
    }

    move_group->setMaxVelocityScalingFactor(speed);
    move_group->setMaxAccelerationScalingFactor(speed);

    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    move_group->setStartStateToCurrentState();

    std::vector<double> joint_angles_rad;
    for (double deg : joint_angles_deg) {
        joint_angles_rad.push_back(deg * M_PI / 180.0);
    }

    if (!move_group->setJointValueTarget(joint_angles_rad)) {
        RCLCPP_ERROR(node->get_logger(), "设置关节目标值失败");
        return -1;
    }

    const size_t moveit_max_iter = 5;
    moveit::core::MoveItErrorCode plan_ret;
    for (size_t test_iter = 0; test_iter < moveit_max_iter; ++test_iter) {
        plan_ret = move_group->plan(my_plan);
        if (plan_ret == moveit::core::MoveItErrorCode::SUCCESS) {
            RCLCPP_INFO(node->get_logger(), "路径规划成功！");
            break;
        }
        rclcpp::sleep_for(std::chrono::milliseconds(100));
    }

    if (plan_ret != moveit::core::MoveItErrorCode::SUCCESS) {
        RCLCPP_ERROR(node->get_logger(), "规划失败，在 %zu 次尝试后仍无法找到有效路径！", moveit_max_iter);
        return -1;
    }

    std::string controller_name = (id.find("left") != std::string::npos) 
        ? "/left_controller/follow_joint_trajectory" 
        : "/right_controller/follow_joint_trajectory";

    auto action_client = rclcpp_action::create_client<FollowJointTrajectory>(node, controller_name);
    if (!action_client->wait_for_action_server(std::chrono::seconds(10))) {
        RCLCPP_ERROR(node->get_logger(), "动作服务器 %s 未响应！", controller_name.c_str());
        return -1;
    }

    FollowJointTrajectory::Goal goal_msg;
    goal_msg.trajectory = my_plan.trajectory_.joint_trajectory;

    auto send_goal_options = rclcpp_action::Client<FollowJointTrajectory>::SendGoalOptions();
    send_goal_options.result_callback = [node](const auto & result) {
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

    action_client->async_send_goal(goal_msg, send_goal_options);
    RCLCPP_INFO(node->get_logger(), "关节轨迹已发送至控制器，正在执行...");
    return 0;
}

/**
 * @brief 主函数：初始化 ROS 2 上下文，执行双臂运动控制流程
 */
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

    // === 关键步骤：加载 YAML 配置 ===
    // 确保你的 launch 文件正确加载了 pick_object.yaml 到这个节点的命名空间
    // 例如: parameters=[{'file_path_to_pick_object.yaml'}]
    RCLCPP_INFO(LOGGER, "正在从参数服务器加载 YAML 配置...");
    // 参数会在 load_collision_object_params 中被声明和读取

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(move_group_node);
    std::thread([&executor]() { executor.spin(); }).detach();
    
    // 等待系统完全启动
    rclcpp::sleep_for(std::chrono::seconds(5));

    // === 打印初始状态 ===
    printCurrentJointStates(move_group_node, "left");
    printCurrentJointStates(move_group_node, "right");

    const std::string ARM_ID = "left";

    // === 步骤 0: 创建并添加 target_plate 作为世界物体 ===
    RCLCPP_INFO(LOGGER, ">>> 正在创建并添加世界物体 ...");
    moveit_msgs::msg::CollisionObject co;
    if (!create_target_plate_collision_object(move_group_node, co)) {
        RCLCPP_ERROR(LOGGER, "创建世界物体失败！");
        return EXIT_FAILURE;
    }

    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    planning_scene_interface.applyCollisionObjects({co});
    rclcpp::sleep_for(std::chrono::milliseconds(1000)); // 等待 RViz 更新
    RCLCPP_INFO(LOGGER, ">>> 世界物体 '%s' 添加成功。", co.id.c_str());

    // === 移动到预抓取位置 ===
    RCLCPP_INFO(LOGGER, ">>> 移动左臂到预抓取位姿...");
    if (to_srdf_NamedTarget(move_group_node, ARM_ID, "left_pose1", 0.3) != 0) {
        RCLCPP_WARN(LOGGER, "左臂移动到预抓取位姿失败，但将继续尝试附着（假设已在附近）");
    }
    rclcpp::sleep_for(std::chrono::seconds(2));

    // === 步骤 1: 附着物体 ===
    RCLCPP_INFO(LOGGER, "尝试附着物体 '%s' 到 %s 臂...", co.id.c_str(), ARM_ID.c_str());
    if (!attach_target_plate(move_group_node, ARM_ID, co.id)) {
        RCLCPP_ERROR(LOGGER, "附着失败！");
        return EXIT_FAILURE;
    }
    rclcpp::sleep_for(std::chrono::seconds(2));

    // === 步骤 2: 携带物体移动 ===
    RCLCPP_INFO(LOGGER, "移动左臂（携带 '%s'）到新位置...", co.id.c_str());
    std::vector<double> carry_pose_deg = {80.0, -40.0, -90.0, -20.0, 90.0, 0.0};
    if (to_joint_config(move_group_node, ARM_ID, carry_pose_deg, 0.3) != 0) {
        RCLCPP_WARN(LOGGER, "携带平板移动失败");
    }
    rclcpp::sleep_for(std::chrono::seconds(3));

    // === 步骤 3: 分离物体 ===
    RCLCPP_INFO(LOGGER, "分离物体 '%s'...", co.id.c_str());
    if (!detach_target_plate(move_group_node, ARM_ID, co.id)) {
        RCLCPP_WARN(LOGGER, "分离失败！");
    }
    rclcpp::sleep_for(std::chrono::seconds(1));

    RCLCPP_INFO(LOGGER, "所有测试完成。按 CTRL+C 退出。");
    
    rclcpp::Rate rate(0.5);
    while (rclcpp::ok()) { 
        rate.sleep(); 
    }
    
    rclcpp::shutdown();
    return EXIT_SUCCESS;
}