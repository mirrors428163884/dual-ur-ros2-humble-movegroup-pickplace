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








// 在文件顶部或其他合适位置添加此辅助函数
moveit_msgs::msg::CollisionObject createTargetPlateCollisionObject()
{
    moveit_msgs::msg::CollisionObject collision_object;
    collision_object.header.frame_id = "world"; // 使用世界坐标系
    collision_object.id = "target_plate";

    // 定义几何形状（盒子）
    shape_msgs::msg::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[primitive.BOX_X] = 0.2;  // 长
    primitive.dimensions[primitive.BOX_Y] = 0.15; // 宽
    primitive.dimensions[primitive.BOX_Z] = 0.01; // 高

    // 定义姿态（位置和朝向）
    // 根据你的 URDF 注释: plate 中心在 z = 0.78
    geometry_msgs::msg::Pose pose;
    pose.orientation.w = 1.0; // 无旋转
    pose.position.x = 0.0;
    pose.position.y = 0.0;
    pose.position.z = 0.78; // 桌面 z=0.75 + 厚度 0.05/2 + plate厚度 0.01/2 = 0.75+0.025+0.005=0.78

    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(pose);
    collision_object.operation = collision_object.ADD;

    return collision_object;
}

/**
 * @brief 将目标平板附着到指定机械臂的末端执行器
 * 
 * 此函数会先将 target_plate 作为一个独立的 CollisionObject 添加到规划场景中，
 * 然后再将其附着到机械臂上。
 */
bool attach_target_plate(const rclcpp::Node::SharedPtr& node, const std::string& arm_id)
{
    // 1. 创建 PlanningSceneInterface 来管理场景中的物体
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    
    // 2. 创建 MoveGroupInterface 用于附着操作
    auto move_group = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node, arm_id);

    // 3. 定义物体和链接名称
    std::string object_name = "target_plate";
    std::string ee_link = (arm_id == "left") ? "left_ee_link" : "right_ee_link";

    // 4. 定义允许接触的链接
    std::vector<std::string> touch_links = {
        ee_link,
        arm_id + "_tool0",
        arm_id + "_flange",
        arm_id + "_wrist_3_link"
    };

    // --- 关键步骤：管理规划场景中的物体 ---
    // 4.1 先移除场景中任何已存在的同名物体（清理）
    std::vector<std::string> objects_to_remove = {object_name};
    planning_scene_interface.removeCollisionObjects(objects_to_remove);
    RCLCPP_INFO(LOGGER, "已从规划场景中移除旧的 '%s'。", object_name.c_str());
    
    // 4.2 等待移除操作完成
    rclcpp::sleep_for(std::chrono::milliseconds(500));

    // 4.3 创建并添加新的碰撞对象
    moveit_msgs::msg::CollisionObject collision_object = createTargetPlateCollisionObject();
    std::vector<moveit_msgs::msg::CollisionObject> collision_objects;
    collision_objects.push_back(collision_object);
    planning_scene_interface.addCollisionObjects(collision_objects);
    RCLCPP_INFO(LOGGER, "已将 '%s' 作为独立碰撞对象添加到规划场景中。", object_name.c_str());

    // 4.4 等待场景完全更新（非常重要！）
    rclcpp::sleep_for(std::chrono::seconds(1));

    // --- 执行附着 ---
    bool success = move_group->attachObject(object_name, ee_link, touch_links);
    if (success) {
        RCLCPP_INFO(LOGGER, "成功将 %s 附着到 %s！", object_name.c_str(), ee_link.c_str());
    } else {
        RCLCPP_ERROR(LOGGER, "附着失败！请检查规划场景和 MoveIt 配置。");
    }
    return success;
}

/**
 * @brief 从末端执行器分离目标平板，并将其放回环境中
 */
bool detach_target_plate(const rclcpp::Node::SharedPtr& node, const std::string& arm_id)
{
    // 1. 创建接口
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    auto move_group = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node, arm_id);

    std::string object_name = "target_plate";

    // 2. 执行分离
    bool success = move_group->detachObject(object_name);
    if (success) {
        RCLCPP_INFO(LOGGER, "成功从机械臂分离 %s！", object_name.c_str());
        
        // 3. 分离后，将物体作为静态障碍物重新添加回环境中
        // （可选：这里可以更新物体的位置为当前EE的位置，或者放回原位）
        moveit_msgs::msg::CollisionObject collision_object = createTargetPlateCollisionObject();
        // 如果你想让它留在分离时的位置，可以在这里更新 collision_object 的 pose
        // 例如：collision_object.primitive_poses[0] = current_ee_pose;
        
        std::vector<moveit_msgs::msg::CollisionObject> collision_objects;
        collision_objects.push_back(collision_object);
        planning_scene_interface.addCollisionObjects(collision_objects);
        RCLCPP_INFO(LOGGER, "已将 %s 作为静态障碍物放回环境中。", object_name.c_str());
        
    } else {
        RCLCPP_ERROR(LOGGER, "分离 %s 失败！", object_name.c_str());
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
    std::thread([&executor]() { executor.spin(); }).detach();
    rclcpp::sleep_for(std::chrono::seconds(5));

    // === 打印初始状态 ===
    printCurrentJointStates(move_group_node, "left");
    printCurrentJointStates(move_group_node, "right");

    // // === 新增：抓取测试 ===
    // RCLCPP_INFO(LOGGER, ">>> 测试阶段 3: 抓取平板");
    // RCLCPP_INFO(LOGGER, "移动左臂到抓取位姿...");
    // if (to_srdf_NamedTarget(move_group_node, "left", "left_pose1", 0.3) != 0) {
    //     RCLCPP_WARN(LOGGER, "左臂移动到抓取位姿失败");
    // }
    // rclcpp::sleep_for(std::chrono::seconds(2));

    RCLCPP_INFO(LOGGER, "尝试附着平板到左臂...");
    if (!attach_target_plate(move_group_node, "left")) {
        RCLCPP_ERROR(LOGGER, "附着失败！");
    }
    rclcpp::sleep_for(std::chrono::seconds(2));

    RCLCPP_INFO(LOGGER, "移动左臂（携带平板）到新位置...");
    std::vector<double> carry_pose_deg = {80.0, -40.0, -90.0, -20.0, 90.0, 0.0};
    if (to_joint_config(move_group_node, "left", carry_pose_deg, 0.3) != 0) {
        RCLCPP_WARN(LOGGER, "携带平板移动失败");
    }
    rclcpp::sleep_for(std::chrono::seconds(3));

    RCLCPP_INFO(LOGGER, "分离平板...");
    if (!detach_target_plate(move_group_node, "left")) {
        RCLCPP_WARN(LOGGER, "分离失败！");
    }
    rclcpp::sleep_for(std::chrono::seconds(1));

    RCLCPP_INFO(LOGGER, "所有测试完成。按 CTRL+C 退出。");
    rclcpp::Rate rate(0.5);
    while (rclcpp::ok()) { rate.sleep(); }
    rclcpp::shutdown();
    return EXIT_SUCCESS;
}