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
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <control_msgs/action/gripper_command.hpp>


// 全局日志器
static const rclcpp::Logger LOGGER = rclcpp::get_logger("dual_move");
using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
using GripperCommand = control_msgs::action::GripperCommand;

// 定义 PI，用于角度转弧度
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/**
 * @brief 打印指定机械臂当前的关节状态，并通过 MoveGroup 和 TF 两种方式获取末端位姿进行对比
 * 
 * 注意: 现在需要传入一个已经初始化好的 tf_buffer。
 */
/**
 * @brief 打印指定机械臂当前的关节状态，并通过 MoveGroup (使用真实关节值) 和 TF 两种方式获取末端位姿进行对比
 */
void printCurrentJointStates(
    const rclcpp::Node::SharedPtr& main_node, 
    const std::string& arm_id,
    const std::shared_ptr<tf2_ros::Buffer>& tf_buffer)
{
    // === 第一部分：打印关节状态（保持不变）===
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

    auto move_group = std::make_shared<moveit::planning_interface::MoveGroupInterface>(main_node, arm_id);
    const std::vector<std::string>& joint_names = move_group->getActiveJoints();

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

    // === 第二部分：通过 MoveGroup (使用真实关节值) 获取末端位姿 ===
    // 1. 获取 RobotModel
    auto robot_model = move_group->getRobotModel();
    // 2. 创建一个新的 RobotState 并用我们获取的真实关节值填充它
    moveit::core::RobotStatePtr current_state(new moveit::core::RobotState(robot_model));
    current_state->setToDefaultValues(); // 先设为默认值
    current_state->setJointGroupPositions(arm_id, joint_values); // 再用真实值覆盖
    current_state->update(); // 非常重要！触发 FK 计算

    std::string ee_link = (arm_id == "left") ? "left_ee_link" : "right_ee_link";
    const Eigen::Isometry3d& end_effector_state = current_state->getGlobalLinkTransform(ee_link);

    // 将 Eigen::Isometry3d 转换为 geometry_msgs::msg::PoseStamped
    geometry_msgs::msg::PoseStamped pose_movegroup;
    pose_movegroup.header.frame_id = robot_model->getModelFrame(); // 通常是 "world"
    pose_movegroup.header.stamp = main_node->now();
    pose_movegroup.pose.position.x = end_effector_state.translation().x();
    pose_movegroup.pose.position.y = end_effector_state.translation().y();
    pose_movegroup.pose.position.z = end_effector_state.translation().z();
    Eigen::Quaterniond quat(end_effector_state.rotation());
    pose_movegroup.pose.orientation.x = quat.x();
    pose_movegroup.pose.orientation.y = quat.y();
    pose_movegroup.pose.orientation.z = quat.z();
    pose_movegroup.pose.orientation.w = quat.w();

    RCLCPP_INFO(LOGGER, "=== 通过 MoveGroup (使用真实关节值) 获取的末端位姿（参考系: %s）===", pose_movegroup.header.frame_id.c_str());
    RCLCPP_INFO(LOGGER, "  位置 [x, y, z]: (%.4f, %.4f, %.4f)",
                pose_movegroup.pose.position.x, pose_movegroup.pose.position.y, pose_movegroup.pose.position.z);
    RCLCPP_INFO(LOGGER, "  四元数 [x, y, z, w]: (%.4f, %.4f, %.4f, %.4f)",
                pose_movegroup.pose.orientation.x, pose_movegroup.pose.orientation.y,
                pose_movegroup.pose.orientation.z, pose_movegroup.pose.orientation.w);

    // 转换为 RPY
    tf2::Quaternion quat_mg(pose_movegroup.pose.orientation.x, pose_movegroup.pose.orientation.y,
                            pose_movegroup.pose.orientation.z, pose_movegroup.pose.orientation.w);
    tf2::Matrix3x3 mat_mg(quat_mg);
    double roll_mg, pitch_mg, yaw_mg;
    mat_mg.getRPY(roll_mg, pitch_mg, yaw_mg);
    RCLCPP_INFO(LOGGER, "  RPY (角度): roll=%.1f°, pitch=%.1f°, yaw=%.1f°",
                roll_mg * 180.0 / M_PI, pitch_mg * 180.0 / M_PI, yaw_mg * 180.0 / M_PI);



    // === 第三部分：通过 TF 获取末端位姿（使用传入的全局 TF Buffer）===
    try {
        // 移除内部创建 Buffer 的代码，直接使用传入的 tf_buffer
        std::string target_frame = "world";

        // 因为 tf_buffer 是全局的并且早已初始化，我们可以安全地查询
        if (tf_buffer->canTransform(target_frame, ee_link, tf2::TimePointZero, std::chrono::seconds(1))) {
            auto transform = tf_buffer->lookupTransform(target_frame, ee_link, tf2::TimePointZero);

            RCLCPP_INFO(LOGGER, "=== 通过 TF 获取的末端位姿（参考系: %s）===", target_frame.c_str());
            RCLCPP_INFO(LOGGER, "  位置 [x, y, z]: (%.4f, %.4f, %.4f)",
                        transform.transform.translation.x, transform.transform.translation.y, transform.transform.translation.z);
            RCLCPP_INFO(LOGGER, "  四元数 [x, y, z, w]: (%.4f, %.4f, %.4f, %.4f)",
                        transform.transform.rotation.x, transform.transform.rotation.y,
                        transform.transform.rotation.z, transform.transform.rotation.w);

            tf2::Quaternion quat_tf(transform.transform.rotation.x, transform.transform.rotation.y,
                                    transform.transform.rotation.z, transform.transform.rotation.w);
            tf2::Matrix3x3 mat_tf(quat_tf);
            double roll_tf, pitch_tf, yaw_tf;
            mat_tf.getRPY(roll_tf, pitch_tf, yaw_tf);
            RCLCPP_INFO(LOGGER, "  RPY (角度): roll=%.1f°, pitch=%.1f°, yaw=%.1f°",
                        roll_tf * 180.0 / M_PI, pitch_tf * 180.0 / M_PI, yaw_tf * 180.0 / M_PI);

            // === 对比差异 ===
            double pos_diff = std::sqrt(
                std::pow(pose_movegroup.pose.position.x - transform.transform.translation.x, 2) +
                std::pow(pose_movegroup.pose.position.y - transform.transform.translation.y, 2) +
                std::pow(pose_movegroup.pose.position.z - transform.transform.translation.z, 2)
            );
            RCLCPP_INFO(LOGGER, ">>> 位置差异（欧氏距离）: %.6f m", pos_diff);
        } else {
            RCLCPP_WARN(LOGGER, "TF 变换不可用: %s -> %s", target_frame.c_str(), ee_link.c_str());
        }
    } catch (const tf2::TransformException &ex) {
        RCLCPP_ERROR(LOGGER, "TF 查询失败: %s", ex.what());
    }
}



/**
 * @brief 打印从目标坐标系到源坐标系的完整 TF 变换链。
 *
 * @param tf_buffer 全局 TF Buffer 的共享指针。
 * @param logger 用于输出日志的 rclcpp::Logger 对象。
 * @param target_frame 目标坐标系（通常是末端执行器链接）。
 * @param source_frame 源坐标系（通常是 "world"）。
 */
void printTfChain(
    const std::shared_ptr<tf2_ros::Buffer>& tf_buffer,
    const rclcpp::Logger& logger,
    const std::string& target_frame,
    const std::string& source_frame = "world")
{
    RCLCPP_INFO(logger, "\n--- 从 '%s' 到 '%s' 的变换链 ---", source_frame.c_str(), target_frame.c_str());

    // 尝试获取变换以确保链存在
    try {
        if (tf_buffer->canTransform(target_frame, source_frame, tf2::TimePointZero)) {
            RCLCPP_INFO(logger, "✅ 变换存在。正在解析坐标系链...");
        } else {
            RCLCPP_WARN(logger, "⚠️  canTransform 返回 false，但仍尝试解析链。");
        }
    } catch (const tf2::TransformException &ex) {
        RCLCPP_ERROR(logger, "❌ 无法验证变换: %s", ex.what());
        return; // 如果无法验证，直接返回，不进行后续遍历
    }

    // --- 核心逻辑：从 target_frame 向上遍历到 root ---
    std::vector<std::string> transform_chain;
    std::string current_frame = target_frame;
    std::set<std::string> visited_frames; // 防止无限循环

    // 首先将目标帧加入链
    transform_chain.push_back(current_frame);

    // 向上遍历直到找到 source_frame 或到达根节点
    while (current_frame != source_frame) {
        if (visited_frames.find(current_frame) != visited_frames.end()) {
            RCLCPP_ERROR(logger, "❌ 检测到循环依赖！在 '%s' 处停止。", current_frame.c_str());
            break;
        }
        visited_frames.insert(current_frame);

        try {
            // 使用修正后的 _getParent API
            std::string parent_frame;
            if (tf_buffer->_getParent(current_frame, tf2::TimePointZero, parent_frame)) {
                current_frame = parent_frame;
                transform_chain.push_back(current_frame);
            } else {
                RCLCPP_WARN(logger, "⚠️  在 '%s' 处到达根节点，但未找到源坐标系 '%s'。",
                            transform_chain.back().c_str(), source_frame.c_str());
                break;
            }
        } catch (const std::exception &e) {
            RCLCPP_ERROR(logger, "❌ 在遍历过程中出错: %s", e.what());
            break;
        }
    }

    // 打印结果
    if (!transform_chain.empty()) {
        RCLCPP_INFO(logger, "坐标系链 (从末端到世界):");
        for (size_t i = 0; i < transform_chain.size(); ++i) {
            // 为了更直观，我们可以反转链并用箭头连接
            if (i == 0) {
                RCLCPP_INFO(logger, "  %s", transform_chain[i].c_str());
            } else {
                RCLCPP_INFO(logger, "  ↑ (来自 %s)", transform_chain[i].c_str());
            }
        }

        // 打印正向链 (world -> ... -> ee_link)
        std::string forward_chain = source_frame;
        bool chain_complete = (transform_chain.back() == source_frame);
        if (chain_complete) {
            for (auto it = transform_chain.rbegin() + 1; it != transform_chain.rend(); ++it) {
                forward_chain += " -> " + *it;
            }
            RCLCPP_INFO(logger, "正向完整链: %s", forward_chain.c_str());
        } else {
            RCLCPP_WARN(logger, "⚠️  未能构建到 '%s' 的完整正向链。", source_frame.c_str());
        }
    } else {
        RCLCPP_ERROR(logger, "❌ 未能构建任何坐标系链。");
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
 * @brief 删除指定的碰撞体
 * 
 * 从规划场景中移除指定ID的碰撞体。
 */
bool deleteCollisionObject(const rclcpp::Node::SharedPtr& node, const std::string& object_id)
{
    RCLCPP_INFO(LOGGER, ">>> 正在删除碰撞体 '%s' ...", object_id.c_str());
    
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    
    // 创建要删除的碰撞体对象（只需要ID和REMOVE操作）
    moveit_msgs::msg::CollisionObject co;
    co.id = object_id;
    co.operation = moveit_msgs::msg::CollisionObject::REMOVE;
    
    // 应用删除操作
    planning_scene_interface.applyCollisionObjects({co});
    rclcpp::sleep_for(std::chrono::milliseconds(1000)); // 等待 RViz 更新
    
    RCLCPP_INFO(LOGGER, ">>> 碰撞体 '%s' 已成功删除。", object_id.c_str());
    return true;
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
 * @brief 控制指定机械臂的夹爪开合。
 *
 * 此函数通过向 GripperActionController 发送 GripperCommand 动作目标来直接控制夹爪，
 * 而不经过 MoveIt 的复杂规划流程。这是一种更高效、更标准的控制夹爪的方式。
 * 它会根据 SRDF 中定义的 "open" 和 "close" 状态来计算对应的目标位置。
 *
 * @param node ROS 2 节点的共享指针。
 * @param arm_id 机械臂 ID ("left" 或 "right")，用于确定控制哪个夹爪。
 * @param open_ratio 夹爪开合比例 (0.0 = 完全闭合, 1.0 = 完全张开)。
 * @return int 0 表示成功发送目标，-1 表示失败。
 */
int control_gripper(const rclcpp::Node::SharedPtr& node, const std::string& arm_id, double open_ratio)
{
    // 输入验证
    if (arm_id != "left" && arm_id != "right") {
        RCLCPP_ERROR(LOGGER, "无效的机械臂 ID: %s。必须是 'left' 或 'right'。", arm_id.c_str());
        return -1;
    }

    if (open_ratio < 0.0 || open_ratio > 1.0) {
        RCLCPP_WARN(LOGGER, "夹爪开合比例 %.2f 超出范围 [0.0, 1.0]，将被钳制到有效范围。", open_ratio);
        open_ratio = std::max(0.0, std::min(1.0, open_ratio));
    }

    // 1. 创建一个临时的 MoveGroupInterface 来从 SRDF 获取开合极限值
    // 这里复用了 printCurrentJointStates 函数第二部分的核心逻辑：创建 RobotState 并从中获取信息
    std::string gripper_group_name = arm_id + "_gripper";
    auto temp_move_group = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node, gripper_group_name);
    
    // 从 SRDF 中获取完全张开和完全闭合时的关节值
    std::map<std::string, double> open_joint_values = temp_move_group->getNamedTargetValues("open");
    std::map<std::string, double> close_joint_values = temp_move_group->getNamedTargetValues("close");

    if (open_joint_values.empty() || close_joint_values.empty()) {
        RCLCPP_ERROR(LOGGER, "无法从 SRDF 获取 '%s' 夹爪的 'open' 或 'close' 状态。请检查 dual_arm.srdf 文件。", gripper_group_name.c_str());
        return -1;
    }

    // 找到主控关节并计算目标位置
    std::string main_joint_name = open_joint_values.begin()->first;
    double open_value = open_joint_values[main_joint_name];
    double close_value = close_joint_values[main_joint_name];
    double target_position = close_value + open_ratio * (open_value - close_value);

    RCLCPP_INFO(LOGGER, "[control_gripper] 为 %s 夹爪设置目标位置: %.4f (开合比例: %.2f)", 
                gripper_group_name.c_str(), target_position, open_ratio);

    // 2. 创建 GripperCommand 动作客户端
    std::string action_name = "/" + arm_id + "_gripper_controller/gripper_cmd";
    auto action_client = rclcpp_action::create_client<GripperCommand>(node, action_name);
    
    if (!action_client->wait_for_action_server(std::chrono::seconds(5))) {
        RCLCPP_ERROR(LOGGER, "夹爪动作服务器 %s 未响应！", action_name.c_str());
        return -1;
    }

    // 3. 构建并发送动作目标
    auto goal_msg = GripperCommand::Goal();
    goal_msg.command.position = target_position;
    // 可以根据需要调整 max_effort，0.0 表示使用默认值
    goal_msg.command.max_effort = 0.0; 

    auto send_goal_options = rclcpp_action::Client<GripperCommand>::SendGoalOptions();
    send_goal_options.result_callback = [node, gripper_group_name](const auto & result) {
        switch (result.code) {
            case rclcpp_action::ResultCode::SUCCEEDED:
                RCLCPP_INFO(node->get_logger(), "%s 夹爪动作执行成功！", gripper_group_name.c_str());
                break;
            case rclcpp_action::ResultCode::ABORTED:
                RCLCPP_ERROR(node->get_logger(), "%s 夹爪动作被中止！", gripper_group_name.c_str());
                break;
            case rclcpp_action::ResultCode::CANCELED:
                RCLCPP_ERROR(node->get_logger(), "%s 夹爪动作被取消！", gripper_group_name.c_str());
                break;
            default:
                RCLCPP_ERROR(node->get_logger(), "%s 夹爪动作返回未知结果码！", gripper_group_name.c_str());
                break;
        }
    };

    action_client->async_send_goal(goal_msg, send_goal_options);
    RCLCPP_INFO(LOGGER, "%s 夹爪命令已发送至控制器。", gripper_group_name.c_str());
    
    return 0;
}

/**
 * @brief 通过指定规划组末端的XYZ轴偏移距离来完成直线规划控制
 * 
 * 此函数使用从 /joint_states 获取的真实关节值来计算当前末端位姿，
 * 确保了笛卡尔路径规划的起点与机器人实际状态完全一致。
 * 
 * @param node ROS 2节点指针
 * @param arm_id 机械臂ID ("left" 或 "right")
 * @param x_offset X轴偏移距离（米）
 * @param y_offset Y轴偏移距离（米）
 * @param z_offset Z轴偏移距离（米）
 * @param eef_step 末端执行器步长（默认0.001米）
 * @param jump_threshold 跳跃阈值（默认0.0，表示禁用跳跃检测）
 * @param speed 速度缩放因子（默认0.3）
 * @param print_poses 是否打印规划前后的末端位姿（默认true）
 * @param visualize 是否在RViz中可视化规划结果（默认false）
 * @return int 0表示成功，-1表示失败
 */
int move_cartesian_offset(
    const rclcpp::Node::SharedPtr& node, 
    const std::string& arm_id,
    double x_offset,
    double y_offset,
    double z_offset,
    double eef_step = 0.001,
    double jump_threshold = 0.0,
    double speed = 0.3,
    bool print_poses = true,
    bool visualize = false)
{
    RCLCPP_INFO(LOGGER, "[move_cartesian_offset] 开始为机械臂 %s 执行笛卡尔路径规划", arm_id.c_str());
    RCLCPP_INFO(LOGGER, "偏移量: X=%.4f, Y=%.4f, Z=%.4f", x_offset, y_offset, z_offset);

    // 创建MoveGroupInterface
    auto move_group = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node, arm_id);
    
    // 设置速度和加速度缩放因子
    move_group->setMaxVelocityScalingFactor(speed);
    move_group->setMaxAccelerationScalingFactor(speed);
    
    // === 关键修改：使用真实关节值获取当前末端位姿 ===
    // 1. 创建一个临时节点用于订阅 /joint_states
    auto fetcher_node = rclcpp::Node::make_shared("cartesian_fetcher_" + arm_id);
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

    const std::vector<std::string>& joint_names = move_group->getActiveJoints();
    auto start_time = fetcher_node->now();
    while (rclcpp::ok() && (fetcher_node->now() - start_time).seconds() < 2.0) {
        rclcpp::spin_some(fetcher_node);
        if (latest_state) break;
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    if (!latest_state) {
        RCLCPP_ERROR(LOGGER, "超时未收到 /joint_states 数据！");
        return -1;
    }

    // 2. 提取所需关节的真实值
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

    // 3. 使用真实关节值创建 RobotState 并计算 FK
    auto robot_model = move_group->getRobotModel();
    moveit::core::RobotStatePtr current_state(new moveit::core::RobotState(robot_model));
    current_state->setToDefaultValues();
    current_state->setJointGroupPositions(arm_id, joint_values);
    current_state->update(); // 触发 FK 计算

    std::string ee_link = (arm_id == "left") ? "left_ee_link" : "right_ee_link";
    const Eigen::Isometry3d& end_effector_state = current_state->getGlobalLinkTransform(ee_link);

    // 将 Eigen::Isometry3d 转换为 geometry_msgs::msg::Pose
    geometry_msgs::msg::Pose current_pose_msg;
    current_pose_msg.position.x = end_effector_state.translation().x();
    current_pose_msg.position.y = end_effector_state.translation().y();
    current_pose_msg.position.z = end_effector_state.translation().z();
    Eigen::Quaterniond quat(end_effector_state.rotation());
    current_pose_msg.orientation.x = quat.x();
    current_pose_msg.orientation.y = quat.y();
    current_pose_msg.orientation.z = quat.z();
    current_pose_msg.orientation.w = quat.w();

    if (print_poses) {
        RCLCPP_INFO(LOGGER, "=== 规划前末端位姿 (%s) ===", ee_link.c_str());
        RCLCPP_INFO(LOGGER, "位置 [x, y, z]: (%.4f, %.4f, %.4f)",
                    current_pose_msg.position.x, current_pose_msg.position.y, current_pose_msg.position.z);
        RCLCPP_INFO(LOGGER, "四元数 [x, y, z, w]: (%.4f, %.4f, %.4f, %.4f)",
                    current_pose_msg.orientation.x, current_pose_msg.orientation.y,
                    current_pose_msg.orientation.z, current_pose_msg.orientation.w);
    }
    
    // 计算目标位姿（在当前位姿基础上加上偏移量）
    geometry_msgs::msg::Pose target_pose = current_pose_msg;
    target_pose.position.x += x_offset;
    target_pose.position.y += y_offset;
    target_pose.position.z += z_offset;
    
    // 创建路点列表（只有一个目标点）
    std::vector<geometry_msgs::msg::Pose> waypoints;
    waypoints.push_back(target_pose);
    
    // 执行笛卡尔路径规划
    moveit_msgs::msg::RobotTrajectory robot_trajectory;
    double fraction = move_group->computeCartesianPath(
        waypoints,           // 路点列表
        eef_step,           // 末端执行器步长
        jump_threshold,     // 跳跃阈值
        robot_trajectory,   // 输出轨迹 (moveit_msgs::msg::RobotTrajectory)
        true               // 避免碰撞
    );
    
    if (fraction < 1.0) {
        RCLCPP_WARN(LOGGER, "笛卡尔路径规划未完全成功，完成度: %.2f%%", fraction * 100.0);
        if (fraction <= 0.0) {
            RCLCPP_ERROR(LOGGER, "笛卡尔路径规划完全失败！");
            return -1;
        }
    } else {
        RCLCPP_INFO(LOGGER, "笛卡尔路径规划成功完成 100%%");
    }
    
    // 检查轨迹是否有效
    if (robot_trajectory.joint_trajectory.points.empty()) {
        RCLCPP_ERROR(LOGGER, "生成的轨迹为空！");
        return -1;
    }
    
    if (print_poses) {
        RCLCPP_INFO(LOGGER, "=== 规划后末端位姿 (%s) ===", ee_link.c_str());
        RCLCPP_INFO(LOGGER, "位置 [x, y, z]: (%.4f, %.4f, %.4f)",
                    target_pose.position.x, target_pose.position.y, target_pose.position.z);
        RCLCPP_INFO(LOGGER, "四元数 [x, y, z, w]: (%.4f, %.4f, %.4f, %.4f)",
                    target_pose.orientation.x, target_pose.orientation.y,
                    target_pose.orientation.z, target_pose.orientation.w);
    }

    // === 新增：可视化规划结果 ===
    if (visualize) {
        RCLCPP_INFO(LOGGER, "正在RViz中可视化笛卡尔路径规划结果...");
        namespace rvt = rviz_visual_tools;

        // 创建 MoveItVisualTools 对象
        moveit_visual_tools::MoveItVisualTools visual_tools(
            node, "world", "cartesian_path_visualization", move_group->getRobotModel()
        );
        visual_tools.deleteAllMarkers();
        visual_tools.loadRemoteControl();

        // 注意：publishAxisLabeled 需要 geometry_msgs::msg::Pose，而不是 PoseStamped
        // 直接使用 current_pose_msg 和 target_pose
        visual_tools.publishAxisLabeled(current_pose_msg, "Start");
        visual_tools.publishAxisLabeled(target_pose, "Goal");

        // 发布用户定义的笛卡尔路径 (waypoints)
        visual_tools.publishPath(waypoints, rvt::LIME_GREEN, rvt::SMALL);

        // 发布 MoveIt 实际规划出的轨迹
        const moveit::core::JointModelGroup* joint_model_group = 
            move_group->getRobotModel()->getJointModelGroup(arm_id);
        visual_tools.publishTrajectoryLine(robot_trajectory, joint_model_group);

        // 触发RViz更新
        visual_tools.trigger();

        RCLCPP_INFO(LOGGER, "可视化完成。检查RViz以查看结果。");
    }
    
    // 发送轨迹到控制器执行
    std::string controller_name = (arm_id.find("left") != std::string::npos) 
        ? "/left_controller/follow_joint_trajectory" 
        : "/right_controller/follow_joint_trajectory";
    
    auto action_client = rclcpp_action::create_client<FollowJointTrajectory>(node, controller_name);
    if (!action_client->wait_for_action_server(std::chrono::seconds(10))) {
        RCLCPP_ERROR(LOGGER, "动作服务器 %s 未响应！", controller_name.c_str());
        return -1;
    }
    
    FollowJointTrajectory::Goal goal_msg;
    goal_msg.trajectory = robot_trajectory.joint_trajectory;
    
    auto send_goal_options = rclcpp_action::Client<FollowJointTrajectory>::SendGoalOptions();
    send_goal_options.result_callback = [node](const auto & result) {
        switch (result.code) {
            case rclcpp_action::ResultCode::SUCCEEDED:
                RCLCPP_INFO(LOGGER, "笛卡尔轨迹执行成功！");
                break;
            case rclcpp_action::ResultCode::ABORTED:
                RCLCPP_ERROR(LOGGER, "笛卡尔轨迹执行被中止！");
                break;
            case rclcpp_action::ResultCode::CANCELED:
                RCLCPP_ERROR(LOGGER, "笛卡尔轨迹执行被取消！");
                break;
            default:
                RCLCPP_ERROR(LOGGER, "未知的结果码！");
                break;
        }
    };
    
    action_client->async_send_goal(goal_msg, send_goal_options);
    RCLCPP_INFO(LOGGER, "笛卡尔轨迹已发送至控制器，正在执行...");
    
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

    // === 关键步骤：加载 YAML 配置 ===
    // 确保你的 launch 文件正确加载了 pick_object.yaml 到这个节点的命名空间
    // 例如: parameters=[{'file_path_to_pick_object.yaml'}]
    RCLCPP_INFO(LOGGER, "正在从参数服务器加载 YAML 配置...");
    // 参数会在 load_collision_object_params 中被声明和读取

    // === 创建全局的 TF Buffer 和 Listener ===
    // 这必须在 executor 启动前完成，以确保 listener 能接收到消息。
    auto tf_buffer = std::make_shared<tf2_ros::Buffer>(move_group_node->get_clock());
    auto tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer, move_group_node);


    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(move_group_node);
    std::thread([&executor]() { executor.spin(); }).detach();    
    // 等待系统完全启动
    rclcpp::sleep_for(std::chrono::seconds(5));

    const std::string LEFT_ARM_ID = "left";
    const std::string RIGHT_ARM_ID = "right";

    // === 打印初始状态 ===
    // 修改调用，传入全局的 tf_buffer
    printCurrentJointStates(move_group_node, LEFT_ARM_ID, tf_buffer);
    printCurrentJointStates(move_group_node, RIGHT_ARM_ID, tf_buffer);

    // // === 打印 TF 坐标系变换链 ===
    // RCLCPP_INFO(LOGGER, "=== 打印完整的 TF 坐标系变换链 ===");
    // RCLCPP_INFO(LOGGER, "所有已知坐标系:\n%s", tf_buffer->allFramesAsString().c_str());
    // // 定义要查询的末端执行器链接
    // std::vector<std::string> ee_links = {"left_ee_link", "right_ee_link"};
    // std::string source_frame = "world";
    // // 为每个末端执行器链接调用新函数
    // for (const auto& target_frame : ee_links) {
    //     printTfChain(tf_buffer, LOGGER, target_frame, source_frame);
    // }




    // === 步骤 0: 创建并添加 target_plate 作为世界物体 ===
    RCLCPP_INFO(LOGGER, ">>> 正在创建并添加世界物体 ...");
    moveit_msgs::msg::CollisionObject co;
    if (!create_target_plate_collision_object(move_group_node, co)) {
        RCLCPP_ERROR(LOGGER, "创建世界物体失败！");
        return EXIT_FAILURE;
    }

    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    planning_scene_interface.applyCollisionObjects({co});
    rclcpp::sleep_for(std::chrono::seconds(2)); // 等待 RViz 更新
    RCLCPP_INFO(LOGGER, ">>> 世界物体 '%s' 添加成功。", co.id.c_str());

    // === 移动到预抓取位置 ===
    RCLCPP_INFO(LOGGER, ">>> 移动 %s 臂到预抓取位姿...", LEFT_ARM_ID.c_str());
    if (to_srdf_NamedTarget(move_group_node, LEFT_ARM_ID, "left_pre_grasp", 0.3) != 0) {
        RCLCPP_WARN(LOGGER, "%s 臂移动到预抓取位姿失败!", LEFT_ARM_ID.c_str());
    }
    rclcpp::sleep_for(std::chrono::seconds(2));
    RCLCPP_INFO(LOGGER, ">>> 移动 %s 臂到预抓取位姿...", RIGHT_ARM_ID.c_str());
    if (to_srdf_NamedTarget(move_group_node, RIGHT_ARM_ID, "right_pre_grasp", 0.3) != 0) {
        RCLCPP_WARN(LOGGER, "%s 臂移动到预抓取位姿失败!", RIGHT_ARM_ID.c_str());
    }
    rclcpp::sleep_for(std::chrono::seconds(5));
    // === 测试笛卡尔路径规划 ===
    RCLCPP_INFO(LOGGER, ">>> 测试 %s 臂笛卡尔路径规划：沿Z轴向上移动5cm", LEFT_ARM_ID.c_str());
    if (move_cartesian_offset(move_group_node, LEFT_ARM_ID, 0.0, 0.0, -0.1, 0.001, 0.0, 0.3, true, true) != 0) {
        RCLCPP_WARN(LOGGER, "%s 臂笛卡尔路径规划执行失败", LEFT_ARM_ID.c_str());
    } else {
        rclcpp::sleep_for(std::chrono::seconds(2));
    }

    rclcpp::sleep_for(std::chrono::seconds(4));
    RCLCPP_INFO(LOGGER, ">>> 打开 %s 夹爪...", LEFT_ARM_ID.c_str());
    if (control_gripper(move_group_node, LEFT_ARM_ID, 1.0) != 0) {
        RCLCPP_ERROR(LOGGER, "打开 %s 夹爪失败！", LEFT_ARM_ID.c_str());
    } else {
        rclcpp::sleep_for(std::chrono::seconds(2)); // 等待动作完成
    }
    // === 步骤 1: 附着物体 ===
    rclcpp::sleep_for(std::chrono::seconds(2));
    RCLCPP_INFO(LOGGER, "尝试附着物体 '%s' 到 %s 臂...", co.id.c_str(), LEFT_ARM_ID.c_str());
    if (!attach_target_plate(move_group_node, LEFT_ARM_ID, co.id)) {
        RCLCPP_ERROR(LOGGER, "附着失败！");
        return EXIT_FAILURE;
    }
    rclcpp::sleep_for(std::chrono::seconds(2));

    // // === 步骤 2: 携带物体移动 ===
    // RCLCPP_INFO(LOGGER, "移动 %s 臂（携带 '%s'）到新位置...", LEFT_ARM_ID.c_str(), co.id.c_str());
    // std::vector<double> carry_pose_deg = {80.0, -40.0, -90.0, -20.0, 90.0, 0.0};
    // if (to_joint_config(move_group_node, LEFT_ARM_ID, carry_pose_deg, 0.3) != 0) {
    //     RCLCPP_WARN(LOGGER, "携带平板移动失败");
    // }
    // rclcpp::sleep_for(std::chrono::seconds(3));

    // // === 步骤 3: 分离物体 ===
    // RCLCPP_INFO(LOGGER, "分离物体 '%s'...", co.id.c_str());
    // if (!detach_target_plate(move_group_node, LEFT_ARM_ID, co.id)) {
    //     RCLCPP_WARN(LOGGER, "分离失败！");
    // }
    // rclcpp::sleep_for(std::chrono::seconds(1));


    rclcpp::sleep_for(std::chrono::seconds(5));
    RCLCPP_INFO(LOGGER, ">>> 移动 %s 臂到预抓取位姿...", LEFT_ARM_ID.c_str());
    if (to_srdf_NamedTarget(move_group_node, LEFT_ARM_ID, "left_pose1", 0.3) != 0) {
        RCLCPP_WARN(LOGGER, "%s 臂移动到预抓取位姿失败!", LEFT_ARM_ID.c_str());
    }
    // === 新增：控制夹爪 ===
    // 1. 打开夹爪 (开合比例 1.0)
    rclcpp::sleep_for(std::chrono::seconds(4));
    RCLCPP_INFO(LOGGER, ">>> 打开 %s 夹爪...", LEFT_ARM_ID.c_str());
    if (control_gripper(move_group_node, LEFT_ARM_ID, 1.0) != 0) {
        RCLCPP_ERROR(LOGGER, "打开 %s 夹爪失败！", LEFT_ARM_ID.c_str());
    } else {
        rclcpp::sleep_for(std::chrono::seconds(2)); // 等待动作完成
    }
    rclcpp::sleep_for(std::chrono::seconds(2));
    // 2. 关闭夹爪 (开合比例 0.0)
    RCLCPP_INFO(LOGGER, ">>> 关闭 %s 夹爪...", LEFT_ARM_ID.c_str());
    if (control_gripper(move_group_node, LEFT_ARM_ID, 0.0) != 0) {
        RCLCPP_ERROR(LOGGER, "关闭 %s 夹爪失败！", LEFT_ARM_ID.c_str());
    } else {
        rclcpp::sleep_for(std::chrono::seconds(2)); // 等待动作完成
    }
    rclcpp::sleep_for(std::chrono::seconds(2));
    // 3. 将夹爪设置为半开状态 (开合比例 0.5)
    RCLCPP_INFO(LOGGER, ">>> 将 %s 夹爪设置为半开状态...", LEFT_ARM_ID.c_str());
    if (control_gripper(move_group_node, LEFT_ARM_ID, 0.5) != 0) {
        RCLCPP_ERROR(LOGGER, "设置 %s 夹爪半开状态失败！", LEFT_ARM_ID.c_str());
    } else {
        rclcpp::sleep_for(std::chrono::seconds(2)); // 等待动作完成
    }
    // === 步骤 3: 分离物体 ===
    rclcpp::sleep_for(std::chrono::seconds(5));
    RCLCPP_INFO(LOGGER, "分离物体 '%s'...", co.id.c_str());
    if (!detach_target_plate(move_group_node, LEFT_ARM_ID, co.id)) {
        RCLCPP_WARN(LOGGER, "分离失败！");
    }
    rclcpp::sleep_for(std::chrono::seconds(1));


// === 步骤 3: 分离物体 ===
rclcpp::sleep_for(std::chrono::seconds(5));


// === 新增：删除碰撞体 ===
RCLCPP_INFO(LOGGER, ">>> 删除碰撞体 '%s' ...", co.id.c_str());
if (!deleteCollisionObject(move_group_node, co.id)) {
    RCLCPP_WARN(LOGGER, "删除碰撞体失败！");
}
rclcpp::sleep_for(std::chrono::seconds(2));

// === 新增：创建下落动画效果（模拟物体逐步下落到桌面）===
RCLCPP_INFO(LOGGER, ">>> 创建碰撞体 '%s' 下落动画到桌面...", co.id.c_str());

// 获取原始碰撞体参数用于动画
std::string object_id, frame_id, shape;
std::vector<double> dimensions;
geometry_msgs::msg::Pose original_pose;
if (!load_collision_object_params(move_group_node, object_id, frame_id, shape, dimensions, original_pose)) {
    RCLCPP_ERROR(LOGGER, "无法加载碰撞体参数，使用默认值");
    // 使用默认值
    dimensions = {0.2, 0.2, 0.05}; // 长 x 宽 x 高
    original_pose.position.z = 1.0; // 假设起始高度为1.0m
}

// 设置下落参数
double start_z = original_pose.position.z;
double end_z = 0.775; // 桌面高度 + 物体半厚 (0.75 + 0.025)
int animation_steps = 20; // 动画步数
double step_delay_ms = 50.0; // 每步延迟毫秒

// 确保起始位置高于结束位置
if (start_z <= end_z) {
    start_z = end_z + 0.1; // 添加小偏移量
}

// 执行下落动画
moveit::planning_interface::PlanningSceneInterface planning_scene_interface_falling;
for (int i = 0; i <= animation_steps; ++i) {
    moveit_msgs::msg::CollisionObject falling_co;
    falling_co.id = co.id;
    falling_co.header.frame_id = "world";
    falling_co.operation = moveit_msgs::msg::CollisionObject::ADD;
    
    // 创建几何形状
    shape_msgs::msg::SolidPrimitive box;
    box.type = box.BOX;
    box.dimensions.resize(3);
    box.dimensions[box.BOX_X] = dimensions[0];
    box.dimensions[box.BOX_Y] = dimensions[1];
    box.dimensions[box.BOX_Z] = dimensions[2];
    falling_co.primitives.push_back(box);
    
    // 计算当前Z位置
    double current_z = start_z - (start_z - end_z) * static_cast<double>(i) / animation_steps;
    
    // 设置当前位置
    geometry_msgs::msg::Pose current_pose;
    current_pose.position.x = original_pose.position.x; // 保持X,Y不变
    current_pose.position.y = original_pose.position.y;
    current_pose.position.z = current_z;
    current_pose.orientation = original_pose.orientation; // 保持朝向不变
    
    falling_co.primitive_poses.push_back(current_pose);
    
    // 应用当前帧
    planning_scene_interface_falling.applyCollisionObjects({falling_co});
    
    // 短暂延迟以创建动画效果
    rclcpp::sleep_for(std::chrono::milliseconds(static_cast<int>(step_delay_ms)));
}

RCLCPP_INFO(LOGGER, ">>> 碰撞体 '%s' 下落动画完成，位于桌面高度 z=%.3f", co.id.c_str(), end_z);
    
    RCLCPP_INFO(LOGGER, "所有测试完成。按 CTRL+C 退出。");
    
    rclcpp::Rate rate(0.5);
    while (rclcpp::ok()) { 
        rate.sleep(); 
    }
    
    rclcpp::shutdown();
    return EXIT_SUCCESS;
}