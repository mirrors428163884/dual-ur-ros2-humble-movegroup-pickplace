#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit_servo/servo_parameters.h>
#include <moveit_servo/pose_tracking.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <thread>
#include <Eigen/Geometry> // 确保包含 Eigen

using namespace std::chrono_literals;

class DualArmFollower : public rclcpp::Node
{
public:
  DualArmFollower() : Node("dual_arm_follower")
  {
    RCLCPP_INFO(get_logger(), "DualArmFollower node created. Initializing...");
    
    // 1. 初始化不依赖 shared_from_this 的成员
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // 2. 创建发布器
    target_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
        "target_pose", rclcpp::SystemDefaultsQoS());

    // 3. 创建定时器 (注意：回调函数现在会检查是否已初始化)
    timer_ = this->create_wall_timer(20ms, [this]() { 
        if (is_initialized_) {
            publishTargetPose(); 
        }
    });

    RCLCPP_INFO(get_logger(), "Node base initialized. Call init() to start MoveIt components.");
  }

  // 新增：独立的初始化函数
  void init()
  {
    RCLCPP_INFO(get_logger(), "Starting MoveIt initialization...");

    // 现在可以安全地使用 shared_from_this() 了，因为对象已经完全构造并被 shared_ptr 持有
    auto shared_node = shared_from_this();

    // 1. 加载 Servo 参数
    auto servo_params = moveit_servo::ServoParameters::makeServoParameters(shared_node);
    if (!servo_params)
    {
      RCLCPP_FATAL(get_logger(), "Failed to load servo parameters!");
      // 可以选择 throw 或者让节点退出，这里选择 throw 以便 main 捕获或终止
      throw std::runtime_error("Failed to load servo parameters");
    }
    RCLCPP_INFO(get_logger(), "Servo parameters loaded.");

    // 2. 创建 PlanningSceneMonitor
    planning_scene_monitor_ = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>(
        shared_node, "robot_description");
    
    if (!planning_scene_monitor_->getPlanningScene())
    {
        RCLCPP_ERROR(get_logger(), "Failed to create PlanningSceneMonitor");
        throw std::runtime_error("Failed to create PlanningSceneMonitor");
    }

    planning_scene_monitor_->startStateMonitor(servo_params->joint_topic);
    planning_scene_monitor_->startSceneMonitor();
    planning_scene_monitor_->startWorldGeometryMonitor();
    RCLCPP_INFO(get_logger(), "PlanningSceneMonitor started.");

    // 3. 创建 PoseTracking 对象
    right_arm_tracker_ = std::make_unique<moveit_servo::PoseTracking>(
        shared_node, servo_params, planning_scene_monitor_);
    
    if (!right_arm_tracker_) {
        RCLCPP_FATAL(get_logger(), "Failed to create PoseTracking object");
        throw std::runtime_error("Failed to create PoseTracking");
    }
    RCLCPP_INFO(get_logger(), "PoseTracking object created.");

    // 4. 启动跟踪线程
    tracking_thread_ = std::thread([this, servo_params]() {
      Eigen::Vector3d lin_tol{0.001, 0.001, 0.001};
      double rot_tol = 0.01;
      RCLCPP_INFO(get_logger(), "Starting moveToPose thread...");
      // 注意：moveToPose 是阻塞的，直到到达目标或超时
      right_arm_tracker_->moveToPose(lin_tol, rot_tol, 0.1);
    });

    is_initialized_ = true;
    RCLCPP_INFO(get_logger(), "MoveIt initialization complete!");
  }

  ~DualArmFollower()
  {
    if (tracking_thread_.joinable())
      tracking_thread_.join();
  }

private:
  void publishTargetPose()
  {
    try
    {
      // 获取左臂 EE 在 world 坐标系下的位姿
      // 注意：确保 "left_ee_link" 在你的 URDF 中真实存在
      auto transform = tf_buffer_->lookupTransform(
          "world", "left_ee_link", tf2::TimePointZero);

      geometry_msgs::msg::PoseStamped target_pose;
      target_pose.header.frame_id = "world";
      target_pose.header.stamp = this->now();
      target_pose.pose.position.x = transform.transform.translation.x;
      target_pose.pose.position.y = transform.transform.translation.y;
      target_pose.pose.position.z = transform.transform.translation.z;
      target_pose.pose.orientation = transform.transform.rotation;

      target_pose_pub_->publish(target_pose);
    }
    catch (const tf2::TransformException &ex)
    {
      // 降低日志频率以免刷屏
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, 
                          "TF lookup failed (left_ee_link not found?): %s", ex.what());
    }
  }

  // 成员变量
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr target_pose_pub_;
  
  planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor_;
  std::unique_ptr<moveit_servo::PoseTracking> right_arm_tracker_;
  std::thread tracking_thread_;
  
  bool is_initialized_ = false; // 标记是否已完成 MoveIt 初始化
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  
  // 必须使用 shared_ptr 创建节点，否则 shared_from_this() 永远无法工作
  auto node = std::make_shared<DualArmFollower>();
  
  try {
      // 显式调用初始化函数
      node->init();
      
      RCLCPP_INFO(node->get_logger(), "Spinning node...");
      rclcpp::spin(node);
  } catch (const std::exception& e) {
      RCLCPP_FATAL(node->get_logger(), "Initialization failed: %s", e.what());
      rclcpp::shutdown();
      return 1;
  }

  rclcpp::shutdown();
  return 0;
}