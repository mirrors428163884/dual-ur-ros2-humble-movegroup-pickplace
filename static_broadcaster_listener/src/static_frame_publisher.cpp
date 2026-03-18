#include "static_frame_publisher.hpp"

StaticFramePublisher::StaticFramePublisher()
    : Node("static_turtle_tf2_broadcaster") {
  RCLCPP_INFO(this->get_logger(), "Starting static_turtle_tf2_broadcaster...");

  declare_parameters();
  // 创建静态变换广播器实例
  tf_static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

  // 发布静态变换一次，在启动时
  left_make_transforms();
  right_make_transforms();
}

void StaticFramePublisher::declare_parameters() 
{
      this->declare_parameter<std::string>("static_transforms.left.header.frame_id", "left_ee_link");
      this->declare_parameter<std::string>("static_transforms.left.child_frame_id", "left_camera_optical_link");
      this->declare_parameter<double>("static_transforms.left.transform.translation.x", 0.0);
      this->declare_parameter<double>("static_transforms.left.transform.translation.y", 0.0);
      this->declare_parameter<double>("static_transforms.left.transform.translation.z", 0.0);
      this->declare_parameter<double>("static_transforms.left.transform.rotation.x", 0.0);
      this->declare_parameter<double>("static_transforms.left.transform.rotation.y", 0.0);
      this->declare_parameter<double>("static_transforms.left.transform.rotation.z", 0.0);
      this->declare_parameter<double>("static_transforms.left.transform.rotation.w", 0.0);

      this->declare_parameter<std::string>("static_transforms.right.header.frame_id", "right_ee_link");
      this->declare_parameter<std::string>("static_transforms.right.child_frame_id", "right_camera_optical_link");
      this->declare_parameter<double>("static_transforms.right.transform.translation.x", 0.0);
      this->declare_parameter<double>("static_transforms.right.transform.translation.y", 0.0);
      this->declare_parameter<double>("static_transforms.right.transform.translation.z", 0.0);
      this->declare_parameter<double>("static_transforms.right.transform.rotation.x", 0.0);
      this->declare_parameter<double>("static_transforms.right.transform.rotation.y", 0.0);
      this->declare_parameter<double>("static_transforms.right.transform.rotation.z", 0.0);
      this->declare_parameter<double>("static_transforms.right.transform.rotation.w", 0.0);
}

void StaticFramePublisher::left_make_transforms() 
{
  geometry_msgs::msg::TransformStamped t;

  // 获取参数值
  auto params = this->get_parameters({
      "static_transforms.left.header.frame_id",
      "static_transforms.left.child_frame_id",
      "static_transforms.left.transform.translation.x",
      "static_transforms.left.transform.translation.y",
      "static_transforms.left.transform.translation.z",
      "static_transforms.left.transform.rotation.x",
      "static_transforms.left.transform.rotation.y",
      "static_transforms.left.transform.rotation.z",
      "static_transforms.left.transform.rotation.w"
  });

  t.header.frame_id = params[0].as_string();
  t.child_frame_id = params[1].as_string();
  t.transform.translation.x = params[2].as_double();
  t.transform.translation.y = params[3].as_double();
  t.transform.translation.z = params[4].as_double();
  t.transform.rotation.x = params[5].as_double();
  t.transform.rotation.y = params[6].as_double();
  t.transform.rotation.z = params[7].as_double();
  t.transform.rotation.w = params[8].as_double();

  t.header.stamp = this->get_clock()->now();

  try {
    tf_static_broadcaster_->sendTransform(t);
    RCLCPP_INFO(this->get_logger(), "Broadcasted static transform from %s to %s",
                t.header.frame_id.c_str(), t.child_frame_id.c_str());
    
    RCLCPP_INFO(this->get_logger(),
                "left_Translation: x=%.6f, y=%.6f, z=%.6f",
                t.transform.translation.x,
                t.transform.translation.y,
                t.transform.translation.z);
    RCLCPP_INFO(this->get_logger(),
                "left_Rotation: qx=%.6f, qy=%.6f, qz=%.6f, qw=%.6f",
                t.transform.rotation.x,
                t.transform.rotation.y,
                t.transform.rotation.z,
                t.transform.rotation.w);
  } catch (const std::exception& e) {
    RCLCPP_ERROR(this->get_logger(), "Failed to broadcast static transform: %s", e.what());
  }
}

void StaticFramePublisher::right_make_transforms() 
{
  geometry_msgs::msg::TransformStamped t;

  // 获取参数值
  auto params = this->get_parameters({
      "static_transforms.right.header.frame_id",
      "static_transforms.right.child_frame_id",
      "static_transforms.right.transform.translation.x",
      "static_transforms.right.transform.translation.y",
      "static_transforms.right.transform.translation.z",
      "static_transforms.right.transform.rotation.x",
      "static_transforms.right.transform.rotation.y",
      "static_transforms.right.transform.rotation.z",
      "static_transforms.right.transform.rotation.w"
  });

  t.header.frame_id = params[0].as_string();
  t.child_frame_id = params[1].as_string();
  t.transform.translation.x = params[2].as_double();
  t.transform.translation.y = params[3].as_double();
  t.transform.translation.z = params[4].as_double();
  t.transform.rotation.x = params[5].as_double();
  t.transform.rotation.y = params[6].as_double();
  t.transform.rotation.z = params[7].as_double();
  t.transform.rotation.w = params[8].as_double();

  t.header.stamp = this->get_clock()->now();

  try {
    tf_static_broadcaster_->sendTransform(t);
    RCLCPP_INFO(this->get_logger(), "Broadcasted static transform from %s to %s",
                t.header.frame_id.c_str(), t.child_frame_id.c_str());

    RCLCPP_INFO(this->get_logger(),
                "right_Translation: x=%.6f, y=%.6f, z=%.6f",
                t.transform.translation.x,
                t.transform.translation.y,
                t.transform.translation.z);
    RCLCPP_INFO(this->get_logger(),
                "right_Rotation: qx=%.6f, qy=%.6f, qz=%.6f, qw=%.6f",
                t.transform.rotation.x,
                t.transform.rotation.y,
                t.transform.rotation.z,
                t.transform.rotation.w);
    
  } catch (const std::exception& e) {
    RCLCPP_ERROR(this->get_logger(), "Failed to broadcast static transform: %s", e.what());
  }
}

