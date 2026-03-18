#include "transform_listener.hpp"

TransformListener::TransformListener()
    : Node("transform_listener"),
      tf_buffer_(std::make_unique<tf2_ros::Buffer>(this->get_clock())),
      tf_listener_(std::make_shared<tf2_ros::TransformListener>(*tf_buffer_)) {
  // Call on_timer function every second
  timer_ = this->create_wall_timer(
      1s, std::bind(&TransformListener::on_timer, this));
}

void TransformListener::on_timer() {
  try {
    geometry_msgs::msg::TransformStamped transformStamped =
        tf_buffer_->lookupTransform("left_ee_link", "left_camera_optical_link", tf2::TimePointZero);

    RCLCPP_INFO(this->get_logger(),
                "left_Translation: x=%.6f, y=%.6f, z=%.6f",
                transformStamped.transform.translation.x,
                transformStamped.transform.translation.y,
                transformStamped.transform.translation.z);
    RCLCPP_INFO(this->get_logger(),
                "left_Rotation: qx=%.6f, qy=%.6f, qz=%.6f, qw=%.6f",
                transformStamped.transform.rotation.x,
                transformStamped.transform.rotation.y,
                transformStamped.transform.rotation.z,
                transformStamped.transform.rotation.w);
  } catch (tf2::TransformException &ex) {
    RCLCPP_WARN(this->get_logger(), "Could not transform left_ee_link to left_camera_link: %s", ex.what());
  }

  try {
    geometry_msgs::msg::TransformStamped transformStamped =
        tf_buffer_->lookupTransform("right_ee_link", "right_camera_optical_link", tf2::TimePointZero);

    RCLCPP_INFO(this->get_logger(),
                "right_Translation: x=%.6f, y=%.6f, z=%.6f",
                transformStamped.transform.translation.x,
                transformStamped.transform.translation.y,
                transformStamped.transform.translation.z);
    RCLCPP_INFO(this->get_logger(),
                "right_Rotation: qx=%.6f, qy=%.6f, qz=%.6f, qw=%.6f",
                transformStamped.transform.rotation.x,
                transformStamped.transform.rotation.y,
                transformStamped.transform.rotation.z,
                transformStamped.transform.rotation.w);
    } catch (tf2::TransformException &ex) {
    RCLCPP_WARN(this->get_logger(), "Could not transform left_ee_link to left_camera_link: %s", ex.what());
    }

}

