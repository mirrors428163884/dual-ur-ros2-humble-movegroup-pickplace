#ifndef TRANSFORM_LISTENER_HPP_
#define TRANSFORM_LISTENER_HPP_

#include <memory>
#include <string>
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2/LinearMath/Quaternion.h"

using namespace std::chrono_literals;

class TransformListener : public rclcpp::Node {
public:
  explicit TransformListener();

private:
  void on_timer();
  rclcpp::TimerBase::SharedPtr timer_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

#endif // TRANSFORM_LISTENER_HPP_