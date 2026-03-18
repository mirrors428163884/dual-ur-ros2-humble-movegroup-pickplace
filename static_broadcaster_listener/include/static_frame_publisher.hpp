#ifndef STATIC_FRAME_PUBLISHER_HPP_
#define STATIC_FRAME_PUBLISHER_HPP_

#include <memory>
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/static_transform_broadcaster.h"

class StaticFramePublisher : public rclcpp::Node {
public:
  explicit StaticFramePublisher();

private:
  void declare_parameters();
  void left_make_transforms();
  void right_make_transforms();
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_broadcaster_;
};

#endif // STATIC_FRAME_PUBLISHER_HPP_