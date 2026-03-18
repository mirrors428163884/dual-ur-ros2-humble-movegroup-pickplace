/*
这个程序用来每2秒打印joint 的state，关节值
*/
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <iostream>
#include <string>
#include <map>
#include <algorithm>
#include <chrono>

// ANSI 转义码用于彩色输出
#define RESET   "\033[0m"
#define MAGENTA "\033[35m"      /* Magenta */
#define BYELLOW "\033[1;33m"    /* Bold Yellow */
#define BGREEN  "\033[1;32m"    /* Bold Green */

class JointStatePrinter : public rclcpp::Node {
public:
  JointStatePrinter()
    : Node("JOINT_VALUE_Printer"),
      latest_joint_map_(),
      joint_print_timer_(nullptr),
      joint_state_sub_(nullptr) {
      
    // 订阅 /joint_states 话题
    joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
        "/joint_states", 10, std::bind(&JointStatePrinter::jointStateCallback, this, std::placeholders::_1));

    // 创建一个每两秒触发一次的定时器来打印关节状态
    joint_print_timer_ = this->create_wall_timer(
        std::chrono::seconds(2), 
        std::bind(&JointStatePrinter::printJointStates, this));
  }

private:
  void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg) {
    // 更新最新的关节值
    std::lock_guard<std::mutex> guard(joint_mutex_);
    latest_joint_map_.clear();
    for (size_t i = 0; i < msg->name.size(); ++i) {
      latest_joint_map_[msg->name[i]] = msg->position[i];
    }
  }

  void printJointStates() {
    // 打印关节名称和位置值
    std::map<std::string, double> joint_map;
    {
      std::lock_guard<std::mutex> guard(joint_mutex_);
      joint_map = latest_joint_map_;
    }

    RCLCPP_INFO(this->get_logger(), "%sJoint States:%s", MAGENTA, RESET);
    for (const auto& [joint_name, joint_position] : joint_map) {
      RCLCPP_INFO(this->get_logger(), "%s%s%s: %s%f%s", BYELLOW, joint_name.c_str(), RESET, BGREEN, joint_position, RESET);
    }

    if (joint_map.empty()) {
      RCLCPP_INFO(this->get_logger(), "No joint states received yet.");
    }
  }

  std::mutex joint_mutex_;  // 保护共享资源的互斥锁
  std::map<std::string, double> latest_joint_map_;  // 存储最新的关节值
  rclcpp::TimerBase::SharedPtr joint_print_timer_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<JointStatePrinter>();
  rclcpp::spin(node);  // 运行节点直到被显式关闭
  rclcpp::shutdown();
  return 0;
}