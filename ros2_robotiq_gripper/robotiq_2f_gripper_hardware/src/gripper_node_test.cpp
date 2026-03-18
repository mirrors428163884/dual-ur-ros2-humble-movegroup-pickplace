#include <chrono>
#include <thread>
#include <iostream>
#include <memory>
#include <cmath>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>

#include <robotiq_driver/default_driver.hpp>
#include <robotiq_driver/default_serial.hpp>

#include <robotiq_2f_gripper_hardware/robotiq_2f_gripper_node.hpp>

using namespace robotiq_2f_gripper_hardware;
using namespace std::placeholders;
using robotiq_driver::DefaultDriver;
using robotiq_driver::DefaultSerial;


int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<GripperNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}