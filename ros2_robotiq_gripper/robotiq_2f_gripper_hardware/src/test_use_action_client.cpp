/******************************线性模式******************************/

#include <chrono>
#include <thread>
#include <memory>
#include <iostream>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "robotiq_2f_gripper_msgs/action/move_two_finger_gripper.hpp"
#include "robotiq_2f_gripper_hardware/gripper_client.hpp"

using namespace std::chrono_literals;  // 用于使用时间单位如ms, s等

int main(int argc, char* argv[])
{  //*使用默认就直接使用node_right->open_gripper()，传值的化就
  // 使用node_right->open_gripper(0.085, 0.8, 0.6),full_open=0.142,close=0.0
  // 分别对应位置、速度、力， 其它类似
  rclcpp::init(argc, argv);

  {
    auto node_left = std::make_shared<GripperClient>("left");
    std::this_thread::sleep_for(2s);
    node_left->open_gripper();
    std::this_thread::sleep_for(3s);  // 假设B代码块执行需要3秒
    node_left->close_gripper();
    std::this_thread::sleep_for(1s);
    node_left->open_gripper();
  }

  {
    // auto node_right = std::make_shared<GripperClient>("right");
    // std::this_thread::sleep_for(2s);  // 假设A代码块执行需要2秒
    // node_right->open_gripper();       // 在a代码块后打开夹爪
    // std::this_thread::sleep_for(3s);  // 假设B代码块执行需要3秒
    // node_right->close_gripper();      // 在b代码块后关闭夹爪
    // std::this_thread::sleep_for(1s);  // 假设C代码块执行需要1秒
    // node_right->open_gripper();       // 在c代码块后再次打开夹爪
  }


  // 保持节点运行，直到所有目标完成
  // rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}

/**********************同时执行线程模式*******************************/
/*
#include <chrono>
#include <thread>
#include <memory>
#include <iostream>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "robotiq_2f_gripper_msgs/action/move_two_finger_gripper.hpp"
#include "robotiq_2f_gripper_hardware/gripper_client.hpp"

using namespace std::chrono_literals; // 用于使用时间单位如ms, s等

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node_right = std::make_shared<GripperClient>("right");
    auto node_left = std::make_shared<GripperClient>("left");

    // 模拟a代码块
    RCLCPP_INFO(node_right->get_logger(), "Executing block A...");
    std::this_thread::sleep_for(2s); // 假设A代码块执行需要2秒

    // 创建线程同时打开左右夹爪
    RCLCPP_INFO(node_right->get_logger(), "Opening both grippers...");
    std::thread right_open_thread(&GripperClient::open_gripper, node_right, 0.08, 0.5, 0.5);
    std::thread left_open_thread(&GripperClient::open_gripper, node_left, 0.08, 0.5, 0.5);

    // 等待两个线程完成
    right_open_thread.join();
    left_open_thread.join();

    // 模拟b代码块
    RCLCPP_INFO(node_right->get_logger(), "Executing block B...");
    std::this_thread::sleep_for(3s); // 假设B代码块执行需要3秒

    // 创建线程同时关闭左右夹爪
    RCLCPP_INFO(node_right->get_logger(), "Closing both grippers...");
    std::thread right_close_thread(&GripperClient::close_gripper, node_right, 0.0, 0.5, 0.5);
    std::thread left_close_thread(&GripperClient::close_gripper, node_left, 0.0, 0.5, 0.5);

    // 等待两个线程完成
    right_close_thread.join();
    left_close_thread.join();

    // 模拟c代码块
    RCLCPP_INFO(node_right->get_logger(), "Executing block C...");
    std::this_thread::sleep_for(1s); // 假设C代码块执行需要1秒

    // 再次同时打开左右夹爪
    RCLCPP_INFO(node_right->get_logger(), "Opening both grippers again...");
    std::thread right_open_thread2(&GripperClient::open_gripper, node_right, 0.08, 0.5, 0.5);
    std::thread left_open_thread2(&GripperClient::open_gripper, node_left, 0.08, 0.5, 0.5);

    // 等待两个线程完成
    right_open_thread2.join();
    left_open_thread2.join();

    // 保持节点运行，直到所有目标完成
    // rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}
*/