#ifndef LEFT_GRIPPER_CLIENT_H
#define LEFT_GRIPPER_CLIENT_H

#include <chrono>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <robotiq_2f_gripper_msgs/action/move_two_finger_gripper.hpp>

using MoveTwoFingerGripper = robotiq_2f_gripper_msgs::action::MoveTwoFingerGripper;
using GoalHandleMoveTwoFingerGripper = rclcpp_action::ClientGoalHandle<MoveTwoFingerGripper>;

class GripperClient : public rclcpp::Node
{
public:
    explicit GripperClient(const std::string& left_or_right); // 构造函数

    void send_goal(double target_position, double target_speed, double target_force); // 发送目标请求
    void open_gripper(double target_position = 0.05, double target_speed = 0.01, double target_force = 1);
    void close_gripper(double target_position = 0.0, double target_speed = 0.01, double target_force = 1);

private:
    rclcpp_action::Client<MoveTwoFingerGripper>::SharedPtr client_; // 动作客户端指针

    void goal_response_callback(const GoalHandleMoveTwoFingerGripper::SharedPtr &goal_handle); // 处理目标响应
    void feedback_callback(GoalHandleMoveTwoFingerGripper::SharedPtr, const std::shared_ptr<const MoveTwoFingerGripper::Feedback> feedback); // 处理反馈
    void result_callback(const GoalHandleMoveTwoFingerGripper::WrappedResult &result); // 处理结果
};

#endif // LEFT_GRIPPER_CLIENT_H
