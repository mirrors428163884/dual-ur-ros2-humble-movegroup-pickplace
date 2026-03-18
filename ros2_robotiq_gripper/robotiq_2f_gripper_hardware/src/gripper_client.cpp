#include <chrono>
#include <memory>
#include <iostream>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "robotiq_2f_gripper_msgs/action/move_two_finger_gripper.hpp"
#include "robotiq_2f_gripper_hardware/gripper_client.hpp"

using namespace std::placeholders;

GripperClient::GripperClient(const std::string& left_or_right) : Node("left_gripper_client")
{
    // 创建一个动作客户端
    std::string gripe_action_name = (left_or_right == "left") ? "/left/robotiq_2f_gripper_action" : "/right/robotiq_2f_gripper_action";
    client_ = rclcpp_action::create_client<MoveTwoFingerGripper>(this, gripe_action_name);

    // 等待服务端启动
    while (!client_->wait_for_action_server(std::chrono::seconds(1))) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the action server to appear.");
            rclcpp::shutdown();
            return;
        }
        RCLCPP_INFO(this->get_logger(), "Waiting for the action server to appear...");
    }
}

void GripperClient::send_goal(double target_position, double target_speed, double target_force)
{
    // 构建目标请求
    auto goal_msg = MoveTwoFingerGripper::Goal();
    goal_msg.target_position = target_position;
    goal_msg.target_speed = target_speed;
    goal_msg.target_force = target_force;

    // 发送目标请求
    auto send_goal_options = rclcpp_action::Client<MoveTwoFingerGripper>::SendGoalOptions();
    send_goal_options.goal_response_callback = std::bind(&GripperClient::goal_response_callback, this, _1);
    send_goal_options.feedback_callback = std::bind(&GripperClient::feedback_callback, this, _1, _2);
    send_goal_options.result_callback = std::bind(&GripperClient::result_callback, this, _1);

    RCLCPP_INFO(this->get_logger(), "Sending goal request with position: %f, speed: %f, force: %f",
                target_position, target_speed, target_force);

    client_->async_send_goal(goal_msg, send_goal_options);
}

// void GripperClient::open_gripper()
// {
//     double open_position = 0.08; // 打开位置（米）
//     double target_speed = 0.5;   // 目标速度（0-1）
//     double target_force = 0.5;   // 目标力（0-1）

//     send_goal(open_position, target_speed, target_force);
// }

// void GripperClient::close_gripper()
// {
//     double close_position = 0.0; // 关闭位置（米）
//     double target_speed = 0.5;   // 目标速度（0-1）
//     double target_force = 0.5;   // 目标力（0-1）

//     send_goal(close_position, target_speed, target_force);
// }
void GripperClient::open_gripper(double target_position, double target_speed, double target_force)
    {
        send_goal(target_position, target_speed, target_force);
    }

void GripperClient::close_gripper(double target_position, double target_speed, double target_force)
{
    send_goal(target_position, target_speed, target_force);
}

void GripperClient::goal_response_callback(const GoalHandleMoveTwoFingerGripper::SharedPtr &goal_handle)
{
    if (!goal_handle) {
        RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
    } else {
        RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
    }
}

void GripperClient::feedback_callback(GoalHandleMoveTwoFingerGripper::SharedPtr, const std::shared_ptr<const MoveTwoFingerGripper::Feedback> feedback)
{
    RCLCPP_INFO(this->get_logger(), "Feedback received: %s", feedback->feedback.c_str());
}

void GripperClient::result_callback(const GoalHandleMoveTwoFingerGripper::WrappedResult &result)
{
    switch (result.code) {
        case rclcpp_action::ResultCode::SUCCEEDED:
            RCLCPP_INFO(this->get_logger(), "Goal succeeded!");
            break;
        case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
            return;
        case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
            return;
        default:
            RCLCPP_ERROR(this->get_logger(), "Unknown result code");
            return;
    }
}
//如果想要在本代码中测试就打开否则不需要，编译时不能有多个main
// int main(int argc, char *argv[])
// {
//     rclcpp::init(argc, argv);
//     auto node = std::make_shared<GripperClient>();

//     // 示例目标请求
//     node->open_gripper(); // 打开夹爪
//     //node->close_gripper();


//     // 保持节点运行，直到目标完成
//     rclcpp::spin(node);
//     rclcpp::shutdown();

//     return 0;
// }