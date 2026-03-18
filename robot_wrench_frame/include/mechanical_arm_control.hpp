#ifndef MECHANICAL_ARM_CONTROL_H
#define MECHANICAL_ARM_CONTROL_H

#include <cstdlib>  // For std::system
#include <iostream> // For std::cout and std::cerr
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/wrench_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <controller_manager_msgs/srv/list_controllers.hpp>


class MechanicalArmControl : public rclcpp::Node
{
public:
    MechanicalArmControl(const std::string& left_or_right_name);
    bool move_to_b_and_execute();
    std::string get_controller_status(const std::string& controller_name);



//private:
    rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr wrench_publisher_left;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_publisher_left;
        rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr wrench_publisher_right;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_publisher_right;

    bool stop_scaled_joint_trajectory_controller();
    bool start_cartesian_compliance_controller();
    bool recovery_start_scaled_joint_trajectory_controller();
    bool recovery_stop_cartesian_compliance_controller();
    void publish_wrench_left(double force_x,double force_y,double force_z,
                             double torque_x,double torque_y,double torque_z);
    void publish_pose_left(double position_x,double position_y,double position_z,
                           double orientation_x,double orientation_y,
                           double orientation_z,double orientation_w);
    void publish_wrench_right(double force_x,double force_y,double force_z,
                             double torque_x,double torque_y,double torque_z);
    void publish_pose_right(double position_x,double position_y,double position_z,
                           double orientation_x,double orientation_y,
                           double orientation_z,double orientation_w);
    //void left_or_right_publish_test();
    bool verify_controller_stopped();
    bool verify_controller_stared();
    bool recovery_verify_controller_stared();
    bool recovery_verify_controller_stopped();
    bool recovery_controller_state();
    private:
    std::string left_or_right;
    std::string pd_left_right;
    int ret_code_stop;
    int ret_code_start;
     
};

#endif // MECHANICAL_ARM_CONTROL_H



