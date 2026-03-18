#include "mechanical_arm_control.hpp"


int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    //auto node = std::make_shared<MechanicalArmControl>("left");
    auto node = std::make_shared<MechanicalArmControl>("right");

    // 左臂模拟机械臂从 A 到 B，并执行后续任务，启动
    if(node->move_to_b_and_execute())
    {
        RCLCPP_INFO(node->get_logger(), "success stop_scaled_joint_controller");
        RCLCPP_INFO(node->get_logger(), "success start_cartesian_controller");
    }
    else
    {
        RCLCPP_INFO(node->get_logger(), "fail stop_scaled_joint_controller");
        RCLCPP_INFO(node->get_logger(), "fail start_cartesian_controller");
    }
    
    /*
    //左臂发布者
    node->publish_wrench_left(0.0,0.0,0.0,
                        0.0,0.0,0.0);
    // 5. 发布位置指令
    //0.42148; -0.1198; 1.1851
    //-0.67546; 0.23834; 0.23875; 0.6557
    //0.33403; -0.091583; 1.295
    //0.71943; -0.045026; -0.6931; -0.0010965
    
    node->publish_pose_left(0.19666, -0.063502, 0.3,
                        -0.47868, 0.87788, 0.0059808, -0.012379);
    RCLCPP_INFO(node->get_logger(), "publish_left");
    */
    
    
    //右臂发布者
    node->publish_wrench_right(0.0,0.0,0.0,
                        0.0,0.0,0.0);
    // 5. 发布位置指令
    
    node->publish_pose_right(-0.21842, -0.14374, 0.25,
                        0.77807, -0.42169, -0.15758, 0.43812);
    RCLCPP_INFO(node->get_logger(), "publish_right");

    //关闭力控制器
    if(node->recovery_controller_state())
    {
        RCLCPP_INFO(node->get_logger(), "success recovery start_scaled_joint_controller");
        RCLCPP_INFO(node->get_logger(), "success recovery stop_cartesian_controller");
    }
    else
    {
        RCLCPP_INFO(node->get_logger(), "fail recovery start_scaled_joint_controller");
        RCLCPP_INFO(node->get_logger(), "fail recovery stop_cartesian_controller");
    }
  

    rclcpp::shutdown();
    return 0;
}