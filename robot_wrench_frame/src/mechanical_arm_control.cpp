#include "mechanical_arm_control.hpp"


MechanicalArmControl::MechanicalArmControl(const std::string& left_or_right_name) : Node("mechanical_arm_control")
{
    // 创建力和位置发布者
    // declare_parameter<std::string>("left_or_right");
    // left_or_right = get_parameter("left_or_right").as_string();//hpp声明,launch传过来赋值
    //RCLCPP_INFO(get_logger(), "Using left_or_right: %s", left_or_right.c_str());
    left_or_right=left_or_right_name;
    RCLCPP_INFO(get_logger(), "Using left_or_right: %s", left_or_right.c_str());
    if(left_or_right=="left")
    {   RCLCPP_INFO(get_logger(), "you are using left_arm_mechannical");
        wrench_publisher_left = this->create_publisher<geometry_msgs::msg::WrenchStamped>("/left/target_wrench", 10);
        pose_publisher_left = this->create_publisher<geometry_msgs::msg::PoseStamped>("/left/target_frame", 10);
    }
    else
    {   
        RCLCPP_INFO(get_logger(), "you are using right_arm_mechannical");
        wrench_publisher_right = this->create_publisher<geometry_msgs::msg::WrenchStamped>("/right/target_wrench", 10);
        pose_publisher_right = this->create_publisher<geometry_msgs::msg::PoseStamped>("/right/target_frame", 10);
    }
   
}

bool MechanicalArmControl::move_to_b_and_execute()
{
    // 1. 模拟机械臂从 A 移动到 B 的动作
    RCLCPP_INFO(this->get_logger(), "Simulation Moving arm from A to B...");
    rclcpp::sleep_for(std::chrono::seconds(2)); // 假装移动需要 2 秒
    RCLCPP_INFO(this->get_logger(), "2s Arm reached position B.");


    // 2. 停止 scaled_joint_trajectory_controller
    if (stop_scaled_joint_trajectory_controller())
    {
        // 3. 启动 cartesian_compliance_controller
        if (start_cartesian_compliance_controller())
        {   
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
            //left_or_right_publish_test();
            return true;
        }
        else
        {
            return false;
        }
    }
    else
    {
        return false;
    }
    
    //必须等待动作完成再恢复
    //rclcpp::sleep_for(std::chrono::seconds(3)); 
    //力控完成恢复控制器状态
    //recovery_controller_state();
    
    //如果要持续使用力控，可使用下面
    //2.1 先确定scal控制器是否是停止的
    /*if (verify_controller_stopped())
    {
        //2.2 是 判断在car是否启动
        if (verify_controller_stared())
        {
            //2.2.1 car启动  直接走发布 
            left_or_right_publish();   
        }
        else
        {
            //2.2.2 car没启动 启动再发布
            if(start_cartesian_compliance_controller())
            {
                rclcpp::sleep_for(std::chrono::seconds(1));
                left_or_right_publish();
            }
            
        }
    }
    else
    {
        //2.2 否 先停止scal 再判断car是否启动
        if(stop_scaled_joint_trajectory_controller())
        {
            if (verify_controller_stared())
            {
                //2.2.1 car启动  直接走发布
                rclcpp::sleep_for(std::chrono::seconds(2)); 
                left_or_right_publish();   
            }
            else
            {
                //2.2.2 car没启动 启动再发布
                if(start_cartesian_compliance_controller())
                {  
                   rclcpp::sleep_for(std::chrono::seconds(2));
                   left_or_right_publish(); 
                }
                
            }
        }        

    } */
 

}

//停止
bool MechanicalArmControl::stop_scaled_joint_trajectory_controller()
{   

    pd_left_right=(left_or_right == "left") ? "/left/controller_manager/list_controllers" : "/right/controller_manager/list_controllers";
    // RCLCPP_INFO(get_logger(), "111111111111111: %s", pd_left_right.c_str());
    
    const char* command_left = "ros2 control switch_controllers -c /left/controller_manager --stop  scaled_joint_trajectory_controller";
    const char* command_right = "ros2 control switch_controllers -c /right/controller_manager --stop  scaled_joint_trajectory_controller";
    if(left_or_right == "left")
    {
      RCLCPP_INFO(get_logger(), "111111111111111");
      ret_code_stop = std::system(command_left);    
    }
    else
    { 
      RCLCPP_INFO(get_logger(), "222222222222");
      ret_code_stop = std::system(command_right); 
    }    

    if (ret_code_stop == 0)
    {
        // 确保控制器状态切换完成（可选的检查逻辑）
        std::this_thread::sleep_for(std::chrono::milliseconds(500));  // 延时500ms，等待切换完成
        RCLCPP_INFO(this->get_logger(), "Successfully stopped scaled_joint_trajectory_controller.");

        // 检查控制器是否真正停止（如果有API可调用）
        if (verify_controller_stopped())  // 需要实现verify_controller_stopped()
        {
            RCLCPP_INFO(this->get_logger(), "Controller has been verified as stopped.");
            return true;
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Controller did not stop successfully after command.");
            return false;
        }
    }
    else
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to stop scaled_joint_trajectory_controller. Error code: %d", ret_code_stop);
        return false;
    }
}

bool MechanicalArmControl::start_cartesian_compliance_controller()
{
    pd_left_right=(left_or_right == "left") ? "/left/controller_manager/list_controllers" : "/right/controller_manager/list_controllers";
    // RCLCPP_INFO(get_logger(), "111111111111111: %s", pd_left_right.c_str());
    
    const char* command_left_st = "ros2 control switch_controllers -c /left/controller_manager --start  cartesian_compliance_controller";
    const char* command_right_st = "ros2 control switch_controllers -c /right/controller_manager --start  cartesian_compliance_controller";
    if(left_or_right == "left")
    {
      ret_code_start = std::system(command_left_st);    
    }
    else
    {
      ret_code_start = std::system(command_right_st); 
    }    

    if (ret_code_start == 0)
    {
        // 确保控制器状态切换完成（可选的检查逻辑）
        std::this_thread::sleep_for(std::chrono::milliseconds(500));  // 延时500ms，等待切换完成
        RCLCPP_INFO(this->get_logger(), "Successfully stared cartesian_compliance_controller.");

        // 检查控制器是否真正启动）
        if (verify_controller_stared())  // 需要实现verify_controller_stared()
        {
            RCLCPP_INFO(this->get_logger(), "Controller has been verified as stared.");
            return true;
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Controller did not start successfully after command.");
            return false;
        }
    }
    else
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to start cartesian_compliance_controller. Error code: %d", ret_code_start);
        return false;
    }
}

bool MechanicalArmControl::recovery_start_scaled_joint_trajectory_controller()
{   

    pd_left_right=(left_or_right == "left") ? "/left/controller_manager/list_controllers" : "/right/controller_manager/list_controllers";
    // RCLCPP_INFO(get_logger(), "111111111111111: %s", pd_left_right.c_str());
    
    const char* command_left = "ros2 control switch_controllers -c /left/controller_manager --start  scaled_joint_trajectory_controller";
    const char* command_right = "ros2 control switch_controllers -c /right/controller_manager --start  scaled_joint_trajectory_controller";
    if(left_or_right == "left")
    {
      ret_code_stop = std::system(command_left);    
    }
    else
    {
      ret_code_stop = std::system(command_right); 
    }    

    if (ret_code_stop == 0)
    {
        // 确保控制器状态切换完成（可选的检查逻辑）
        std::this_thread::sleep_for(std::chrono::milliseconds(500));  // 延时500ms，等待切换完成  
        //RCLCPP_INFO(this->get_logger(), "Successfully recovery start scaled_joint_trajectory_controller.");

        // 检查控制器是否真正停止（如果有API可调用）
        if (recovery_verify_controller_stared())  // 需要实现recovery 关节控制器的启动
        {
            RCLCPP_INFO(this->get_logger(), "Controller has been verified as recovery started.");
            return true;
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Controller did not recovery start successfully after command.");
            return false;
        }
    }
    else
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to recovery start scaled_joint_trajectory_controller. Error code: %d", ret_code_stop);
        return false;
    }
}

bool MechanicalArmControl::recovery_stop_cartesian_compliance_controller()
{
    pd_left_right=(left_or_right == "left") ? "/left/controller_manager/list_controllers" : "/right/controller_manager/list_controllers";
    // RCLCPP_INFO(get_logger(), "111111111111111: %s", pd_left_right.c_str());
    
    const char* command_left_st = "ros2 control switch_controllers -c /left/controller_manager --stop  cartesian_compliance_controller";
    const char* command_right_st = "ros2 control switch_controllers -c /right/controller_manager --stop  cartesian_compliance_controller";
    if(left_or_right == "left")
    {
      ret_code_start = std::system(command_left_st);    
    }
    else
    {
      ret_code_start = std::system(command_right_st); 
    }    

    if (ret_code_start == 0)
    {
        // 确保控制器状态切换完成（可选的检查逻辑）
        std::this_thread::sleep_for(std::chrono::milliseconds(500));  // 延时500ms，等待切换完成
        RCLCPP_INFO(this->get_logger(), "Successfully recovery stoped cartesian_compliance_controller.");

        // 检查控制器是否真正启动）
        if (recovery_verify_controller_stopped())  // 恢复力控关闭状态
        {
            RCLCPP_INFO(this->get_logger(), "Controller has been verified as recovery stoped.");
            return true;
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Controller did not recovery stoped successfully after command.");
            return false;
        }
    }
    else
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to recovery stop cartesian_compliance_controller. Error code: %d", ret_code_start);
        return false;
    }
}

void MechanicalArmControl::publish_wrench_left(double force_x,
                                               double force_y,
                                               double force_z,
                                               double torque_x,
                                               double torque_y,
                                               double torque_z)
{
    auto wrench_msg = geometry_msgs::msg::WrenchStamped();
    wrench_msg.header.stamp = this->get_clock()->now();
    wrench_msg.header.frame_id = "left_base_link_inertia";
   
    // 设置力和力矩  
    wrench_msg.wrench.force.x = force_x;
    wrench_msg.wrench.force.y = force_y;
    wrench_msg.wrench.force.z = force_z;

    wrench_msg.wrench.torque.x = torque_x;
    wrench_msg.wrench.torque.y = torque_y;
    wrench_msg.wrench.torque.z = torque_z;

    wrench_publisher_left->publish(wrench_msg);

    RCLCPP_INFO(this->get_logger(), "Published left WrenchStamped: force x: %.2f, y: %.2f, z: %.2f",
                wrench_msg.wrench.force.x, wrench_msg.wrench.force.y, wrench_msg.wrench.force.z);
}


void MechanicalArmControl::publish_pose_left(double position_x,
                                             double position_y,
                                             double position_z,
                                             double orientation_x,
                                             double orientation_y,
                                             double orientation_z,
                                             double orientation_w)
{
    auto pose_msg = geometry_msgs::msg::PoseStamped();
    pose_msg.header.stamp = this->get_clock()->now();
    pose_msg.header.frame_id = "left_base_link_inertia";

    // 设置位置和方向
    //0.069843; 0.99502; -0.057578; 0.041637
    pose_msg.pose.position.x = position_x;
    pose_msg.pose.position.y = position_y;
    pose_msg.pose.position.z = position_z;

    pose_msg.pose.orientation.x = orientation_x;
    pose_msg.pose.orientation.y = orientation_y;
    pose_msg.pose.orientation.z = orientation_z;
    pose_msg.pose.orientation.w = orientation_w;

    pose_publisher_left->publish(pose_msg);

    RCLCPP_INFO(this->get_logger(), "Published left PoseStamped: position x: %.2f, y: %.2f, z: %.2f",
                pose_msg.pose.position.x, pose_msg.pose.position.y, pose_msg.pose.position.z);
    RCLCPP_INFO(this->get_logger(), "Published left PoseStamped: orirentation x: %.2f, y: %.2f, z: %.2f,  w: %.2f",
            pose_msg.pose.orientation.x, pose_msg.pose.orientation.y, pose_msg.pose.orientation.z, pose_msg.pose.orientation.w);
}


void MechanicalArmControl::publish_wrench_right(double force_x,
                                                double force_y,
                                                double force_z,
                                                double torque_x,
                                                double torque_y,
                                                double torque_z)
{
    auto wrench_msg = geometry_msgs::msg::WrenchStamped();
    wrench_msg.header.stamp = this->get_clock()->now();
    wrench_msg.header.frame_id = "right_base_link_inertia";
   
    // 设置力和力矩
    wrench_msg.wrench.force.x = force_x;
    wrench_msg.wrench.force.y = force_y;
    wrench_msg.wrench.force.z = force_z;

    wrench_msg.wrench.torque.x = torque_x;
    wrench_msg.wrench.torque.y = torque_y;
    wrench_msg.wrench.torque.z = torque_z;

    wrench_publisher_right->publish(wrench_msg);

    RCLCPP_INFO(this->get_logger(), "Published right WrenchStamped: force x: %.2f, y: %.2f, z: %.2f",
                wrench_msg.wrench.force.x, wrench_msg.wrench.force.y, wrench_msg.wrench.force.z);
}



void MechanicalArmControl::publish_pose_right(double position_x,
                                              double position_y,
                                              double position_z,
                                              double orientation_x,
                                              double orientation_y,
                                              double orientation_z,
                                              double orientation_w)
{
    auto pose_msg = geometry_msgs::msg::PoseStamped();
    pose_msg.header.stamp = this->get_clock()->now();
    pose_msg.header.frame_id = "right_base_link_inertia";

    // 设置位置和方向
    //0.069843; 0.99502; -0.057578; 0.041637
    pose_msg.pose.position.x = position_x;
    pose_msg.pose.position.y = position_y;
    pose_msg.pose.position.z = position_z;

    pose_msg.pose.orientation.x = orientation_x;
    pose_msg.pose.orientation.y = orientation_y;
    pose_msg.pose.orientation.z = orientation_z;
    pose_msg.pose.orientation.w = orientation_w;

    pose_publisher_right->publish(pose_msg);

    RCLCPP_INFO(this->get_logger(), "Published right PoseStamped: position x: %.2f, y: %.2f, z: %.2f",
                pose_msg.pose.position.x, pose_msg.pose.position.y, pose_msg.pose.position.z);
    RCLCPP_INFO(this->get_logger(), "Published right PoseStamped: orirentation x: %.2f, y: %.2f, z: %.2f,  w: %.2f",
            pose_msg.pose.orientation.x, pose_msg.pose.orientation.y, pose_msg.pose.orientation.z, pose_msg.pose.orientation.w);
}


// void MechanicalArmControl::left_or_right_publish_test()
// {
//     if(left_or_right=="left")
//     {
//         // 4. 发布力指令
//         //0.26542; -0.21098; 0.32257
//         publish_wrench_left(0.0,0.0,0.0,
//                             0.0,0.0,0.0);
//         // 5. 发布位置指令
//         //0.030482; 0.99657; -0.05991; 0.048189
//         publish_pose_left(0.26542,-0.2109,0.5,
//                           0.030482,0.99657,-0.05991,0.048189);
//         RCLCPP_INFO(this->get_logger(), "publish_left");
//     }
//     else
//     {
        
//         // 6. 发布力指令
//         publish_wrench_right(0.0,0.0,0.0,
//                              0.0,0.0,0.0);                
//         // 7. 发布位置指令
//         publish_pose_right(0.0,0.1,0.2,
//                           0.0,0.1,0.1,0.2);
//         RCLCPP_INFO(this->get_logger(), "publish_right");            
//     }
// }

//返回控制器状态
std::string MechanicalArmControl::get_controller_status(const std::string& controller_name)
    {
        // 创建服务客户端
        pd_left_right=(left_or_right == "left") ? "/left/controller_manager/list_controllers" : "/right/controller_manager/list_controllers";
        RCLCPP_INFO(get_logger(), "pd_left_right: %s", pd_left_right.c_str());
        auto client = this->create_client<controller_manager_msgs::srv::ListControllers>(
            pd_left_right);

        // 等待服务可用
        if (!client->wait_for_service(std::chrono::seconds(5)))
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to connect to /controller_manager/list_controllers service.");
            return "unknown";
        }

        // 创建请求
        auto request = std::make_shared<controller_manager_msgs::srv::ListControllers::Request>();

        // 调用服务并获取响应
        auto future_result = client->async_send_request(request);
        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future_result) != rclcpp::FutureReturnCode::SUCCESS)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to call /controller_manager/list_controllers service.");
            return "unknown";
        }

        // 获取服务响应
        auto response = future_result.get();
        for (const auto& controller : response->controller)
        {
            if (controller.name == controller_name)
            {
                // 返回目标控制器的状态
                RCLCPP_INFO(this->get_logger(), "Controller '%s' is in state: %s", controller.name.c_str(), controller.state.c_str());
                return controller.state;
            }
        }

        // 未找到目标控制器
        RCLCPP_WARN(this->get_logger(), "Controller '%s' not found.", controller_name.c_str());
        return "not_found";
    }

bool MechanicalArmControl::verify_controller_stopped()
{
    // 通过调用ROS服务或状态接口，确认控制器状态是否为“stopped”
    // 示例（伪代码）：调用服务获取状态
    std::string controller_status = get_controller_status("scaled_joint_trajectory_controller");
    return controller_status == "inactive";
}

bool MechanicalArmControl::verify_controller_stared()
{
    // 通过调用ROS服务或状态接口，确认控制器状态是否为“stared”
    // 示例（伪代码）：调用服务获取状态
    std::string controller_status = get_controller_status("cartesian_compliance_controller");
    return controller_status == "active";
}


bool MechanicalArmControl::recovery_verify_controller_stared()
{

    std::string controller_status = get_controller_status("scaled_joint_trajectory_controller");
    RCLCPP_INFO(this->get_logger(), "controller_status %s",controller_status.c_str());  
    return controller_status == "active";
}

bool MechanicalArmControl::recovery_verify_controller_stopped()
{

    std::string controller_status = get_controller_status("cartesian_compliance_controller");
    RCLCPP_INFO(this->get_logger(), "controller_status %s",controller_status.c_str()); 
    return controller_status == "inactive";
}

//力控恢复
bool MechanicalArmControl::recovery_controller_state()
{
    
    //std::this_thread::sleep_for(std::chrono::milliseconds(500));

    if(recovery_stop_cartesian_compliance_controller())//恢复到力控关闭
    {
        RCLCPP_INFO(this->get_logger(), "recovery_stop_cartesian_compliance_controller");
        
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    
        if(recovery_start_scaled_joint_trajectory_controller())
        {
            RCLCPP_INFO(this->get_logger(), "recovery_start_scaled_joint_trajectory_controller");
            return true;
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "fail recovery_start_scaled_joint_trajectory_controller");
            return false;
        }
    }
    else
    {
        RCLCPP_INFO(this->get_logger(), "fail recovery_stop_cartesian_compliance_controller");
        return false;
    }
    

}


// int main(int argc, char *argv[])
// {
//     rclcpp::init(argc, argv);

//     auto node = std::make_shared<MechanicalArmControl>();

//     // 模拟机械臂从 A 到 B，并执行后续任务
//     node->move_to_b_and_execute();
    
//     rclcpp::shutdown();
//     return 0;
// }


