import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, Command, FindExecutable

def generate_launch_description():
    
    rviz2_arg = DeclareLaunchArgument(
        'rviz2',
        default_value='False',
        description='Wether to launch rviz2 for simultaneous visualization'
    )
    
    test_use_action_client_node = Node(
        package='robotiq_2f_gripper_hardware',
        executable='test_use_action_client_node',
        name='test_use_action_client_node',
        output='screen'
        
    )

    urdf_file = os.path.join(get_package_share_directory('robotiq_description'), 'urdf', 'robotiq_2f_85_macro.urdf.xacro')
    rviz_config_file = os.path.join(get_package_share_directory('robotiq_description'), 'rviz', 'view_urdf.rviz')

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': Command(['xacro ', urdf_file])
        }],
        condition=IfCondition(LaunchConfiguration('rviz2'))
    )

    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[{
            'source_list': ['/robotiq_2f_gripper/joint_states']
        }],
        condition=IfCondition(LaunchConfiguration('rviz2'))
    )

    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config_file],
        condition=IfCondition(LaunchConfiguration('rviz2'))
    )

    return LaunchDescription([      
        rviz2_arg,
        rviz2_node,
        robot_state_publisher_node,
        joint_state_publisher_node,         
        test_use_action_client_node        
    ])