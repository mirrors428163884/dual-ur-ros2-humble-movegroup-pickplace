import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node, PushRosNamespace
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, Command, FindExecutable


def generate_launch_description():
    
    serial_right_port_arg = DeclareLaunchArgument(
        'serial_right_port',
        default_value='/tmp/ttyUR',
        description='Serial port of the gripper'
    )
    serial_left_port_arg = DeclareLaunchArgument(
        'serial_left_port',
        default_value='/tmp/ttyURL',
        description='Serial left port of the gripper'
    )
    baudrate_arg = DeclareLaunchArgument(
        'baudrate',
        default_value='115200',
        description='Baudrate of the gripper'
    )
    timeout_arg = DeclareLaunchArgument(
        'timeout',
        default_value='1.0',
        description='Timeout (connection) of the gripper'
    )
    action_timeout_arg = DeclareLaunchArgument(
        'action_timeout',
        default_value='20',
        description='Action timeout (movement) of the gripper'
    )
    slave_address_arg = DeclareLaunchArgument(
        'slave_address',
        default_value='9',
        description='Slave address of the gripper'
    )
    fake_hardware_arg = DeclareLaunchArgument(
        'fake_hardware',
        default_value='False',
        description='Whether to use fake hardware (if real hardware is not available)'
    )
    rviz2_arg = DeclareLaunchArgument(
        'rviz2',
        default_value='False',
        description='Wether to launch rviz2 for simultaneous visualization'
    )

    # left_robotiq_2f_gripper_node = Node(
    #     package='robotiq_2f_gripper_hardware',
    #     executable='robotiq_2f_gripper_node',
    #     name='robotiq_2f_gripper_node',
    #     output='screen',
    #     parameters=[{
    #         'serial_port': LaunchConfiguration('serial_left_port'),
    #         'baudrate': LaunchConfiguration('baudrate'),
    #         'timeout': LaunchConfiguration('timeout'),
    #         'action_timeout': LaunchConfiguration('action_timeout'),
    #         'slave_address': LaunchConfiguration('slave_address'),
    #         'fake_hardware': LaunchConfiguration('fake_hardware')           
    #     }]
    # )
    
    # right_robotiq_2f_gripper_node = Node(
    #     package='robotiq_2f_gripper_hardware',
    #     executable='robotiq_2f_gripper_node',
    #     name='robotiq_2f_gripper_node',
    #     output='screen',
    #     parameters=[{
    #         'serial_port': LaunchConfiguration('serial_right_port'),
    #         'baudrate': LaunchConfiguration('baudrate'),
    #         'timeout': LaunchConfiguration('timeout'),
    #         'action_timeout': LaunchConfiguration('action_timeout'),
    #         'slave_address': LaunchConfiguration('slave_address'),
    #         'fake_hardware': LaunchConfiguration('fake_hardware')           
    #     }]
    # )
    
    left_robotiq_2f_gripper_node_ns = GroupAction(
     actions=[
         PushRosNamespace('left'),
        Node(
        package='robotiq_2f_gripper_hardware',
        executable='gripper_node_test',
        name='gripper_node_test',
        output='screen',
        parameters=[{
            'serial_port': LaunchConfiguration('serial_left_port'),
            'baudrate': LaunchConfiguration('baudrate'),
            'timeout': LaunchConfiguration('timeout'),
            'action_timeout': LaunchConfiguration('action_timeout'),
            'slave_address': LaunchConfiguration('slave_address'),
            'fake_hardware': LaunchConfiguration('fake_hardware')           
        }]
    )        
      ]
    )
    
    right_robotiq_2f_gripper_node_ns = GroupAction(
     actions=[
         PushRosNamespace('right'),
        Node(
        package='robotiq_2f_gripper_hardware',
        executable='gripper_node_test',
        name='gripper_node_test',
        output='screen',
        parameters=[{
            'serial_port': LaunchConfiguration('serial_right_port'),
            'baudrate': LaunchConfiguration('baudrate'),
            'timeout': LaunchConfiguration('timeout'),
            'action_timeout': LaunchConfiguration('action_timeout'),
            'slave_address': LaunchConfiguration('slave_address'),
            'fake_hardware': LaunchConfiguration('fake_hardware')           
        }]
    )        
      ]
    )
    # left_gripper_client_start_node = Node(
    #     package='robotiq_2f_gripper_hardware',
    #     executable='left_gripper_client_node',
    #     name='left_gripper_client_node',
    #     output='screen'
        
    # )
    
    # test_use_action_client_node = Node(
    #     package='robotiq_2f_gripper_hardware',
    #     executable='test_use_action_client_node',
    #     name='test_use_action_client_node',
    #     output='screen'
        
    # )

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
        serial_right_port_arg,
        serial_left_port_arg,
        baudrate_arg,
        timeout_arg,
        action_timeout_arg,
        slave_address_arg,
        fake_hardware_arg,
        rviz2_arg,
        rviz2_node,
        robot_state_publisher_node,
        joint_state_publisher_node,
        right_robotiq_2f_gripper_node_ns,
        # left_robotiq_2f_gripper_node_ns,        
        # test_use_action_client_node        
    ])