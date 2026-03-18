import os
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch_ros.substitutions import FindPackageShare
from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration, Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution

from launch.actions import GroupAction
from launch_ros.actions import PushRosNamespace, Node
from launch_ros.descriptions import ParameterValue


def generate_launch_description():

    right_ur_type = LaunchConfiguration('right_ur_type')     
    right_robot_ip = LaunchConfiguration('right_robot_ip') 
    right_controller_file = LaunchConfiguration('right_controller_file') 
    right_tf_prefix = LaunchConfiguration('right_tf_prefix') 
    right_script_command_port = LaunchConfiguration('right_script_command_port')# default is 50004
    right_reverse_port = LaunchConfiguration('right_reverse_port')# default is 50001
    right_script_sender_port = LaunchConfiguration('right_script_sender_port')# default is 50002
    right_trajectory_port = LaunchConfiguration('right_trajectory_port')# default is 50003

    # right_gripper
    right_use_tool_communication = LaunchConfiguration("right_use_tool_communication")# default is false
    right_tool_parity = LaunchConfiguration("right_tool_parity")#
    right_tool_baud_rate = LaunchConfiguration("right_tool_baud_rate")
    right_tool_stop_bits = LaunchConfiguration("right_tool_stop_bits")
    right_tool_rx_idle_chars = LaunchConfiguration("right_tool_rx_idle_chars")
    right_tool_tx_idle_chars = LaunchConfiguration("right_tool_tx_idle_chars")
    right_tool_device_name = LaunchConfiguration("right_tool_device_name")
    right_tool_voltage = LaunchConfiguration("right_tool_voltage")



    left_ur_type = LaunchConfiguration('left_ur_type')    
    left_robot_ip = LaunchConfiguration('left_robot_ip') 
    left_controller_file = LaunchConfiguration('left_controller_file') 
    left_tf_prefix = LaunchConfiguration('left_tf_prefix') 
    left_script_command_port = LaunchConfiguration('left_script_command_port')# default is 50004
    left_reverse_port = LaunchConfiguration('left_reverse_port')# default is 50001
    left_script_sender_port = LaunchConfiguration('left_script_sender_port')# default is 50002
    left_trajectory_port = LaunchConfiguration('left_trajectory_port')# default is 50003

    # left_gripper
    left_use_tool_communication = LaunchConfiguration("left_use_tool_communication")# default is false
    left_tool_parity = LaunchConfiguration("left_tool_parity")#
    left_tool_baud_rate = LaunchConfiguration("left_tool_baud_rate")
    left_tool_stop_bits = LaunchConfiguration("left_tool_stop_bits")
    left_tool_rx_idle_chars = LaunchConfiguration("left_tool_rx_idle_chars")
    left_tool_tx_idle_chars = LaunchConfiguration("left_tool_tx_idle_chars")
    left_tool_device_name = LaunchConfiguration("left_tool_device_name")
    left_tool_voltage = LaunchConfiguration("left_tool_voltage")



    right_ur_type_arg = DeclareLaunchArgument(
            "right_ur_type",
            default_value='ur3e',
    )
    right_robot_ip_arg = DeclareLaunchArgument(
            "right_robot_ip",
            default_value='192.168.1.101',
    )
    right_controller_file_arg = DeclareLaunchArgument(
            "right_controller_file",
            default_value="ur_controllers.yaml",
    )
    right_tf_prefix_arg = DeclareLaunchArgument(
            "right_tf_prefix",
            default_value="right_",
    )
    right_script_command_port_arg =  DeclareLaunchArgument(
            "right_script_command_port",
            default_value="50004",
            description="Port that will be opened to forward URScript commands to the robot.",
        )
    right_reverse_port_arg = DeclareLaunchArgument(
            "right_reverse_port",
            default_value="50001",
            description="Port that will be opened to send cyclic instructions from the driver to the robot controller.",
        )
    right_script_sender_port_arg = DeclareLaunchArgument(
            "right_script_sender_port",
            default_value="50002",
            description="The driver will offer an interface to query the external_control URScript on this port.",
        )
    right_trajectory_port_arg = DeclareLaunchArgument(
            "right_trajectory_port",
            default_value="50003",
            description="Port that will be opened for trajectory control.",
        )
    right_use_tool_communication_arg = DeclareLaunchArgument(
            "right_use_tool_communication",
            default_value="true",
            description="Only available for e series!",
        )
    right_tool_parity_arg = DeclareLaunchArgument(
            "right_tool_parity",
            default_value="0",
            description="Parity configuration for serial communication. Only effective, if \
            use_tool_communication is set to True.",
        )
    right_tool_baud_rate_arg = DeclareLaunchArgument(
            "right_tool_baud_rate",
            default_value="115200",
            description="Baud rate configuration for serial communication. Only effective, if \
            use_tool_communication is set to True.",
        )
    right_tool_stop_bits_arg = DeclareLaunchArgument(
            "right_tool_stop_bits",
            default_value="1",
            description="Stop bits configuration for serial communication. Only effective, if \
            use_tool_communication is set to True.",
        )
    right_tool_rx_idle_chars_arg = DeclareLaunchArgument(
            "right_tool_rx_idle_chars",
            default_value="1.5",
            description="RX idle chars configuration for serial communication. Only effective, \
            if use_tool_communication is set to True.",
        )
    right_tool_tx_idle_chars_arg = DeclareLaunchArgument(
            "right_tool_tx_idle_chars",
            default_value="3.5",
            description="TX idle chars configuration for serial communication. Only effective, \
            if use_tool_communication is set to True.",
        )
    right_tool_device_name_arg = DeclareLaunchArgument(
            "right_tool_device_name",
            default_value="/tmp/ttyUR",
            description="File descriptor that will be generated for the tool communication device. \
            The user has be be allowed to write to this location. \
            Only effective, if use_tool_communication is set to True.",
        )
    right_tool_voltage_arg = DeclareLaunchArgument(
            "right_tool_voltage",
            default_value="24",  # 0 being a conservative value that won't destroy anything
            description="Tool voltage that will be setup.",
        )



    left_ur_type_arg = DeclareLaunchArgument(
            "left_ur_type",
            default_value='ur3e',
    )
    left_robot_ip_arg = DeclareLaunchArgument(
            "left_robot_ip",
            default_value='192.168.1.50',
    )
    left_controller_file_arg = DeclareLaunchArgument(
            "left_controller_file",
            default_value="ur_controllers.yaml",
    )
    left_tf_prefix_arg = DeclareLaunchArgument(
            "left_tf_prefix",
            default_value="left_",
    )
    left_script_command_port_arg =  DeclareLaunchArgument(
            "left_script_command_port",
            default_value="60004",
            description="Port that will be opened to forward URScript commands to the robot.",
        )
    left_reverse_port_arg = DeclareLaunchArgument(
            "left_reverse_port",
            default_value="60001",
            description="Port that will be opened to send cyclic instructions from the driver to the robot controller.",
        )
    left_script_sender_port_arg = DeclareLaunchArgument(
            "left_script_sender_port",
            default_value="60002",
            description="The driver will offer an interface to query the external_control URScript on this port.",
        )
    left_trajectory_port_arg = DeclareLaunchArgument(
            "left_trajectory_port",
            default_value="60003",
            description="Port that will be opened for trajectory control.",
        )
    left_use_tool_communication_arg = DeclareLaunchArgument(
            "left_use_tool_communication",
            default_value="true",
            description="Only available for e series!",
        )
    left_tool_parity_arg = DeclareLaunchArgument(
            "left_tool_parity",
            default_value="0",
            description="Parity configuration for serial communication. Only effective, if \
            use_tool_communication is set to True.",
        )
    left_tool_baud_rate_arg = DeclareLaunchArgument(
            "left_tool_baud_rate",
            default_value="115200",
            description="Baud rate configuration for serial communication. Only effective, if \
            use_tool_communication is set to True.",
        )
    left_tool_stop_bits_arg = DeclareLaunchArgument(
            "left_tool_stop_bits",
            default_value="1",
            description="Stop bits configuration for serial communication. Only effective, if \
            use_tool_communication is set to True.",
        )
    left_tool_rx_idle_chars_arg = DeclareLaunchArgument(
            "left_tool_rx_idle_chars",
            default_value="1.5",
            description="RX idle chars configuration for serial communication. Only effective, \
            if use_tool_communication is set to True.",
        )
    left_tool_tx_idle_chars_arg = DeclareLaunchArgument(
            "left_tool_tx_idle_chars",
            default_value="3.5",
            description="TX idle chars configuration for serial communication. Only effective, \
            if use_tool_communication is set to True.",
        )
    left_tool_device_name_arg = DeclareLaunchArgument(
            "left_tool_device_name",
            default_value="/tmp/ttyURL",
            description="File descriptor that will be generated for the tool communication device. \
            The user has be be allowed to write to this location. \
            Only effective, if use_tool_communication is set to True.",
        )
    left_tool_voltage_arg = DeclareLaunchArgument(
            "left_tool_voltage",
            default_value="24",  # 0 being a conservative value that won't destroy anything
            description="Tool voltage that will be setup.",
        )
  


    robot_description_content = ParameterValue(Command
    (
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([FindPackageShare("ur_description"), "urdf", "dual_ur.urdf.xacro"]),
            " ",
        ]
    ),
    value_type=str
    )
    robot_description = {"robot_description": robot_description_content}
    ur_robot_driver_path = get_package_share_directory('ur_robot_driver')



    right = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(ur_robot_driver_path, 'launch', 'ur_control.launch.py')),
        launch_arguments={'ur_type': right_ur_type,
                          'robot_ip': right_robot_ip,
                          'controllers_file': right_controller_file,
                          'tf_prefix': right_tf_prefix,
                          'script_command_port': right_script_command_port,
                          'trajectory_port': right_trajectory_port,
                          'reverse_port': right_reverse_port,
                          'script_sender_port': right_script_sender_port,
                          'use_tool_communication': right_use_tool_communication,
                          'tool_parity': right_tool_parity,
                          'tool_baud_rate': right_tool_baud_rate,
                          'tool_stop_bits': right_tool_stop_bits,
                          'tool_rx_idle_chars': right_tool_rx_idle_chars,
                          'tool_tx_idle_chars': right_tool_tx_idle_chars,
                          'tool_device_name': right_tool_device_name,
                          'tool_voltage': right_tool_voltage,
                          }.items())
    
    right_with_namespace = GroupAction(
     actions=[
         PushRosNamespace('right'),
         right,
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[robot_description]
        )         
      ]
    )



    left = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(ur_robot_driver_path, 'launch', 'ur_control.launch.py')),
        launch_arguments={'ur_type': left_ur_type,
                          'robot_ip': left_robot_ip,
                          'controllers_file': left_controller_file,
                          'tf_prefix': left_tf_prefix,
                          'script_command_port': left_script_command_port,
                          'trajectory_port': left_trajectory_port,
                          'reverse_port': left_reverse_port,
                          'script_sender_port': left_script_sender_port,
                          'use_tool_communication': left_use_tool_communication,
                          'tool_parity': left_tool_parity,
                          'tool_baud_rate': left_tool_baud_rate,
                          'tool_stop_bits': left_tool_stop_bits,
                          'tool_rx_idle_chars': left_tool_rx_idle_chars,
                          'tool_tx_idle_chars': left_tool_tx_idle_chars,
                          'tool_device_name': left_tool_device_name,
                          'tool_voltage': left_tool_voltage,
                          }.items())
    
    left_with_namespace = GroupAction(
     actions=[
         PushRosNamespace('left'),
         left,
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[robot_description]
        )         
      ]
    )



    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("ur_description"), "rviz", "view_robot_for_driver.rviz"]
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
    )

    

    return LaunchDescription([
        Node(
            package='topic_tools',
            executable='relay',
            name='relay_right',
            output='screen',
            arguments=['/right/joint_states', '/joint_states'],
        ), 
        Node(
            package='topic_tools',
            executable='relay',
            name='relay_left',
            output='screen',
            arguments=['/left/joint_states', '/joint_states'],
        ),       
        right_ur_type_arg,
        right_robot_ip_arg,
        right_controller_file_arg,
        right_tf_prefix_arg,
        right_script_command_port_arg,
        right_trajectory_port_arg,
        right_reverse_port_arg,
        right_script_sender_port_arg,
        right_use_tool_communication_arg,
        right_tool_parity_arg,
        right_tool_baud_rate_arg,
        right_tool_stop_bits_arg,
        right_tool_rx_idle_chars_arg,
        right_tool_tx_idle_chars_arg,
        right_tool_device_name_arg,
        right_tool_voltage_arg,
        left_ur_type_arg,
        left_robot_ip_arg,
        left_controller_file_arg,
        left_tf_prefix_arg,
        left_script_command_port_arg,
        left_trajectory_port_arg,
        left_reverse_port_arg,
        left_script_sender_port_arg,
        left_use_tool_communication_arg,
        left_tool_parity_arg,
        left_tool_baud_rate_arg,
        left_tool_stop_bits_arg,
        left_tool_rx_idle_chars_arg,
        left_tool_tx_idle_chars_arg,
        left_tool_device_name_arg,
        left_tool_voltage_arg,
        #rviz_node,# if you don't want to launch the rviz2 to show the robot state, comment it        
        right_with_namespace,
        left_with_namespace
    ])
