import os
import copy

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
    ExecuteProcess
)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):
    # Initialize Arguments
    description_package = LaunchConfiguration("description_package")
    description_file = LaunchConfiguration("description_file")
    moveit_config_package = LaunchConfiguration("moveit_config_package") # 保留用于查找 ros2_controllers.yaml
    use_sim_time = LaunchConfiguration("use_sim_time", default="true")
    launch_gazebo_gui = LaunchConfiguration("launch_gazebo_gui", default="true")

    # Generate Robot Description (URDF)
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([FindPackageShare(description_package), "urdf", description_file]),
            " ",
        ]
    )
    robot_description = {"robot_description": ParameterValue(robot_description_content, value_type=str)}


    gazebo_world_file = os.path.join(
        FindPackageShare(package='ur_description').find('ur_description'),
        'gazebo','empty.world'
    )
    # gazebo = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         [FindPackageShare("gazebo_ros"), "/launch", "/gazebo.launch.py"]
    #     ),
    #     launch_arguments={
    #         "world": gazebo_world_file,
    #     }.items(),
    # )

    gazebo = ExecuteProcess(
        cmd=['gazebo', '--verbose', gazebo_world_file, '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so'], 
        output='screen',
    )

    # Spawn robot
    gazebo_spawn_robot = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        name="spawn_ur",
        arguments=["-entity", "dual_ur", "-topic", "robot_description"],
        output="screen",
    )




    # === ROS 2 Control Setup ===
    
    # 加载 ros2_controllers.yaml (通常位于 moveit_config_package 或 description_package 的 config 文件夹)
    # 这里沿用原逻辑，假设它在 moveit_config_package 中，如果报错请调整包名
    ros2_controllers_path = PathJoinSubstitution(
        [FindPackageShare(moveit_config_package), "config", "ros2_controllers.yaml"]
    )
    
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            robot_description, 
            ros2_controllers_path,
            {"use_sim_time": use_sim_time}
        ],
        output="screen",
        remappings=[
            ("/controller_manager/robot_description", "/robot_description"),
        ]
    )

    # === Controller Spawners ===
    # 根据常见的双 UR 配置定义控制器列表
    controller_names = [
        "joint_state_broadcaster",
        "left_controller",
        "right_controller",
        "left_gripper_controller",
        "right_gripper_controller"
    ]
    
    controller_spawners = []
    for controller in controller_names:
        spawner = Node(
            package="controller_manager",
            executable="spawner",
            arguments=[
                controller, 
                "--controller-manager", "/controller_manager",
            ],
            parameters=[{"use_sim_time": use_sim_time}],
            output="screen"
        )
        controller_spawners.append(spawner)

    # Robot State Publisher (Required for TF trees in Gazebo/Control)
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            robot_description,
            {"use_sim_time": use_sim_time}
        ],
    )

    # Assemble nodes list
    nodes_to_start = [
        gazebo,
        gazebo_spawn_robot,
        robot_state_publisher,
        ros2_control_node,
    ] + controller_spawners

    return nodes_to_start


def generate_launch_description():
    declared_arguments = []
    
    # General Arguments
    declared_arguments.append(
        DeclareLaunchArgument("description_package", default_value="ur_description", description="Description package with robot URDF/XACRO files.")
    )
    declared_arguments.append(
        DeclareLaunchArgument("description_file", default_value="test_new.urdf", description="URDF/XACRO description file with the robot.")
    )
    # 保留此参数以便找到 ros2_controllers.yaml，即使我们不再加载其他 MoveIt 配置
    declared_arguments.append(
        DeclareLaunchArgument("moveit_config_package", default_value="dual_ur_moveit_config", description="Config package containing ros2_controllers.yaml.")
    )
    
    declared_arguments.append(
        DeclareLaunchArgument("use_sim_time", default_value="true", description="Make controllers use simulation time.")
    )
    declared_arguments.append(
        DeclareLaunchArgument("launch_gazebo_gui", default_value="true", description="Launch Gazebo GUI?")
    )

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])