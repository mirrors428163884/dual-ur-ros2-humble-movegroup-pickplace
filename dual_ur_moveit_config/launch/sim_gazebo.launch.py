import os
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
    description_package = LaunchConfiguration("description_package")
    description_file = LaunchConfiguration("description_file")
    moveit_config_package = LaunchConfiguration("moveit_config_package")
    use_sim_time = LaunchConfiguration("use_sim_time")
    launch_controllers = LaunchConfiguration("launch_controllers")

    # 加载机器人描述（URDF/XACRO）
    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name="xacro")]), " ",
        PathJoinSubstitution([FindPackageShare(description_package), "urdf", description_file])
    ])
    robot_description = {"robot_description": ParameterValue(robot_description_content, value_type=str)}









    # 启动 Gazebo 仿真环境，使用空世界
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("gazebo_ros"), "launch", "gazebo.launch.py"])
        ),
        launch_arguments={
            "world": PathJoinSubstitution([FindPackageShare("ur_description"), "gazebo", "empty.world"]),
            "verbose": "true"
        }.items(),
    )

    # gazebo_world_file = os.path.join(
    #     FindPackageShare(package='ur_description').find('ur_description'),
    #     'gazebo','empty.world'
    # )
    # gazebo = ExecuteProcess(
    #     cmd=['gazebo', '--verbose', gazebo_world_file, '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so'], 
    #     output='screen',
    # )


    # 将机器人模型加载到 Gazebo，设置超时防止大模型加载失败
    gazebo_spawn_robot = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        name="spawn_ur",
        arguments=["-entity", "dual_arm", "-topic", "robot_description", "-timeout", "120"],
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
    )

    # 发布机器人状态（TF 树）
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description, {"use_sim_time": use_sim_time}],
    )

    # 控制器加载器（由 Gazebo 内部的 ros2_control 插件管理）
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
            arguments=[controller, "--controller-manager", "/controller_manager"],
            parameters=[{"use_sim_time": use_sim_time}],
            output="screen",
            condition=IfCondition(launch_controllers)
        )
        controller_spawners.append(spawner)

    return [gazebo, robot_state_publisher, gazebo_spawn_robot] + controller_spawners


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("description_package", default_value="ur_description"),
        DeclareLaunchArgument("description_file", default_value="test_new.urdf"),
        DeclareLaunchArgument("moveit_config_package", default_value="dual_ur_moveit_config"),
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("launch_controllers", default_value="true"),
        OpaqueFunction(function=launch_setup)
    ])