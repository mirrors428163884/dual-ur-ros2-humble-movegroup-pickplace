from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution, Command
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from ur_moveit_config.launch_common import load_yaml
import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
from launch_param_builder import ParameterBuilder


import yaml
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from ament_index_python.packages import get_package_share_directory
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import PythonExpression

def generate_launch_description():
    # --- 1. 加载 URDF ---
    robot_description_content = ParameterValue(
        Command([
            'xacro ', 
            PathJoinSubstitution([
                FindPackageShare('ur_description'),
                'urdf',
                'test.urdf'
            ])
        ]),
        value_type=str
    )
    # robot_description_content = Command(
    #     [
    #         PathJoinSubstitution([FindExecutable(name="xacro")]),
    #         " ",
    #         PathJoinSubstitution(
    #             [FindPackageShare("ur_description"), "urdf", "test.urdf"]
    #         ),
    #     ]
    # )
    robot_description = {"robot_description": robot_description_content}

    # MoveIt Configuration
    robot_description_semantic_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("dual_ur_moveit_config"), "config", "dual_arm.srdf"]
            ),
            " ",
        ]
    )
    robot_description_semantic = {
        "robot_description_semantic": ParameterValue(
            robot_description_semantic_content, value_type=str
        )
    }

    # robot_description_kinematics = PathJoinSubstitution(
    #     [FindPackageShare("uv_single_moveit"), "config", "kinematics.yaml"]
    # )
    # robot_description_kinematics = {
    #     "robot_description_kinematics":
    #     load_yaml(
    #         "dual_ur_moveit_config",
    #         os.path.join("config", "kinematics.yaml"),
    #     )
    # }




    # RViz
    # rviz_config_file = (
    #     get_package_share_directory("x_moveit_servo")
    #     + "/config/demo_rviz_pose_tracking.rviz"
    # )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        # prefix=['xterm -e gdb -ex run --args'],
        output="log",
        # arguments=["-d", rviz_config_file],
        parameters=[
            robot_description,
            robot_description_semantic,
        ],
    )

    # Publishes tf's for the robot
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[
            robot_description,
        ],
    )

    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        arguments=["0.0", "0.0", "0.0", "0.0", "0.0", "0.0", "world", "one_arm_base_link"],
    )

    # # Get parameters for the Pose Tracking node
    # servo_params = {
    #     "moveit_servo": ParameterBuilder("x_moveit_servo")
    #     .yaml("config/pose_tracking_settings.yaml")
    #     .yaml("config/dual_arm_simulated_config_pose_tracking.yaml")
    #     .to_dict()
    # }

    # pose_tracking_node = Node(
    #     package="x_moveit_servo",
    #     executable="servo_pose_tracking_demo",
    #     # prefix=['xterm -e gdb -ex run --args'],
    #     output="screen",
    #     parameters=[
    #         robot_description,
    #         robot_description_semantic,
    #         servo_params,
    #     ],
    # )


    # 右臂伺服参数（使用新配置文件）
    right_arm_servo_params = {
        "moveit_servo": ParameterBuilder("dual_ur_moveit_config")
        .yaml("config/pose_tracking_settings.yaml")  # PID参数
        .yaml("config/right_arm_servo_config.yaml")  # 新增的右臂配置
        .to_dict()
    }

    # 双臂跟随节点
    dual_arm_follower_node = Node(
        package="motion_plan",
        executable="dual_arm_follower",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            right_arm_servo_params,
        ],
    )



    # --- 3. 加载你的 pick_object.yaml ---
    pick_object_params = load_yaml('dual_ur_moveit_config', 'config/pick_object.yaml')

    # --- 4. 合并所有参数 ---
    parameters = [
        robot_description,
        # {'robot_description': robot_description_content},
        # {'robot_description_semantic': robot_description_semantic_content},
        pick_object_params,
        {'use_sim_time': True}
    ]

    jointprinter_node = Node(
        package="motion_plan",
        executable="jointprinter",
        name="dual_move",
        output="screen",
        parameters=parameters,
    )

    return LaunchDescription([
        jointprinter_node,
        dual_arm_follower_node,
        # rviz_node,
    ])