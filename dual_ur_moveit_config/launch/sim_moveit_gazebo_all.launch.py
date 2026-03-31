import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, ExecuteProcess
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue

try:
    from ur_moveit_config.launch_common import load_yaml
except ImportError:
    def load_yaml(package_name, file_path):
        import yaml
        share_dir = FindPackageShare(package_name).find(package_name)
        full_path = os.path.join(share_dir, file_path)
        with open(full_path, 'r') as f:
            return yaml.safe_load(f)


def launch_setup(context, *args, **kwargs):
    # --- 1. 获取启动参数 ---
    description_package = LaunchConfiguration("description_package")
    description_file = LaunchConfiguration("description_file")
    moveit_config_package = LaunchConfiguration("moveit_config_package")
    moveit_config_file = LaunchConfiguration("moveit_config_file")
    moveit_joint_limits_file = LaunchConfiguration("moveit_joint_limits_file")
    use_sim_time = LaunchConfiguration("use_sim_time")
    launch_controllers = LaunchConfiguration("launch_controllers")
    launch_rviz = LaunchConfiguration("launch_rviz")
    warehouse_sqlite_path = LaunchConfiguration("warehouse_sqlite_path")

    # --- 2. 加载机器人描述 (URDF) ---
    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name="xacro")]), " ",
        PathJoinSubstitution([FindPackageShare(description_package), "urdf", description_file])
    ])
    robot_description = {"robot_description": ParameterValue(robot_description_content, value_type=str)}

    # --- 3. 加载 MoveIt 语义描述 (SRDF) ---
    robot_description_semantic_content = Command([
        PathJoinSubstitution([FindExecutable(name="xacro")]), " ",
        PathJoinSubstitution([FindPackageShare(moveit_config_package), "config", moveit_config_file])
    ])
    robot_description_semantic = {"robot_description_semantic": ParameterValue(robot_description_semantic_content, value_type=str)}

    # --- 4. 加载 MoveIt 核心配置文件 ---
    # 运动学配置
    robot_description_kinematics = PathJoinSubstitution(
        [FindPackageShare(moveit_config_package), "config", "kinematics.yaml"]
    )

    # 关节限制配置
    robot_description_planning = {
        "robot_description_planning": load_yaml(
            str(moveit_config_package.perform(context)),
            os.path.join("config", str(moveit_joint_limits_file.perform(context))),
        )
    }

    # OMPL 规划器配置
    ompl_planning_pipeline_config = {
        "move_group": {
            "planning_plugin": "ompl_interface/OMPLPlanner",
            "request_adapters": """default_planner_request_adapters/AddTimeOptimalParameterization default_planner_request_adapters/FixWorkspaceBounds default_planner_request_adapters/FixStartStateBounds default_planner_request_adapters/FixStartStateCollision default_planner_request_adapters/FixStartStatePathConstraints""",
            "start_state_max_bounds_error": 0.1,
        }
    }
    ompl_planning_yaml = load_yaml(str(moveit_config_package.perform(context)), "config/ompl_planning.yaml")
    ompl_planning_pipeline_config["move_group"].update(ompl_planning_yaml)

    # 控制器映射配置 (MoveIt -> ros2_control)
    controllers_yaml = load_yaml(str(moveit_config_package.perform(context)), "config/moveit_controllers.yaml")
    moveit_controllers = {
        "moveit_simple_controller_manager": controllers_yaml,
        "moveit_controller_manager": "moveit_simple_controller_manager/MoveItSimpleControllerManager",
    }

    # 轨迹执行参数
    trajectory_execution = {
        "moveit_manage_controllers": False,
        "trajectory_execution.allowed_execution_duration_scaling": 6.0,
        "trajectory_execution.allowed_goal_duration_margin": 0.5,
        "trajectory_execution.allowed_start_tolerance": 0.01,
    }

    # 规划场景监控参数
    planning_scene_monitor_parameters = {
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
        "publish_robot_description": True,
        "publish_robot_description_semantic": True,
        "use_sim_time": use_sim_time,
    }

    # 数据库配置
    warehouse_ros_config = {
        "warehouse_plugin": "warehouse_ros_sqlite::DatabaseConnection",
        "warehouse_host": warehouse_sqlite_path,
    }

    # # --- 5. 启动 Gazebo 仿真环境 ---
    # gazebo = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         PathJoinSubstitution([FindPackageShare("gazebo_ros"), "launch", "gazebo.launch.py"])
    #     ),
    #     launch_arguments={
    #         "world": PathJoinSubstitution([FindPackageShare("ur_description"), "gazebo", "empty.world"]),
    #         "verbose": "true",
    #         "use_sim_time": use_sim_time
    #     }.items(),
    # )

    gazebo_world_file = os.path.join(
        FindPackageShare(package='ur_description').find('ur_description'),
        'gazebo','empty.world'
    )
    gazebo = ExecuteProcess(
        cmd=['gazebo', '--verbose', gazebo_world_file, '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so'], 
        output='screen',
    )

    # --- 6. 生成机器人实体到 Gazebo ---
    gazebo_spawn_robot = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        name="spawn_ur",
        arguments=["-entity", "dual_arm", "-topic", "robot_description", "-timeout", "120"],
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
    )

    # --- 7. 发布机器人状态 (TF 树) ---
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description, {"use_sim_time": use_sim_time}],
    )

    # --- 8. 激活 ros2_control 控制器 ---
    # 注意：在 Gazebo 中，controller_manager 由 gazebo_ros 插件启动，此处仅负责激活具体控制器
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

    # --- 9. 启动 MoveGroup 节点 ---
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            robot_description_planning,
            ompl_planning_pipeline_config,
            trajectory_execution,
            moveit_controllers,
            planning_scene_monitor_parameters,
            {"use_sim_time": use_sim_time},
            warehouse_ros_config,
        ],
    )

    # --- 10. 启动 RViz (带 MoveIt 插件) ---
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare(moveit_config_package), "config", "moveit.rviz"]
    )
    rviz_node = Node(
        package="rviz2",
        condition=IfCondition(launch_rviz),
        executable="rviz2",
        name="rviz2_moveit",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[
            robot_description,
            robot_description_semantic,
            ompl_planning_pipeline_config,
            robot_description_kinematics,
            robot_description_planning,
            warehouse_ros_config,
            {"use_sim_time": use_sim_time}
        ],
    )

    # 组合并返回所有节点
    return [
        gazebo,
        robot_state_publisher,
        gazebo_spawn_robot,
    ] + controller_spawners + [
        move_group_node,
        rviz_node
    ]


def generate_launch_description():
    # 简明定义所有启动参数
    args = [
        DeclareLaunchArgument("description_package", default_value="ur_description"),
        DeclareLaunchArgument("description_file", default_value="test_new.urdf"),
        DeclareLaunchArgument("moveit_config_package", default_value="dual_ur_moveit_config"),
        DeclareLaunchArgument("moveit_config_file", default_value="dual_arm.srdf"),
        DeclareLaunchArgument("moveit_joint_limits_file", default_value="joint_limits.yaml"),
        DeclareLaunchArgument("warehouse_sqlite_path", default_value=os.path.expanduser("~/.ros/warehouse_ros.sqlite")),
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("launch_controllers", default_value="true"),
        DeclareLaunchArgument("launch_rviz", default_value="true"),
    ]
    return LaunchDescription(args + [OpaqueFunction(function=launch_setup)])