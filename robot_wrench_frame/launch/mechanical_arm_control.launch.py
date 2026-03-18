from launch import LaunchDescription
from launch_ros.actions import Node

from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # left_or_right_arg = DeclareLaunchArgument(
    #     'left_or_right',
    #     default_value='left',
    #     description='left_or_right controller'
    # )
    return LaunchDescription([        
        Node(
            package='robot_wrench_frame',  # 替换为你的包名
            executable='mechanical_arm_control',           # 替换为你的可执行文件名（如 CMakeLists.txt 中设置的目标）
            name='mechanical_arm_control',  # 节点名，可自定义
            output='screen',                   # 日志输出到终端
                                  # 如果有需要，可以在这里传入参数
        ),
    ])
