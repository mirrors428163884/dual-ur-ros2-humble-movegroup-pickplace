import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    package_name = 'static_broadcaster_listener'
    pkg_share = get_package_share_directory(package_name)
    print(pkg_share)
    parameters_file_path = os.path.join(pkg_share, 'config', 'static_transforms.yaml')

    return LaunchDescription([
        Node(
            package=package_name,
            executable='publisher_listener_node',
            name='publisher_listener_node',
            parameters=[parameters_file_path],
            output='screen',
        ),
    ])