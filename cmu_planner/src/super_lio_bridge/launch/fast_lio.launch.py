import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    config_path = os.path.join(
        get_package_share_directory('super_lio_bridge'),
        'config', 'fast_lio.yaml'
    )

    bridge_node = Node(
        package='super_lio_bridge',
        executable='super_lio_bridge',
        name='fast_lio_bridge',
        output='screen',
        parameters=[config_path]
    )

    return LaunchDescription([bridge_node])
