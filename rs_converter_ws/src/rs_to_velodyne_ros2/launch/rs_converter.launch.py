from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='rs_converter',
            executable='rs_converter',
            name='rs_converter',
            output='screen',
            arguments=['XYZIRT', 'XYZIRT']
        )
    ])
