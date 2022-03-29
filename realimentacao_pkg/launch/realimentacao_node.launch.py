from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            name='realimentacao_node',
            package='realimentacao_pkg',
            executable='realimentacao_node',
            output='screen'
        )
    ])