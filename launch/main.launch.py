
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='robocar_human_follow',
            executable='human_node',
            output='screen'),
    ])
