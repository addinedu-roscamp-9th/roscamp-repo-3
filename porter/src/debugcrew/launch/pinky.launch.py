from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """Launch vel_sub and pid_node together."""
    return LaunchDescription([
        Node(
            package="debugcrew",
            executable="vel_sub",
            name="vel_sub",
            output="screen",
        ),
        Node(
            package="debugcrew",
            executable="pid_node",
            name="pid_node",
            output="screen",
        ),
    ])
