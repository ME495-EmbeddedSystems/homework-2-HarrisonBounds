"""Python Launch file to control the robot movement."""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch.substitutions import ThisLaunchFileDir

from launch_ros.actions import Node


def generate_launch_description():
    """Generate Launch file."""
    return LaunchDescription([
        Node(
            package='turtle_brick',
            namespace='arena',
            executable='arena_node',
            name='arena_node',
        ),
        Node(
            package='turtle_brick',
            namespace='catcher',
            executable='catcher_node',
            name='catcher_node',
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([ # unnecessary
                PathJoinSubstitution([ThisLaunchFileDir(), 'run_turtle.launch.py'])
            ])
        )
    ])
