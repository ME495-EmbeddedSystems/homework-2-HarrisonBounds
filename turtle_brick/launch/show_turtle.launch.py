"""Python Launch File that shows the turtle robot."""
from launch import LaunchDescription
from launch.substitutions import Command, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import ExecutableInPackage, FindPackageShare


def generate_launch_description():
    """Generate Launch file."""
    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[
                {'robot_description':
                    Command([ExecutableInPackage('xacro', 'xacro'), ' ',
                            PathJoinSubstitution(
                            [FindPackageShare('turtle_brick'), 'turtle.urdf.xacro'])])}
                            ]
            ),
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', PathJoinSubstitution([FindPackageShare('turtle_brick'), 'view_robot.rviz'])]
        )
            ])
