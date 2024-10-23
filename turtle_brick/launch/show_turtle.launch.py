from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.substitutions import ExecutableInPackage, FindPackageShare

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[
                {"robot_description" :
                Command([ExecutableInPackage("xacro", "xacro"), " ",
                        PathJoinSubstitution(
                        [FindPackageShare("turtle_brick"), "turtle.urdf"])])}
                        ]
            ),
        Node(
                package="joint_state_publisher",
                executable="joint_state_publisher"
            ),
        Node(
            package="rviz2",
            executable="rviz2",
            arguments=["-d", PathJoinSubstitution([FindPackageShare("turtle_brick"), "view_robot.rviz"])]
        )
                        ])