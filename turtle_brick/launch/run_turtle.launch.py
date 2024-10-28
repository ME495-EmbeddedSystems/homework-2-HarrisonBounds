from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir, PathJoinSubstitution
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.substitutions import ExecutableInPackage, FindPackageShare

def generate_launch_description():
   return LaunchDescription([
    Node(
        package='turtlesim',
        namespace='turtlesim1',
        executable='turtlesim_node',
        name='sim',
        parameters=[{'holonomic': True}]
        ),
    
    Node(
        package='turtle_brick',
        namespace='turtle_bot',
        executable='turtle_bot_node',
        name='turtle_bot_node',
    ),
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
    #     Node(
    #     package='turtle_brick',
    #     namespace='test',
    #     executable='test_node',
    #     name='test_node',
    # ),
    Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[
                {"robot_description" :
                Command([ExecutableInPackage("xacro", "xacro"), " ",
                        PathJoinSubstitution(
                        [FindPackageShare("turtle_brick"), "turtle.urdf.xacro"])])}
                        ]
            ),
    
    
        # Include another launch file
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([ThisLaunchFileDir(), 'show_turtle.launch.py'])
            ])
        )   
   ])