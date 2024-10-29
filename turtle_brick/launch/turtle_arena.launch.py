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
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([ThisLaunchFileDir(), 'run_turtle.launch.py'])
            ])
        )   
   ])