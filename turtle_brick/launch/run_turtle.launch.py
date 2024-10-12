from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir, PathJoinSubstitution

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
    
    
        # Include another launch file
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([ThisLaunchFileDir(), 'show_turtle.launch.py'])
            ])
        )   
   ])