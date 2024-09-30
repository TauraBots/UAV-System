import launch
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='interface_package',
            executable='interface_node',
            name='interface_node',
            output='screen'
        ),
        Node(
            package='mission_package',  
            executable='mission_node', 
            name='mission_node',
            output='screen'
        )
    ])
