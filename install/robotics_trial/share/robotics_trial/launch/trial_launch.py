from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='robotics_trial',       
            executable='map_publisher',      
            # name='my_custom_node',      
            output='screen'
        ),
        Node(
            package='robotics_trial',       
            executable='start_node_publisher',       
            # name='my_custom_node',      
            output='screen'
        ),
        Node(
            package='robotics_trial',       
            executable='goal_node_publisher',       
            # name='my_custom_node',      
            output='screen'
        ),
        Node(
            package='robotics_trial',       
            executable='path_compute',       
            # name='my_custom_node',      
            output='screen'
        )
    ]) 