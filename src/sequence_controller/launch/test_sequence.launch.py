from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # RELbot Simulator
        Node(
            package='relbot_simulator',
            executable='relbot_simulator',
            name='relbot_simulator',
            parameters=[{
                'image_stream_FPS': 30.0,
                'visual_frequency_turtlesim': 62.5
            }]
        ),
        
        # Sequence Controller
        Node(
            package='sequence_controller',
            executable='sequence_controller',
            name='sequence_controller'
        ),
        
        # Turtlesim for visualization
        Node(
            package='turtlesim',
            executable='turtlesim_node',
            name='turtlesim'
        ),
    ])