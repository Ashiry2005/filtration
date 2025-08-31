from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='pre_interview',
            executable='vehicle_model',
            name='vehicle_model',
            output='screen'
        ),
        Node(
            package='pre_interview',
            executable='controller',
            name='controller',
            output='screen'
        ),
        Node(
            package='pre_interview',
            executable='plot',
            name='plot',
            output='screen'
        ),
    ])
