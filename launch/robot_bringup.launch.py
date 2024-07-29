from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='robot_bringup',
            executable='robot_bringup',
            name='bringup',
            output='screen'
        ),
        Node(
            package='robot_bringup',
            executable='robot_odom',
            name='odom',
            output='screen'
        ),
    ])