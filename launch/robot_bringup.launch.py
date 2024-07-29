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
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_pub_laser',
            arguments=['-0.057158', '0', '0.02','3.14159265', '0', '0','base_link','laser'],
        ),
    ])