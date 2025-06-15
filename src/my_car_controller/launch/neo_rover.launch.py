from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            output='screen'
        ),
        Node(
            package='teleop_twist_joy',
            executable='teleop_node',
            name='teleop_twist_joy_node',
            output='screen',
            parameters=['/home/pi/ros2_ws/src/joy_config_pkg/config/teleop.yaml']
        ),
        Node(
            package='my_car_controller',
            executable='serial_bridge_2',
            name='serial_bridge_2',
            output='screen'
        )
    ])

