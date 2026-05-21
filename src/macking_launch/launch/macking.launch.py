from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    return LaunchDescription([

        Node(
            package='vision_node',
            executable='vision_node',
            output='screen'
        ),

        Node(
            package='joy',
            executable='joy_node',
            output='screen'
        ),

        Node(
            package='joystick_control',
            executable='joystick_control',
            output='screen'
        ),

        Node(
            package='influx_node',
            executable='telemetry_node',
            output='screen'
        ),

    ])
