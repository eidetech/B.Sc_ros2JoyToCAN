from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='joy',
            executable='joy_node',
        ),
        Node(
            package='joy2can',
            executable='tranceiver',
        ),
        Node(
            package='joy2can',
            executable='odriveCAN.py',
            output='screen',
	    emulate_tty=True,
	    arguments=[('__log_level:=debug')]
        ),

    ])
