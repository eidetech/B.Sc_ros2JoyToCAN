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
            executable='kinematics',
        ),
        Node(
            package='joy2can',
            executable='odriveCAN.py',
            output='screen',
	    emulate_tty=True,
	    arguments=[('__log_level:=debug')]
        ),
        Node(
            package='joy2can',
            executable='app.py',
            output='screen',
	    emulate_tty=True,
	    arguments=[('__log_level:=debug')]
        ),

    ])
