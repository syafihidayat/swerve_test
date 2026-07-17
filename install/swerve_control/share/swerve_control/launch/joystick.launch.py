import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_control = get_package_share_directory('swerve_control')
    joy_config = os.path.join(pkg_control, 'config', 'joystick.yaml')

    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        output='screen',
    )

    teleop_node = Node(
        package='teleop_twist_joy',
        executable='teleop_node',
        name='teleop_twist_joy_node',
        output='screen',
        parameters=[joy_config],
    )

    return LaunchDescription([
        joy_node,
        teleop_node,
    ])
