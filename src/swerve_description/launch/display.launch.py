import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import FindExecutable


def generate_launch_description():
    pkg_description = get_package_share_directory('swerve_description')

    model_arg = DeclareLaunchArgument(
        'model',
        default_value=os.path.join(pkg_description, 'urdf','swerve_drive','swerve_drive_robot.urdf.xacro')
    )
    gui_arg = DeclareLaunchArgument('gui', default_value='false')
    # rvizconfig_arg = DeclareLaunchArgument(
    #     'rvizconfig',
    #     default_value=os.path.join(pkg_description, 'rviz', 'confirm_urdf.rviz')
    # )

    # robot_description = ParameterValue(
    #     Command(['xacro', LaunchConfiguration('model')]),
    #     value_type=str
    # )

    robot_description = ParameterValue(
    Command([
        FindExecutable(name='xacro'),
        ' ',
        LaunchConfiguration('model')
    ]),
    value_type=str
    )

    # robot_state_publisher = Node(
    #     package='robot_state_publisher',
    #     executable='robot_state_publisher',
    #     name='robot_state_publisher',
    #     parameters=[{'robot_description': robot_description}]
    # )


    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        emulate_tty=True,
        parameters=[{
        'robot_description': robot_description}]
    )

    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        condition=UnlessCondition(LaunchConfiguration('gui'))
    )

    joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        condition=IfCondition(LaunchConfiguration('gui'))
    )

    # rviz = Node(
    #     package='rviz2',
    #     executable='rviz2',
    #     name='rviz2',
    #     arguments=['-d', LaunchConfiguration('rvizconfig')]
    # )
    rviz = Node(
    package='rviz2',
    executable='rviz2',
    name='rviz2'
    )

    return LaunchDescription([
        model_arg,
        gui_arg,
        # rvizconfig_arg,
        robot_state_publisher,
        joint_state_publisher,
        joint_state_publisher_gui,
        rviz,
    ])
