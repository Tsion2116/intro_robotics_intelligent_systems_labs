import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    ExecuteProcess,
    RegisterEventHandler,
    TimerAction,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():

    diffdrive_share = get_package_share_directory('diffdrive_bot')

    control_launch = os.path.join(
        diffdrive_share,
        'launch',
        'control.launch.py'
    )

    square_driver = Node(
        package='my_robot_nodes',
        executable='square_driver',
        parameters=[{
            'side_length': ParameterValue(
                LaunchConfiguration('side_length'),
                value_type=float
            )
        }],
        output='screen'
    )

    # Wait until Gazebo starts publishing odometry.
    wait_for_odom = ExecuteProcess(
        cmd=['ros2', 'topic', 'echo', '--once', '/odom'],
        name='wait_for_first_odom',
        output='log'
    )

    # Start square_driver one second after the first odometry message.
    start_square_after_odom = RegisterEventHandler(
        OnProcessExit(
            target_action=wait_for_odom,
            on_exit=[
                TimerAction(
                    period=1.0,
                    actions=[square_driver]
                )
            ]
        )
    )

    return LaunchDescription([

        DeclareLaunchArgument(
            'side_length',
            default_value='0.5',
            description='Length of each square side in metres'
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(control_launch)
        ),

        Node(
            package='my_robot_nodes',
            executable='rotate_server',
            output='screen'
        ),

        start_square_after_odom,
        wait_for_odom,

    ])
