import os
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    package_name='mobile_project_pkg'

    mecanum_node = Node(
        package='mobile_project_pkg',
        executable='mecanum_bot.py',
        name='mecanum_bot',
        output='screen',
        prefix='python3',  # Specify the Python interpreter
    )
    Bridge = Node(
        package='mobile_project_pkg',
        executable='bridge.py',
        name='bridge',
        prefix='python3',  # Specify the Python interpreter
    )
    launch_sim_launch = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','launch_sim.launch.py'
                )]),
    )

    joy_stick_launch = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','joystick.launch.py'
                )]),
    )

    return LaunchDescription([
       mecanum_node,
       Bridge,
       launch_sim_launch,
       joy_stick_launch
        ])
