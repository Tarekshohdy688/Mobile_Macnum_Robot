from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')

    joy_params = os.path.join(get_package_share_directory('mobile_project_pkg'),'config','joystick.yaml')
    joy_params_gazebo = os.path.join(get_package_share_directory('mobile_project_pkg'),'config','joystick_gazebo.yaml')
 
    joy_node = Node(
            package='joy',
            executable='joy_node',
            parameters=[joy_params, {'use_sim_time': use_sim_time}],
         )

    teleop_node = Node(
            package='teleop_twist_joy',
            executable='teleop_node',
            name='teleop_node',
            parameters=[joy_params, {'use_sim_time': use_sim_time}],
            remappings=[('/cmd_vel','/diff_cont/cmd_vel_unstamped')]
         )
    teleop_node2 = Node(
            package='teleop_twist_joy',
            executable='teleop_node',
            name='teleop_node',
            parameters=[joy_params_gazebo, {'use_sim_time': use_sim_time}],
            # remappings=[('/cmd_vel','/diff_cont/cmd_vel_unstamped')]
        )
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use sim time if true'),
        joy_node,
        teleop_node,
        teleop_node2,
    ])