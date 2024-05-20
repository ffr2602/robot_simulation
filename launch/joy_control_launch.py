import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node'
    )

    teleop_node = Node(
        package='robot_system',
        executable='joy_control.py',
        name='joy_control_node',
        parameters=[{
            'enable_button_require' : 5,
            'data_scale_x' : 0.5,
            'data_scale_y' : 0.5,
            'data_scale_z' : 1.0
        }],
        remappings=[('/cmd_vel','/cmd_vel_joy')]
    )

    return LaunchDescription([
        joy_node,
        teleop_node,   
    ])