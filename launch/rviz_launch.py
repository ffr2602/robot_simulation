import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    package_name = 'robot_simulation'
    path_pkg = os.path.join(get_package_share_directory(package_name))
    rviz_bot = os.path.join(path_pkg, 'rviz', 'view_bot.rviz')

    node_rviz_bot = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=['-d', rviz_bot]
    )

    return LaunchDescription([
        node_rviz_bot
    ])
