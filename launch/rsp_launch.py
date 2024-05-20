import os
import xacro

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    package_name = 'robot_system'
    xacro_file = os.path.join(get_package_share_directory(package_name), 'description', 'robot.xacro')
    robot_dec = xacro.process_file(xacro_file).toxml()
    params = {'robot_description': robot_dec}

    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[params]
    )

    return LaunchDescription([
        node_robot_state_publisher
    ])