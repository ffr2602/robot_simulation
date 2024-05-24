import os
import xacro

from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource 
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    package_name = 'robot_simulation'
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

    gazebo_params_file = os.path.join(get_package_share_directory(package_name), 'config', 'gazebo_params.yaml')
    gazebo_params_maps = os.path.join(get_package_share_directory(package_name), 'world', 'map_maze.world')
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
        launch_arguments={
            'extra_gazebo_args': '--ros-args --params-file ' + gazebo_params_file,
            'world': gazebo_params_maps
        }.items()
    )

    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'my_bot'],
        output='screen'
    )

    node_controller = Node(
        package=package_name,
        executable='omni_controller.py',
        name='robot_controller'
    )

    twist_mux_params = os.path.join(get_package_share_directory(package_name), 'config', 'twist_mux.yaml')
    twist_mux = Node(
        package="twist_mux",
        executable="twist_mux",
        parameters=[twist_mux_params],
        remappings=[('/cmd_vel_out', '/omni_cont/cmd_vel_unstamped')]
    )

    control_params = os.path.join(get_package_share_directory(package_name), 'config', 'control_params.yaml')
    node_tracking = Node(
        package=package_name,
        executable='path_tracking.py',
        name='path_tracking',
        parameters=[control_params]
    )

    rviz_bot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(get_package_share_directory(package_name), 'launch', 'rviz_launch.py')]),
    )

    rosbridge = IncludeLaunchDescription(
        XMLLaunchDescriptionSource([os.path.join(get_package_share_directory(package_name), 'launch', 'rosbridge_websocket_launch.xml')])
    )

    return LaunchDescription([
        node_robot_state_publisher,
        node_controller,
        rviz_bot,
        rosbridge,
        twist_mux,
        node_tracking
        # gazebo,
        # spawn_entity
    ])
