from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch.conditions import IfCondition
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os

from rclpy_utils.utils import urdf_xacro_reader

def generate_launch_description():

    # Package directories
    pkg_robot_navigation = get_package_share_directory('robot_navigation')

    # Launch arguments
    launch_arg_rosbag_file = DeclareLaunchArgument('rosbag_file')
    launch_arg_rviz = DeclareLaunchArgument('rviz', default_value='true')
    # Currently no way to extract launch argument values with LaunchConfiguration
    # launch_arg_urdf = DeclareLaunchArgument(
    #     'urdf_xacro',
    #     description='The path to the robot .urdf.xacro file.',
    #     default_value=[os.path.join(get_package_share_directory('robot_description'), 'urdf', 'generic_roomba_bot.urdf.xacro')]
    # )

    urdf_xacro = os.path.join(get_package_share_directory('robot_description'), 'urdf', 'generic_roomba_bot.urdf.xacro')
    robot_desc = urdf_xacro_reader(urdf_xacro)

    # rosbag2 can only be run as an executable, not node action
    process_rosbag_play = ExecuteProcess(
        cmd=['ros2', 'bag', 'play', LaunchConfiguration('rosbag_file')],
        output='screen'
    )

    # Nodes 
    node_robot_state_pub = Node( # publish the urdf so that RViz gets it
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_desc}]
    )

    node_static_transform_pub = Node( # static transform publisher node to establish odom to map
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', '1', 'map', 'odom']
    )

    node_rviz = Node( # RViz node
        package='rviz2',
        executable='rviz2',
        arguments=['-d', os.path.join(pkg_robot_navigation, 'rviz', 'robot_rosbag_play.rviz')],
        condition=IfCondition(LaunchConfiguration('rviz'))
    )
    
    return LaunchDescription([
        launch_arg_rosbag_file,
        launch_arg_rviz,
        process_rosbag_play,
        node_static_transform_pub,
        node_robot_state_pub,
        node_rviz
    ])