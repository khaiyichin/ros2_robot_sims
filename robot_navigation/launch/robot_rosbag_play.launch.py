from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution, TextSubstitution
from launch.conditions import IfCondition, LaunchConfigurationEquals
from launch_ros.actions import Node
from launch.launch_description_sources import FrontendLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

import os

from rclpy_utils.utils import urdf_xacro_reader

def generate_launch_description():

    # Package directories
    pkg_robot_navigation = get_package_share_directory('robot_navigation')
    pkg_clock_publisher = get_package_share_directory('clock_publisher')

    # Launch arguments
    launch_arg_rosbag_file = DeclareLaunchArgument('rosbag_file')
    launch_arg_rviz = DeclareLaunchArgument('rviz', default_value='true')
    launch_arg_use_sim_time = DeclareLaunchArgument('use_sim_time', default_value='true')
    launch_arg_rate = DeclareLaunchArgument('rate', default_value='1')
    # Currently no way to extract launch argument values with LaunchConfiguration
    # launch_arg_urdf = DeclareLaunchArgument(
    #     'urdf_xacro',
    #     description='The path to the robot .urdf.xacro file.',
    #     default_value=[os.path.join(get_package_share_directory('robot_description'), 'urdf', 'generic_roomba_bot.urdf.xacro')]
    # )

    urdf_xacro = os.path.join(get_package_share_directory('robot_description'), 'urdf', 'generic_roomba_bot.urdf.xacro')
    robot_desc = urdf_xacro_reader(urdf_xacro)

    # rosbag2 can only be run as an executable, not node action
    process_rosbag_play_sim_time = ExecuteProcess(
        cmd=['ros2', 'bag', 'play', LaunchConfiguration('rosbag_file'),
             '--rate', LaunchConfiguration('rate'),
             '--remap', '/clock:=/clock_real'],
        output='screen',
        condition=LaunchConfigurationEquals('use_sim_time', 'true')
    )

    process_rosbag_play_real_time = ExecuteProcess(
        cmd=['ros2', 'bag', 'play', LaunchConfiguration('rosbag_file'),
             '--rate', LaunchConfiguration('rate')],
        output='screen',
        condition=LaunchConfigurationEquals('use_sim_time', 'false')
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

    # Temporary workaround to using simulated time for rosbag playback in ros2
    launch_clock_publisher = IncludeLaunchDescription(
        FrontendLaunchDescriptionSource(os.path.join(pkg_clock_publisher, 'launch', 'clock_publisher.launch.xml')),
        condition=IfCondition(LaunchConfiguration('use_sim_time'))
    )
    
    return LaunchDescription([
        launch_arg_rosbag_file,
        launch_arg_rviz,
        launch_arg_use_sim_time,
        launch_arg_rate,
        process_rosbag_play_sim_time,
        process_rosbag_play_real_time,
        node_static_transform_pub,
        node_robot_state_pub,
        node_rviz,
        launch_clock_publisher
    ])