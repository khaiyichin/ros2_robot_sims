# Required python launch modules
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

import os
import subprocess

def read_file(path):
    with open(path, 'r') as f:
        contents = f.read()
    return contents

# Launch description
def generate_launch_description():
    
    # Define package share directory paths
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_robot_description = get_package_share_directory('robot_description')
    pkg_gazebo_models_worlds_collection = get_package_share_directory('gazebo_models_worlds_collection')


    urdf_dir =  os.path.join(pkg_robot_description, 'urdf')
    xacro_urdf = os.path.join(urdf_dir, 'generic_roomba_bot.urdf.xacro')
    robot_urdf = os.path.join(urdf_dir, 'generic_roomba_bot.urdf')
    xacro_proc = subprocess.Popen("xacro {0} > {1}".format(xacro_urdf, robot_urdf) ,
        stdout=subprocess.PIPE, stderr=subprocess.STDOUT, shell=True)
    xacro_proc.wait()
    assert os.path.exists(robot_urdf)
    urdf_contents = read_file(robot_urdf)

    # Publish the urdf so that RViz gets it
    node_robot_state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='both',
        arguments=[robot_urdf]
    )

    # Gazebo launch file
    launch_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py')),
    )

    # RViz node
    node_rviz = Node(
        package='rviz2',
        executable='rviz2',
        # arguments=['-d', os.path.join(pkg_dolly_gazebo, 'rviz', 'dolly_gazebo.rviz')],
        # condition=IfCondition(LaunchConfiguration('rviz'))
    )

    # Spawn entity node
    node_spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'myentity',
                   '-x', '0',
                   '-y', '0',
                   '-z', '0',
                   '-file', robot_urdf]
    )

    # Static transform publisher node to establish odom to map
    node_static_transform_pub = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', '1', 'map', 'odom']
    )

    launch_arg_gazebo_world = DeclareLaunchArgument(
        'world',
        default_value=[os.path.join(pkg_gazebo_models_worlds_collection, 'worlds', 'office_small.world'), ''],
        description='SDF world file'
    )

    launch_arg_gazebo_verbose = DeclareLaunchArgument(
        'verbose',
        default_value='true',
        description='Set true for gazebo messages'
    )
    
    return LaunchDescription([
        launch_arg_gazebo_world,
        launch_arg_gazebo_verbose,
        node_spawn_entity,
        launch_gazebo,
        node_rviz,
        node_static_transform_pub,
        node_robot_state_pub        
    ])