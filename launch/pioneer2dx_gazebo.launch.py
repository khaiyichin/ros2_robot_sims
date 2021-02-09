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
    pkg_ros2_robot_sims = get_package_share_directory('ros2_robot_sims')
    pkg_gazebo_models_worlds_collection = get_package_share_directory('gazebo_models_worlds_collection')


    urdf_dir =  os.path.join(pkg_ros2_robot_sims, 'urdf')
    xacro_urdf = os.path.join(urdf_dir, 'generic_roomba_bot.urdf.xacro')
    robot_urdf = os.path.join(urdf_dir, 'generic_roomba_bot.urdf')
    xacro_proc = subprocess.Popen("xacro {0} > {1}".format(xacro_urdf, robot_urdf) ,
        stdout=subprocess.PIPE, stderr=subprocess.STDOUT, shell=True)
    xacro_proc.wait()
    assert os.path.exists(robot_urdf)
    urdf_contents = read_file(robot_urdf)

    # Gazebo node
    node_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py'))
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

    launch_arg_gazebo = DeclareLaunchArgument(
        'world',
        default_value=[os.path.join(pkg_gazebo_models_worlds_collection, 'worlds', 'office_small.world'), ''],
        description='SDF world file'
    )
    
    return LaunchDescription([launch_arg_gazebo, node_spawn_entity, node_gazebo])