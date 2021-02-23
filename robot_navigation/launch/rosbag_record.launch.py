from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, EnvironmentVariable

from datetime import datetime

def generate_launch_description():
    
    # Currently the output flag doesn't append argument as prefix, but uses the argument as the full path instead
    # The workaround is copied from the rosbag2 source code whereby we give the user the ability to put a directory
    # as the prefix path to the timestamped folder
    output_dir = datetime.now().strftime('rosbag2_%Y_%m_%d-%H_%M_%S')

    # Launch arguments
    launch_arg_rosbag_topics = DeclareLaunchArgument('topics', default_value='-a')
    launch_arg_rosbag_output = DeclareLaunchArgument('output_prefix', default_value=[EnvironmentVariable('HOME'), '/rosbags/'])

    # rosbag2 can only be run as an executable, not node action
    process_rosbag_record = ExecuteProcess(
        cmd=['ros2', 'bag', 'record', LaunchConfiguration('topics'),
             '-o', [LaunchConfiguration('output_prefix'), output_dir]],
        # cmd=['ros2', 'bag', 'record', LaunchConfiguration('topics')],
        output='screen'
    )
    
    return LaunchDescription([
        launch_arg_rosbag_topics,
        launch_arg_rosbag_output,
        process_rosbag_record
    ])