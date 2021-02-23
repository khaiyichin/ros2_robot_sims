from ament_index_python.packages import get_package_share_directory

import os
import subprocess

def read_file(path):
    with open(path, 'r') as f:
        contents = f.read()
    return contents

def urdf_xacro_reader(urdf_xacro):
    """Parse and obtain contents from .urdf.xacro files.

    This is useful especially in generating the robot description for the 
    robot_state_publisher node.

    Args:
        urdf_xacro (str): Absolute path to the .urdf.xacro file.

    Returns:
        A string containing the contents of the .urdf.xacro file.
    """
    
    robot_urdf = urdf_xacro[:-6]
    xacro_proc = subprocess.Popen("xacro {0} > {1}".format(urdf_xacro, robot_urdf),
        stderr=subprocess.STDOUT, shell=True)
    xacro_proc.wait()
    assert os.path.exists(robot_urdf)
    urdf_contents = read_file(robot_urdf)

    return urdf_contents