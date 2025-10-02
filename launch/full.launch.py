import os 

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():

    #Package path
    pkg_mobile_robot = get_package_share_directory('mobile_robot')

    #Path to gazebo launch
    gazebo_launch = os.path.join(pkg_mobile_robot, 'launch', 'gazebo_model.launch.py')

    #Lidar launch
    lidar = os.path.join(pkg_mobile_robot, 'launch', 'scan_fixed.launch.py')

    rviz = os.path.join(pkg_mobile_robot, 'launch' , 'rviz.launch.py')
    #Inculde gazebo launch
    gazebo_launch_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gazebo_launch)
    )
    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(lidar)
    )
    rviz_launch = IncludeLaunchDescription(PythonLaunchDescriptionSource(rviz))

    ld = LaunchDescription()
    ld.add_action(gazebo_launch_include)
    ld.add_action(lidar_launch)
    ld.add_action(rviz_launch)

    return ld 
