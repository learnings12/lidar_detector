import os 
import xacro 

from ament_index_python.packages import get_package_share_directory 
from launch import LaunchDescription 
from launch.actions import IncludeLaunchDescription 
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():

    robotXacroName = 'differential_drive_model' #Robot name in xacro
    namePackage = "mobile_robot"  #package name
    modelFileRelativePath = 'model/robot.xacro'  #relative path to xacro file 
    
    #Absolute path to the model
    pathModelFile = os.path.join(
        get_package_share_directory(namePackage), modelFileRelativePath
        )
    
    #Robot decription form xacro file 
    robotDescription = xacro.process_file(pathModelFile).toxml()


    #Coustom world

    #launch file from gazebo_ros package
    gazebo_rosPackageLaunch = PythonLaunchDescriptionSource(
        os.path.join(
            get_package_share_directory('ros_gz_sim'), 
            'launch', 'gz_sim.launch.py'
        )
    )

    #World
    gazeboLaunch = IncludeLaunchDescription(
        gazebo_rosPackageLaunch, launch_arguments = {
            'gz_args' : ['-r -v -v4 empty.sdf'], 'on_exit_shutdown':'true'
        }.items()
    )

    ##this for launching own world
    

   

    #Gazebo Nodes 
    spawModelNodeGazebo = Node(
        package = 'ros_gz_sim',
        executable = 'create',
        arguments = [
            '-name', robotXacroName,
            '-topic', 'robot_description'
        ], 
        output = 'screen', 
    )


    nodeRobotStatePublisher = Node(
        package = 'robot_state_publisher', 
        executable = 'robot_state_publisher', 
        output = 'screen', 
        parameters = [{'robot_description': robotDescription, 'use_sim_time': True}]
    )


    #Control the robot from ROS2

    bridge_params = os.path.join(
        get_package_share_directory(namePackage), 
        'parameters', 'bridge_parameters.yaml'
    )

    start_gazebo_ros_bridge_cmd = Node(
        package = 'ros_gz_bridge', 
        executable = 'parameter_bridge',
        arguments = [
            '--ros-args', '-p', f'config_file:={bridge_params}'
        ], 
        output = 'screen',
    )

    #Empty launch description 
    launchDescriptionObject = LaunchDescription()

    #gazebolaunch
    launchDescriptionObject.add_action(gazeboLaunch)

    launchDescriptionObject.add_action(spawModelNodeGazebo)
    launchDescriptionObject.add_action(nodeRobotStatePublisher)
    launchDescriptionObject.add_action(start_gazebo_ros_bridge_cmd)

    return launchDescriptionObject