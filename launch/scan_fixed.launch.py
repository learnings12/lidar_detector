from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="mobile_robot",
            executable="lidar",
            name="lidar",
            output="screen",
            remappings=[("/scan_fixed", "/scan")]
        ), 

        Node(
            package="mobile_robot",
            executable="lidar_radius",
            name="lidar_radius",
            output="screen"
        ), 

        Node(
            package="mobile_robot",
            executable="point",
            name="point",
            output="screen"
        ), 
    ])
