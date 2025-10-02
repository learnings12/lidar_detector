import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Path to your robot.rviz file
    pkg_mobile_robot = get_package_share_directory('mobile_robot')
    rviz_config_file = os.path.join(pkg_mobile_robot, 'rviz', 'robot.rviz')

    # Launch RViz2 with the given config
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen'
    )

    return LaunchDescription([rviz_node])
