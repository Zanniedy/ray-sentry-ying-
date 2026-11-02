import subprocess
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    package_share = Path(get_package_share_directory('rm_9_2'))
    xacro_path = package_share / 'urdf' / 'RM_9.2.xacro'
    rviz_config_path = package_share / 'config' / 'RM_9.2.rviz'

    result = subprocess.run(
        ['/opt/ros/humble/bin/xacro', str(xacro_path)],
        capture_output=True, text=True, check=True,
    )
    robot_description = result.stdout

    return LaunchDescription([
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            output='screen',
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{'robot_description': robot_description}],
            output='screen',
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', str(rviz_config_path)],
            output='screen',
        ),
    ])