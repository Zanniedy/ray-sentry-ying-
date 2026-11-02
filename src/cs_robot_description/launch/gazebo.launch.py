import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('rm_9_2')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    sim_config_path = os.path.join(pkg_share, 'config', 'sim.yaml')
    with open(sim_config_path) as f:
        sim_config = yaml.safe_load(f)
    world_name = sim_config['world']

    pkg_share_parent = os.path.dirname(pkg_share)
    world_models = os.path.join(pkg_share, 'world', 'models')
    existing = os.environ.get('IGN_GAZEBO_RESOURCE_PATH', '')
    parts = [pkg_share_parent, world_models] + ([existing] if existing else [])
    resource_path = os.pathsep.join(parts)

    world_sdf = os.path.join(pkg_share, 'world', 'worlds', world_name + '.sdf')
    ign_config = os.path.join(pkg_share, 'world', 'ign', 'gui.config')

    set_resource_path = SetEnvironmentVariable(
        name='IGN_GAZEBO_RESOURCE_PATH',
        value=resource_path,
    )

    # gz_ros2_control-system.so 在 /opt/ros/humble/lib，必须加到 Gazebo 服务端的插件路径，
    # 只在 spawn_robots.launch.py 里设置对服务端无效
    set_plugin_path = SetEnvironmentVariable(
        name='IGN_GAZEBO_SYSTEM_PLUGIN_PATH',
        value=os.pathsep.join(['/opt/ros/humble/lib', os.environ.get('IGN_GAZEBO_SYSTEM_PLUGIN_PATH', '')]),
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_version': '6',
            'gz_args': world_sdf + ' --gui-config ' + ign_config,
        }.items(),
    )

    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock'],
        output='screen',
    )

    return LaunchDescription([
        set_resource_path,
        set_plugin_path,
        gazebo,
        clock_bridge,
    ])
