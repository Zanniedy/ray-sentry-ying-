import os
import subprocess

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, SetEnvironmentVariable, TimerAction
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('rm_9_2')

    sim_config_path = os.path.join(pkg_share, 'config', 'sim.yaml')
    with open(sim_config_path) as f:
        sim_config = yaml.safe_load(f)
    robot = sim_config['robot']

    xacro_path = os.path.join(pkg_share, 'urdf', 'RM_9.2.xacro')

    # 用系统 xacro 命令处理，避免 conda Python 环境下 yaml 模块缺失导致展开失败
    result = subprocess.run(
        ['/opt/ros/humble/bin/xacro', xacro_path],
        capture_output=True, text=True, check=True,
    )
    robot_description = result.stdout

    # gz_ros2_control-system.so 安装在 /opt/ros/humble/lib，
    # ros_gz_sim launch 不会自动把它加到 Ignition 插件搜索路径，需手动补上
    ros_lib = '/opt/ros/humble/lib'
    set_plugin_path = SetEnvironmentVariable(
        name='IGN_GAZEBO_SYSTEM_PLUGIN_PATH',
        value=ros_lib + ':' + os.environ.get('IGN_GAZEBO_SYSTEM_PLUGIN_PATH', ''),
    )

    remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]

    # 放入独立 namespace，避免与同机其他项目的 robot_state_publisher 冲突
    # gazebo_plugins.xacro 里的 robot_param_node 指向 /rm92/robot_state_publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace='rm92',
        parameters=[{
            'use_sim_time': True,
            'robot_description': robot_description,
        }],
        remappings=remappings,
        output='screen',
    )

    # 直接用 -string 传 URDF，不依赖 /robot_description topic（避免被其他发布者覆盖）
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-string', robot_description,
            '-name', robot['name'],
            '-x', str(robot['x_pose']),
            '-y', str(robot['y_pose']),
            '-z', str(robot['z_pose']),
            '-Y', str(robot['yaw']),
        ],
        output='screen',
    )

    # Bridge Ignition sensor topics to ROS 2
    sensor_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/imu/data@sensor_msgs/msg/Imu[ignition.msgs.IMU',
            '/imu/mid360@sensor_msgs/msg/Imu[ignition.msgs.IMU',
            '/lidar/points@sensor_msgs/msg/PointCloud2[ignition.msgs.PointCloudPacked',
            # 底盘速度指令 ROS -> Ignition（MecanumDrive2 订阅 <model_name>/cmd_vel）
            f"/{robot['name']}/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist",
        ],
        output='screen',
    )

    # Activate controllers after spawn (give gz_ros2_control time to load)
    activate_controllers = TimerAction(
        period=5.0,
        actions=[
            ExecuteProcess(
                cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
                     'joint_state_broadcaster'],
                output='screen',
            ),
            ExecuteProcess(
                cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
                     'gimbal_position_controller'],
                output='screen',
            ),
        ],
    )

    controller_joint = Node(
        package='rm_9_2',
        executable='controller_joint',
        name='controller_joint',
        parameters=[
            os.path.join(pkg_share, 'config', 'controller_joint.yaml'),
            {'chassis_cmd_vel_topic': f"/{robot['name']}/cmd_vel"},
        ],
        output='screen',
    )

    return LaunchDescription([
        set_plugin_path,
        robot_state_publisher,
        spawn_robot,
        sensor_bridge,
        activate_controllers,
        controller_joint,
    ])
