from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import os

def generate_launch_description():
    pkg_path = '/root/sensor_launch_ws/src/launch_all'
    cfg_fu   = os.path.join(pkg_path, 'config', 'fusion')
    slam_params   = os.path.join(cfg_fu, 'slam_mapper.yaml')
    nav2_params   = os.path.join(cfg_fu, 'nav2_with_map.yaml')

    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            '/opt/ros/', os.environ.get('ROS_DISTRO','jazzy'), '/share/slam_toolbox/launch/online_async_launch.py'
        ]),
        launch_arguments={'slam_params_file': slam_params}.items()
    )

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            '/opt/ros/', os.environ.get('ROS_DISTRO','jazzy'), '/share/nav2_bringup/launch/bringup_launch.py'
        ]),
        launch_arguments={
            'slam': 'True',
            'use_sim_time': 'False',
            'params_file': nav2_params
        }.items()
    )

    guard = ExecuteProcess(
        cmd=['python3', os.path.join(pkg_path, 'scripts', 'cmd_vel_guard.py')],
        output='screen'
    )

    return LaunchDescription([slam, nav2, guard])
