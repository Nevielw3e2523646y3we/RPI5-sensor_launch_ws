from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    params_file = '/root/sensor_launch_ws/src/launch_all/config/fusion/nav2_with_map.yaml'
    map_yaml = LaunchConfiguration('map', default='/root/sensor_launch_ws/src/launch_all/maps/site1.yaml')

    sensor_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource('/root/sensor_launch_ws/src/launch_all/launch/sensor_bringup.launch.py')
    )

    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('nav2_bringup'), 'launch', 'localization_launch.py')
        ),
        launch_arguments={
            'params_file': params_file,
            'use_sim_time': 'false',
            'map': map_yaml
        }.items()
    )

    return LaunchDescription([
        DeclareLaunchArgument('map', default_value='/root/sensor_launch_ws/src/launch_all/maps/site1.yaml'),
        sensor_bringup,
        nav2_launch
    ])
