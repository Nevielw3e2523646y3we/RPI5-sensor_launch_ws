from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    sensor_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource('/root/sensor_launch_ws/src/launch_all/launch/sensor_bringup.launch.py')
    )
    slam = Node(
        package='slam_toolbox',
	executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=['/root/sensor_launch_ws/src/launch_all/config/fusion/slam_mapper.yaml'],
   	arguments=['--ros-args', '--log-level', 'slam_toolbox:=info']
    )
    return LaunchDescription([sensor_bringup, slam])
