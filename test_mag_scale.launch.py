from launch import LaunchDescription
from launch.actions import ExecuteProcess

def generate_launch_description():
    return LaunchDescription([
        ExecuteProcess(
            cmd=['bash','-lc',
                 'python3 -u /root/sensor_launch_ws/src/launch_all/config/fusion/mag_scale.py --scale 0.0052'],
            output='screen',
            additional_env={'PYTHONUNBUFFERED':'1'}
        )
    ])
