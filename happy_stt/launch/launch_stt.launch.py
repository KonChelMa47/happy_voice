from launch import LaunchDescription
from launch.actions import ExecuteProcess
import os

def generate_launch_description():
    return LaunchDescription([
        ExecuteProcess(
            cmd=[
                'bash',
                os.path.expanduser('~/main_ws/src//happy_voice/happy_stt/start_stt.sh')
            ],
            output='screen'
        )
    ])
