from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    # happy_stt の launch ファイルパスを取得
    stt_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                FindPackageShare("happy_stt").find("happy_stt"),
                "launch",
                "launch_stt.launch.py"
            )
        )
    )

    return LaunchDescription([
        Node(
            package='happy_tts',
            executable='launch_tts_server',
            name='tts_server'
        ),
        Node(
            package='happy_tts',
            executable='tts',
            name='tts_client'
        ),
        stt_launch,
        Node(
            package='happy_voice',
            executable='yes_no',
            name='yes_no_node'
        )
    ])
