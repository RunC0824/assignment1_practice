from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

def generate_launch_description():
    share = Path(get_package_share_directory('ira1_sim'))
    return LaunchDescription([
        ExecuteProcess(cmd=['gz','sim','-r',str(share/'worlds/room.sdf')], output='screen'),
        Node(package='ros_gz_bridge', executable='parameter_bridge',
             parameters=[{'config_file': str(share/'config/bridge.yaml')}], output='screen'),
    ])
