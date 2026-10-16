from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, RegisterEventHandler, EmitEvent
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch_ros.actions import Node


def generate_launch_description():
    share = Path(get_package_share_directory('ira1_sim'))
    simulator = ExecuteProcess(cmd=['gz', 'sim', '-r', str(share/'worlds/room.sdf')], output='screen')
    return LaunchDescription([
        simulator,
        Node(package='ros_gz_bridge', executable='parameter_bridge',
             parameters=[{'config_file': str(share/'config/bridge.yaml')}], output='screen'),
        RegisterEventHandler(OnProcessExit(target_action=simulator,
            on_exit=[EmitEvent(event=Shutdown(reason='Gazebo closed; stop its bridge'))])),
    ])
