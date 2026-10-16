"""Run localisation and evaluation against a fresh, stationary simulation."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction, RegisterEventHandler, EmitEvent
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = Path(get_package_share_directory('ira1_sim'))
    output = LaunchConfiguration('output_dir')
    config = LaunchConfiguration('config')
    common = [{'output_dir': output}]
    estimator = Node(package='ira1_sim', executable='estimate', parameters=[config, *common], output='screen')
    evaluator = Node(package='ira1_sim', executable='evaluate', parameters=common, output='screen')
    return LaunchDescription([
        DeclareLaunchArgument('output_dir', description='New directory for this experiment'),
        DeclareLaunchArgument('config', default_value=str(share/'config/odometry.yaml')),
        Node(package='ira1_sim', executable='record', parameters=common, output='screen'),
        evaluator,
        estimator,
        RegisterEventHandler(OnProcessExit(target_action=estimator,
            on_exit=[EmitEvent(event=Shutdown(reason='Estimator stopped; stop the experiment'))])),
        RegisterEventHandler(OnProcessExit(target_action=evaluator,
            on_exit=[EmitEvent(event=Shutdown(reason='Evaluator stopped; stop the experiment'))])),
        # Allow subscriptions to connect; drive.py also waits 2 simulation seconds.
        TimerAction(period=3.0, actions=[Node(package='ira1_sim', executable='drive', output='screen')]),
    ])
