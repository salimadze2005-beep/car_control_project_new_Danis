from pathlib import Path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from car_control_core.fsds import validate_host


def setup(context):
    host = validate_host(LaunchConfiguration('host').perform(context))
    config = LaunchConfiguration('controller_config').perform(context)
    # Official executable, native host_ip RPC path. No replacement bridge/camera nodes.
    return [
        Node(package='fsds_ros2_bridge', executable='fsds_ros2_bridge', namespace='fsds',
             name='ros_bridge', output='screen', parameters=[{'host_ip': host, 'timeout': 2.0,
                 'competition_mode': False, 'manual_mode': False, 'mission_name': 'trackdrive',
                 'track_name': 'A'}]),
        Node(package='car_control_sim', executable='controller', output='screen',
             parameters=[{'backend': 'fsds', 'host': host, 'controller_config': config,
                          'use_sim_time': True, 'auto_start': False}])]


def generate_launch_description():
    default_config = str(Path(get_package_share_directory('car_control_sim')) / 'config/fsds.json')
    return LaunchDescription([
        DeclareLaunchArgument('host', default_value='localhost'),
        DeclareLaunchArgument('controller_config', default_value=default_config),
        OpaqueFunction(function=setup)])
