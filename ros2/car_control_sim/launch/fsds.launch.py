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
    bridge_telemetry_period = float(
        LaunchConfiguration('bridge_telemetry_period').perform(context)
    )
    if not 0.01 <= bridge_telemetry_period <= 10.0:
        raise ValueError('bridge_telemetry_period must be between 0.01 and 10 seconds')

    fsds_max_speed_mps = float(LaunchConfiguration('fsds_max_speed_mps').perform(context))
    fsds_speed_brake = float(LaunchConfiguration('fsds_speed_brake').perform(context))
    if not fsds_max_speed_mps > 0:
        raise ValueError('fsds_max_speed_mps must be positive')
    if not 0 <= fsds_speed_brake <= 1:
        raise ValueError('fsds_speed_brake must be between 0 and 1')


    # The upstream bridge has a single-threaded executor. Its 250 Hz default
    # telemetry polling can starve control callbacks over the WSL<->Windows RPC
    # path, so keep telemetry at 20 Hz by default.
    # Official executable, native host_ip RPC path. No replacement bridge/camera nodes.
    return [
        Node(package='fsds_ros2_bridge', executable='fsds_ros2_bridge', namespace='fsds',
             name='ros_bridge', output='screen', parameters=[{'host_ip': host, 'timeout': 2.0,
                 'competition_mode': False, 'manual_mode': False, 'mission_name': 'trackdrive',
                 'track_name': 'A',
                 'update_odom_every_n_sec': bridge_telemetry_period,
                 'update_gss_every_n_sec': bridge_telemetry_period,
                 'update_wheel_states_every_n_sec': bridge_telemetry_period}]),
        Node(package='car_control_sim', executable='controller', output='screen',
             parameters=[{'backend': 'fsds', 'host': host, 'controller_config': config,
                          'use_sim_time': True, 'auto_start': False,
                          'tick_period': bridge_telemetry_period,
                          'fsds_max_speed_mps': fsds_max_speed_mps,
                          'fsds_speed_brake': fsds_speed_brake}])]


def generate_launch_description():
    default_config = str(Path(get_package_share_directory('car_control_sim')) / 'config/fsds.json')
    return LaunchDescription([
        DeclareLaunchArgument('host', default_value='localhost'),
        DeclareLaunchArgument('controller_config', default_value=default_config),
        DeclareLaunchArgument('bridge_telemetry_period', default_value='0.05'),
        DeclareLaunchArgument('fsds_max_speed_mps', default_value='2.0'),
        DeclareLaunchArgument('fsds_speed_brake', default_value='0.25'),
        OpaqueFunction(function=setup)])
