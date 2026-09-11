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
    fsds_throttle_scale = float(LaunchConfiguration('fsds_throttle_scale').perform(context))
    fsds_speed_soft_zone_mps = float(
        LaunchConfiguration('fsds_speed_soft_zone_mps').perform(context))
    fsds_speed_brake_margin_mps = float(
        LaunchConfiguration('fsds_speed_brake_margin_mps').perform(context))
    if not fsds_max_speed_mps > 0:
        raise ValueError('fsds_max_speed_mps must be positive')
    if not 0 <= fsds_speed_brake <= 1:
        raise ValueError('fsds_speed_brake must be between 0 and 1')
    if not 0 <= fsds_throttle_scale <= 1:
        raise ValueError('fsds_throttle_scale must be between 0 and 1')
    if fsds_speed_soft_zone_mps < 0:
        raise ValueError('fsds_speed_soft_zone_mps cannot be negative')
    if not 0 <= fsds_speed_brake_margin_mps < fsds_max_speed_mps:
        raise ValueError('fsds_speed_brake_margin_mps must be in [0, max speed)')

    sensor_values = {
        'data_timeout': float(LaunchConfiguration('data_timeout').perform(context)),
        'sensor_rate_hz': float(LaunchConfiguration('sensor_rate_hz').perform(context)),
        'sensor_latency_s': float(LaunchConfiguration('sensor_latency_s').perform(context)),
        'sensor_dropout_probability': float(LaunchConfiguration('sensor_dropout_probability').perform(context)),
        'sensor_lateral_std_m': float(LaunchConfiguration('sensor_lateral_std_m').perform(context)),
        'sensor_depth_std_m': float(LaunchConfiguration('sensor_depth_std_m').perform(context)),
        'sensor_depth_relative_std': float(LaunchConfiguration('sensor_depth_relative_std').perform(context)),
        'sensor_camera_offset_x_m': float(LaunchConfiguration('sensor_camera_offset_x_m').perform(context)),
        'sensor_camera_offset_z_m': float(LaunchConfiguration('sensor_camera_offset_z_m').perform(context)),
        'sensor_max_per_color': int(LaunchConfiguration('sensor_max_per_color').perform(context)),
        'sensor_seed': int(LaunchConfiguration('sensor_seed').perform(context)),
        'sensor_fov_deg': float(LaunchConfiguration('sensor_fov_deg').perform(context)),
    }


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
                          'fsds_speed_brake': fsds_speed_brake,
                          'fsds_throttle_scale': fsds_throttle_scale,
                          'fsds_speed_soft_zone_mps': fsds_speed_soft_zone_mps,
                          'fsds_speed_brake_margin_mps': fsds_speed_brake_margin_mps,
                          **sensor_values}])]


def generate_launch_description():
    default_config = str(Path(get_package_share_directory('car_control_sim')) / 'config/fsds.json')
    return LaunchDescription([
        DeclareLaunchArgument('host', default_value='localhost'),
        DeclareLaunchArgument('controller_config', default_value=default_config),
        DeclareLaunchArgument('bridge_telemetry_period', default_value='0.05'),
        DeclareLaunchArgument('fsds_max_speed_mps', default_value='2.0'),
        DeclareLaunchArgument('fsds_speed_brake', default_value='0.25'),
        DeclareLaunchArgument('fsds_throttle_scale', default_value='0.20'),
        DeclareLaunchArgument('fsds_speed_soft_zone_mps', default_value='0.0'),
        DeclareLaunchArgument('fsds_speed_brake_margin_mps', default_value='0.0'),
        DeclareLaunchArgument('data_timeout', default_value='0.4'),
        DeclareLaunchArgument('sensor_rate_hz', default_value='0.0'),
        DeclareLaunchArgument('sensor_latency_s', default_value='0.0'),
        DeclareLaunchArgument('sensor_dropout_probability', default_value='0.0'),
        DeclareLaunchArgument('sensor_lateral_std_m', default_value='0.0'),
        DeclareLaunchArgument('sensor_depth_std_m', default_value='0.0'),
        DeclareLaunchArgument('sensor_depth_relative_std', default_value='0.0'),
        DeclareLaunchArgument('sensor_camera_offset_x_m', default_value='0.0'),
        DeclareLaunchArgument('sensor_camera_offset_z_m', default_value='0.0'),
        DeclareLaunchArgument('sensor_max_per_color', default_value='0'),
        DeclareLaunchArgument('sensor_seed', default_value='0'),
        DeclareLaunchArgument('sensor_fov_deg', default_value='90.0'),
        OpaqueFunction(function=setup)])
