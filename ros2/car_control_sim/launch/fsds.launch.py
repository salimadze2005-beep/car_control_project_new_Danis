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
    manual_text = LaunchConfiguration('initial_manual_mode').perform(context).strip().lower()
    if manual_text not in ('true', 'false'):
        raise ValueError('initial_manual_mode must be true or false')
    initial_manual_mode = manual_text == 'true'
    camera_text = LaunchConfiguration('enable_front_camera').perform(context).strip().lower()
    if camera_text not in ('true', 'false'):
        raise ValueError('enable_front_camera must be true or false')
    enable_front_camera = camera_text == 'true'
    camera_framerate = float(LaunchConfiguration('camera_framerate').perform(context))
    bridge_rpc_timeout = float(
        LaunchConfiguration('bridge_rpc_timeout').perform(context))
    camera_rpc_timeout = float(
        LaunchConfiguration('camera_rpc_timeout').perform(context))
    if not 1 <= camera_framerate <= 60:
        raise ValueError('camera_framerate must be within 1..60')
    if not 2 <= bridge_rpc_timeout <= 60 or not 2 <= camera_rpc_timeout <= 60:
        raise ValueError('RPC timeouts must be within 2..60 seconds')
    if not 0.01 <= bridge_telemetry_period <= 10.0:
        raise ValueError('bridge_telemetry_period must be between 0.01 and 10 seconds')

    fsds_max_speed_mps = float(LaunchConfiguration('fsds_max_speed_mps').perform(context))
    fsds_speed_brake = float(LaunchConfiguration('fsds_speed_brake').perform(context))
    fsds_throttle_scale = float(LaunchConfiguration('fsds_throttle_scale').perform(context))
    fsds_speed_soft_zone_mps = float(
        LaunchConfiguration('fsds_speed_soft_zone_mps').perform(context))
    fsds_speed_brake_margin_mps = float(
        LaunchConfiguration('fsds_speed_brake_margin_mps').perform(context))
    fsds_overspeed_brake_zone_mps = float(
        LaunchConfiguration('fsds_overspeed_brake_zone_mps').perform(context))
    fsds_speed_kp = float(LaunchConfiguration('fsds_speed_kp').perform(context))
    fsds_speed_ki = float(LaunchConfiguration('fsds_speed_ki').perform(context))
    fsds_speed_integral_limit = float(
        LaunchConfiguration('fsds_speed_integral_limit').perform(context))
    fsds_speed_brake_gain = float(
        LaunchConfiguration('fsds_speed_brake_gain').perform(context))
    fsds_speed_breakaway_throttle = float(
        LaunchConfiguration('fsds_speed_breakaway_throttle').perform(context))
    fsds_longitudinal_mode = LaunchConfiguration(
        'fsds_longitudinal_mode').perform(context).strip().lower()
    if fsds_longitudinal_mode not in ('legacy', 'pi'):
        raise ValueError('fsds_longitudinal_mode must be legacy or pi')
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
    if fsds_overspeed_brake_zone_mps < 0:
        raise ValueError('fsds_overspeed_brake_zone_mps cannot be negative')

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
    # Official executables and native host_ip RPC path.
    nodes = [
        Node(package='fsds_ros2_bridge', executable='fsds_ros2_bridge', namespace='fsds',
             name='ros_bridge', output='screen', respawn=True, respawn_delay=2.0,
             parameters=[{'host_ip': host, 'timeout': bridge_rpc_timeout,
                 'competition_mode': False, 'manual_mode': initial_manual_mode,
                 'mission_name': 'trackdrive',
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
                          'fsds_overspeed_brake_zone_mps': fsds_overspeed_brake_zone_mps,
                          'fsds_speed_kp': fsds_speed_kp,
                          'fsds_speed_ki': fsds_speed_ki,
                          'fsds_speed_integral_limit': fsds_speed_integral_limit,
                          'fsds_speed_brake_gain': fsds_speed_brake_gain,
                          'fsds_speed_breakaway_throttle': fsds_speed_breakaway_throttle,
                          'fsds_longitudinal_mode': fsds_longitudinal_mode,
                          **sensor_values}])]
    if enable_front_camera:
        nodes.append(Node(
            package='fsds_ros2_bridge', executable='fsds_ros2_bridge_camera',
            namespace='fsds/camera', name='front', output='screen',
            respawn=True, respawn_delay=3.0,
            parameters=[{'camera_name': 'front', 'depthcamera': False,
                         'framerate': camera_framerate, 'host_ip': host,
                         'timeout': camera_rpc_timeout}]))
    return nodes


def generate_launch_description():
    default_config = str(Path(get_package_share_directory('car_control_sim')) / 'config/fsds.json')
    return LaunchDescription([
        DeclareLaunchArgument('host', default_value='localhost'),
        DeclareLaunchArgument('controller_config', default_value=default_config),
        DeclareLaunchArgument('bridge_telemetry_period', default_value='0.05'),
        DeclareLaunchArgument('initial_manual_mode', default_value='false'),
        DeclareLaunchArgument('enable_front_camera', default_value='false'),
        DeclareLaunchArgument('camera_framerate', default_value='15.0'),
        DeclareLaunchArgument('bridge_rpc_timeout', default_value='10.0'),
        DeclareLaunchArgument('camera_rpc_timeout', default_value='15.0'),
        DeclareLaunchArgument('fsds_max_speed_mps', default_value='2.0'),
        DeclareLaunchArgument('fsds_speed_brake', default_value='0.25'),
        DeclareLaunchArgument('fsds_throttle_scale', default_value='0.20'),
        DeclareLaunchArgument('fsds_speed_soft_zone_mps', default_value='0.0'),
        DeclareLaunchArgument('fsds_speed_brake_margin_mps', default_value='0.0'),
        DeclareLaunchArgument('fsds_overspeed_brake_zone_mps', default_value='0.0'),
        DeclareLaunchArgument('fsds_speed_kp', default_value='0.08'),
        DeclareLaunchArgument('fsds_speed_ki', default_value='0.04'),
        DeclareLaunchArgument('fsds_speed_integral_limit', default_value='3.0'),
        DeclareLaunchArgument('fsds_speed_brake_gain', default_value='1.0'),
        DeclareLaunchArgument('fsds_speed_breakaway_throttle', default_value='0.20'),
        DeclareLaunchArgument('fsds_longitudinal_mode', default_value='legacy'),
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
