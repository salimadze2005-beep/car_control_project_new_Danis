from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    package = Path(get_package_share_directory('car_control_sim'))
    arguments = {
        'host': LaunchConfiguration('host'),
        'controller_config': str(package / 'config/fsds_drive.json'),
        'bridge_telemetry_period': '0.05',
        # Start with keyboard ownership so the operator can position the car.
        'initial_manual_mode': LaunchConfiguration('initial_manual_mode'),
        'enable_front_camera': 'true',
        # This laptop delivers about 2 FPS at 640x480; requesting 15 FPS makes
        # the shared FSDS RPC server less responsive without adding frames.
        'camera_framerate': '5.0',
        'bridge_rpc_timeout': '10.0',
        'camera_rpc_timeout': '15.0',
        'fsds_max_speed_mps': LaunchConfiguration('fsds_max_speed_mps'),
        'fsds_speed_brake': '1.0',
        'fsds_throttle_scale': LaunchConfiguration('fsds_throttle_scale'),
        'fsds_speed_soft_zone_mps': LaunchConfiguration('fsds_speed_soft_zone_mps'),
        'fsds_speed_brake_margin_mps': '0.0',
        'fsds_overspeed_brake_zone_mps': LaunchConfiguration(
            'fsds_overspeed_brake_zone_mps'),
        'fsds_speed_kp': '0.05',
        'fsds_speed_ki': '0.04',
        # Calibrated against FSDS v2.2.0 at 2.5 m/s.  A small integral cap
        # avoids the stop/go cycle caused by the simulator's strong drivetrain.
        'fsds_speed_integral_limit': '0.5',
        'fsds_speed_brake_gain': '1.0',
        'fsds_speed_breakaway_throttle': '0.20',
        # Closed-loop speed control is simulation-only; Jetson profile keeps
        # the previous legacy limiter unless explicitly changed.
        'fsds_longitudinal_mode': 'pi',
        'data_timeout': '0.4',
        'sensor_rate_hz': '15.0',
        'sensor_latency_s': '0.067',
        'sensor_dropout_probability': '0.0',
        'sensor_lateral_std_m': '0.0',
        'sensor_depth_std_m': '0.0',
        'sensor_depth_relative_std': '0.0',
        'sensor_camera_offset_x_m': '0.0',
        'sensor_camera_offset_z_m': '0.10',
        'sensor_max_per_color': '6',
        'sensor_seed': '2005',
        # Ground-truth reacquisition aid for manually repositioned FSDS cars.
        'sensor_fov_deg': '140.0',
    }
    return LaunchDescription([
        DeclareLaunchArgument('host', default_value='localhost'),
        DeclareLaunchArgument('initial_manual_mode', default_value='true'),
        DeclareLaunchArgument('fsds_max_speed_mps', default_value='2.5'),
        DeclareLaunchArgument('fsds_throttle_scale', default_value='0.20'),
        DeclareLaunchArgument('fsds_speed_soft_zone_mps', default_value='1.5'),
        DeclareLaunchArgument('fsds_overspeed_brake_zone_mps', default_value='1.0'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(package / 'launch/fsds.launch.py')),
            launch_arguments=arguments.items()),
    ])
