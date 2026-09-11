from pathlib import Path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    package = Path(get_package_share_directory('car_control_sim'))
    fsds_launch = str(package / 'launch/fsds.launch.py')
    jetson_config = str(package / 'config/fsds_jetson.json')
    arguments = {
        'host': LaunchConfiguration('host'),
        'controller_config': jetson_config,
        'bridge_telemetry_period': '0.05',
        'fsds_max_speed_mps': LaunchConfiguration('fsds_max_speed_mps'),
        'fsds_speed_brake': LaunchConfiguration('fsds_speed_brake'),
        # The hardware core asks for full normalized speed; each actuator maps it.
        'fsds_throttle_scale': LaunchConfiguration('fsds_throttle_scale'),
        'fsds_speed_soft_zone_mps': LaunchConfiguration('fsds_speed_soft_zone_mps'),
        'fsds_speed_brake_margin_mps': LaunchConfiguration('fsds_speed_brake_margin_mps'),
        'data_timeout': '0.4',
        # Values below mirror the active Main/server.py perception path.
        'sensor_rate_hz': '15.0',
        'sensor_latency_s': '0.067',
        'sensor_dropout_probability': LaunchConfiguration('sensor_dropout_probability'),
        'sensor_lateral_std_m': LaunchConfiguration('sensor_lateral_std_m'),
        'sensor_depth_std_m': LaunchConfiguration('sensor_depth_std_m'),
        'sensor_depth_relative_std': LaunchConfiguration('sensor_depth_relative_std'),
        # Main loads camera_offset_x=-0.06 but does not currently apply it.
        'sensor_camera_offset_x_m': LaunchConfiguration('sensor_camera_offset_x_m'),
        'sensor_camera_offset_z_m': '0.10',
        'sensor_max_per_color': '6',
        'sensor_seed': LaunchConfiguration('sensor_seed'),
        'sensor_fov_deg': '90.0',
    }
    return LaunchDescription([
        DeclareLaunchArgument('host', default_value='localhost'),
        DeclareLaunchArgument('fsds_max_speed_mps', default_value='1.0'),
        DeclareLaunchArgument('fsds_speed_brake', default_value='1.0'),
        DeclareLaunchArgument('fsds_throttle_scale', default_value='0.20'),
        DeclareLaunchArgument('fsds_speed_soft_zone_mps', default_value='1.0'),
        DeclareLaunchArgument('fsds_speed_brake_margin_mps', default_value='0.80'),
        DeclareLaunchArgument('sensor_dropout_probability', default_value='0.0'),
        DeclareLaunchArgument('sensor_lateral_std_m', default_value='0.0'),
        DeclareLaunchArgument('sensor_depth_std_m', default_value='0.0'),
        DeclareLaunchArgument('sensor_depth_relative_std', default_value='0.0'),
        DeclareLaunchArgument('sensor_camera_offset_x_m', default_value='0.0'),
        DeclareLaunchArgument('sensor_seed', default_value='2005'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(fsds_launch),
                                 launch_arguments=arguments.items()),
    ])
