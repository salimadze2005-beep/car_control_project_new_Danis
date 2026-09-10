from pathlib import Path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    config = str(Path(get_package_share_directory('car_control_sim')) / 'config/lightweight.json')
    return LaunchDescription([
        DeclareLaunchArgument('auto_start', default_value='true'),
        Node(package='car_control_sim', executable='controller', output='screen',
             parameters=[{'backend': 'lightweight', 'controller_config': config,
                          'use_sim_time': False,
                          'auto_start': ParameterValue(LaunchConfiguration('auto_start'), value_type=bool)}])])
