"""Gazebo Classic Go2-W with CHAMP hybrid drive and PX4 ECL."""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    return LaunchDescription([IncludeLaunchDescription(PythonLaunchDescriptionSource(
        os.path.join(get_package_share_directory('px4_gps_imu'), 'launch', 'go2_px4_sim.launch.py')))])
