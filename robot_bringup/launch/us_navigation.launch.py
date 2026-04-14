import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    bringup = get_package_share_directory('robot_bringup')
    nav2    = get_package_share_directory('nav2_bringup')

    localisation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup, 'launch', 'localisation.launch.py')))

    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2, 'launch', 'navigation_launch.py')),
        launch_arguments={
            'params_file': os.path.join(bringup, 'config', 'nav2_params.yaml'),
            'use_sim_time': 'False',
        }.items()
    )

    return LaunchDescription([
        localisation_launch,
        nav2_launch,
    ])
