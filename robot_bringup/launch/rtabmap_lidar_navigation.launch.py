import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    bringup = get_package_share_directory('robot_bringup')
    desc    = get_package_share_directory('robot_description')
    nav2    = get_package_share_directory('nav2_bringup')
    ekf_cfg = os.path.join(bringup, 'config', 'ekf.yaml')

    description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(desc, 'launch', 'description.launch.py')))

    rplidar_node = Node(
        package='rplidar_ros',
        executable='rplidar_node',
        name='rplidar_node',
        parameters=[{
            'serial_port': '/dev/rplidar',
            'serial_baudrate': 115200,
            'frame_id': 'laser_frame',
            'inverted': False,
            'angle_compensate': True,
            'scan_mode': 'Sensitivity',
        }],
        output='screen'
    )

    zlac_node = Node(
        package='zlac_driver',
        executable='motor_node',
        name='zlac_driver_node',
        parameters=[{
            'port': '/dev/zlac',
            'wheel_base': 0.275,
            'wheel_radius': 0.0865,
        }],
        output='screen'
    )

    imu_node = Node(
        package='grove_sensors',
        executable='imu_node',
        output='screen'
    )

    imu_filter = Node(
        package='imu_filter_madgwick',
        executable='imu_filter_madgwick_node',
        name='imu_filter',
        parameters=[{
            'use_mag': False,
            'gain': 0.1,
            'publish_tf': False,
            'world_frame': 'enu',
        }],
        output='screen'
    )

    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        parameters=[ekf_cfg],
        remappings=[('odometry/filtered', 'odometry/filtered')],
        output='screen'
    )

    rtabmap_node = Node(
        package='rtabmap_slam',
        executable='rtabmap',
        name='rtabmap',
        output='screen',
        parameters=[{
            'subscribe_depth': False,
            'subscribe_rgb': False,
            'subscribe_scan': True,
            'subscribe_odom_info': False,
            'odom_frame_id': 'odom',
            'map_frame_id': 'map',
            'base_frame_id': 'base_link',
            'approx_sync': True,
            'queue_size': 10,
            'use_sim_time': False,
            'database_path': '/home/s23/rtabmap/rtabmap.db',
            'Mem/IncrementalMemory': 'false',
            'Mem/InitWMWithAllNodes': 'true',
            'Reg/Strategy': '1',
            'Reg/Force3DoF': 'true',
            'RGBD/NeighborLinkRefining': 'false',
            'RGBD/AngularUpdate': '0.05',
            'RGBD/LinearUpdate': '0.05',
            'Grid/FromDepth': 'false',
            'Grid/Sensor': '0',
            'Grid/CellSize': '0.05',
            'Grid/RangeMax': '12.0',
        }],
        remappings=[
            ('scan', '/scan'),
            ('odom', '/odometry/filtered'),
        ],
    )

    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2, 'launch', 'navigation_launch.py')),
        launch_arguments={
            'params_file': os.path.join(bringup, 'config', 'nav2_params.yaml'),
            'use_sim_time': 'False',
        }.items()
    )

    return LaunchDescription([
        description_launch,
        rplidar_node,
        zlac_node,
        imu_node,
        imu_filter,
        ekf_node,
        rtabmap_node,
        nav2_launch,
    ])
