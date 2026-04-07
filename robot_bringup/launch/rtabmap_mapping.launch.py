import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    bringup = get_package_share_directory('robot_bringup')
    desc    = get_package_share_directory('robot_description')
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

    realsense_node = Node(
        package='realsense2_camera',
        executable='realsense2_camera_node',
        name='camera',
        namespace='camera',
        parameters=[{
            'enable_color': True,
            'enable_depth': True,
            'enable_accel': False,
            'enable_gyro': False,
            'align_depth.enable': True,
            'pointcloud__neon_.enable': True,
            'depth_module.profile': '848x480x30',
            'rgb_camera.profile': '640x480x30',
            'base_frame_id': 'camera_link',
            'camera_name': 'camera',
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

    camera_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_base_tf',
        arguments=['--x', '0', '--y', '0', '--z', '0',
                   '--roll', '0', '--pitch', '0', '--yaw', '0',
                   '--frame-id', 'camera_link',
                   '--child-frame-id', 'camera_camera_link'],
        output='screen'
    )

    os.makedirs('/home/s23/rtabmap', exist_ok=True)

    rtabmap_node = Node(
        package='rtabmap_slam',
        executable='rtabmap',
        name='rtabmap',
        output='screen',
        parameters=[{
            # Input streams
            'subscribe_depth': True,
            'subscribe_rgb': True,
            'subscribe_scan': True,
            'subscribe_odom_info': False,
            # Frames
            'odom_frame_id': 'odom',
            'map_frame_id': 'map',
            'base_frame_id': 'base_link',
            # Sync
            'approx_sync': True,
            'queue_size': 10,
            'use_sim_time': False,
            # Database
            'database_path': '/home/s23/rtabmap/rtabmap.db',
            # Trust odometry — do NOT re-estimate motion from camera/ICP
            'Reg/Strategy': '0',            # trust odometry completely
            'Reg/Force3DoF': 'true',        # 2D robot
            'Odom/Strategy': '0',
            # Loop closure
            'RGBD/NeighborLinkRefining': 'false',  # don't correct odom
            'RGBD/ProximityBySpace': 'true',
            'RGBD/AngularUpdate': '0.05',
            'RGBD/LinearUpdate': '0.05',
            'RGBD/OptimizeFromGraphEnd': 'false',
            'RGBD/OptimizeMaxError': '0.0',  # accept all loop closures
            # 2D grid from LiDAR only
            'Grid/FromDepth': 'false',
            'Grid/CellSize': '0.05',
            'Grid/RangeMax': '12.0',
            'Mem/NotLinkedNodesKept': 'false',
        }],
        remappings=[
            ('rgb/image',       '/camera/camera/color/image_raw'),
            ('depth/image',     '/camera/camera/aligned_depth_to_color/image_raw'),
            ('rgb/camera_info', '/camera/camera/color/camera_info'),
            ('scan',            '/scan'),
            ('odom',            '/odometry/filtered'),
        ],
        arguments=['--delete_db_on_start'],
    )

    return LaunchDescription([
        description_launch,
        rplidar_node,
        realsense_node,
        zlac_node,
        imu_node,
        imu_filter,
        ekf_node,
        camera_tf,
        rtabmap_node,
    ])
