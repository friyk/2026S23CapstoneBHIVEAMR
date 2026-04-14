import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
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
            'scan_frequency': 5.0,
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
            'enable_accel': True,
            'enable_gyro': True,
            'unite_imu_method': 2,
            'align_depth.enable': True,
            'pointcloud__neon_.enable': True,
            'pointcloud__neon_.ordered.pc': False,
            'enable_sync': True,
            'depth_module.enable_auto_exposure': True,
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
        remappings=[('cmd_vel', 'cmd_vel_safe')],
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
        remappings=[
            ('/imu/data_raw', '/imu/data_raw'),
            ('/imu/data',     '/imu/data'),
        ],
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

    ultrasonic_node = Node(
        package='ultrasonic_bridge',
        executable='ultrasonic_node',
        name='ultrasonic_bridge',
        parameters=[{'port': '/dev/esp32', 'baud': 115200}],
        output='screen'
    )

    safety_stop_node = Node(
        package='ultrasonic_bridge',
        executable='safety_stop_node',
        name='safety_stop',
        parameters=[{
            'front_stop_distance': 0.25,
            'back_stop_distance':  0.18,
            'hysteresis':          0.05,
            'sensor_timeout':      0.5,
        }],
        output='screen'
    )

    pc_to_scan_node = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='depth_to_scan',
        remappings=[
            ('cloud_in', '/camera/camera/depth/color/points'),
            ('scan', '/scan_depth'),
        ],
        parameters=[{
            'target_frame': 'base_link',
            'transform_tolerance': 0.1,
            'min_height': 0.10,
            'max_height': 0.39,
            'angle_min':  -0.80,
            'angle_max':   0.80,
            'angle_increment': 0.008,
            'scan_time': 0.066,
            'range_min': 0.25,
            'range_max': 3.00,
            'use_inf': True,
            'inf_epsilon': 1.0,
        }]
    )

    slam_node = TimerAction(period=5.0, actions=[
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            parameters=[
                os.path.join(bringup, 'config', 'slam_toolbox_params.yaml'),
                {'use_sim_time': False}
            ],
            output='screen',
        )
    ])

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

    return LaunchDescription([
        description_launch,
        rplidar_node,
        realsense_node,
        zlac_node,
        imu_node,
        imu_filter,
        ekf_node,
        camera_tf,
        ultrasonic_node,
        safety_stop_node,
        pc_to_scan_node,
        slam_node,
    ])
