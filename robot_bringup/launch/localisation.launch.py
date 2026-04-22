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
            'pointcloud.enable': False,
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

    amcl_node = TimerAction(period=5.0, actions=[
        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            parameters=[{
                'use_sim_time': False,
                'base_frame_id': 'base_link',
                'global_frame_id': 'map',
                'odom_frame_id': 'odom',
                'scan_topic': '/scan',
                'min_particles': 2000,
                'max_particles': 8000,
                'update_min_d': 0.1,
                'update_min_a': 0.1,
                'resample_interval': 1,
                'transform_tolerance': 1.0,
                'recovery_alpha_slow': 0.0,
                'recovery_alpha_fast': 0.0,
                'laser_model_type': 'likelihood_field',
                'laser_max_range': 12.0,
                'laser_min_range': 0.15,
                'laser_max_beams': 60,
                'sigma_hit': 0.2,
                'z_hit': 0.5,
                'z_rand': 0.5,
                'alpha1': 0.1,
                'alpha2': 0.1,
                'alpha3': 0.05,
                'alpha4': 0.05,
                'alpha5': 0.1,
                'set_initial_pose': False,
                'always_reset_initial_pose': False,
            }],
            output='screen'
        )
    ])

    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        parameters=[{
            'use_sim_time': False,
            'yaml_filename': '/home/s23/ros2_ws/maps/new_graninja_map.yaml',
        }],
        output='screen'
    )

    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        parameters=[{
            'use_sim_time': False,
            'autostart': True,
            'node_names': ['map_server', 'amcl'],
        }],
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
        map_server_node,
        amcl_node,
        lifecycle_manager,
    ])
