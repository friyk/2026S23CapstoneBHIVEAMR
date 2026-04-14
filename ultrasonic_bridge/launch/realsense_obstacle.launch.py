from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """Converts the D435 depth pointcloud (already published by robot.launch.py)
    into a virtual 2D LaserScan, height-filtered to the body band that LiDAR
    misses and ultrasonic can't reach reliably."""

    pc_to_scan = Node(
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
            # Height band above ground (metres).
            # Camera is mounted ~30 cm above base_link (which is at wheel axle).
            # Wheel radius 0.0865 -> camera ~38 cm above ground.
            # Slice from just above ultrasonic plane to just below LiDAR.
            # TUNE these once you know your LiDAR mount height.
            'min_height': 0.10,
            'max_height': 0.39,
            'angle_min':  -0.80,   # ~±46° (D435 depth FOV is ~87° H)
            'angle_max':   0.80,
            'angle_increment': 0.008,  # ~0.46°
            'scan_time': 0.066,
            'range_min': 0.25,     # D435 min depth is ~0.2 m
            'range_max': 3.00,
            'use_inf': True,
            'inf_epsilon': 1.0,
        }]
    )

    return LaunchDescription([pc_to_scan])
