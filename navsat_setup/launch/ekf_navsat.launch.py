# ============================================================
# Launch file to run two EKF filters and one NavSatTransform node
# using robot_localization package.
# ============================================================

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    # Correct ROS2 way to load the config
    config_path = os.path.join(
        get_package_share_directory('navsat_setup'),
        'config',
        'ekf_navsat_config.yaml'
    )

    # ------------------------------------------------------------
    # Static transform: base_link -> imu_link
    # ------------------------------------------------------------
    static_tf_imu = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_imu_broadcaster',
        arguments=[
            '--x', '0',
            '--y', '0',
            '--z', '0.4',
            '--qx', '0',
            '--qy', '0',
            '--qz', '0',
            '--qw', '1',
            '--frame-id', 'base_link',
            '--child-frame-id', 'imu_link'
        ],
        output='screen'
    )

    # ------------------------------------------------------------
    # Static transform: base_link -> gps_link
    # ------------------------------------------------------------
    static_tf_gps = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_gps_broadcaster',
        arguments=[
            '--x', '0',
            '--y', '0',
            '--z', '0',
            '--qx', '0',
            '--qy', '0',
            '--qz', '0',
            '--qw', '1',
            '--frame-id', 'base_link',
            '--child-frame-id', 'gps_link'
        ],
        output='screen'
    )

    # ------------------------------------------------------------
    # Local EKF (odom frame)
    # ------------------------------------------------------------
    ekf_local = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node_odom',
        output='screen',
        parameters=[config_path],
        remappings=[
            ('/odometry/filtered', '/odometry/filtered_local')
        ]
    )

    # ------------------------------------------------------------
    # Global EKF (map frame)
    # ------------------------------------------------------------
    ekf_global = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node_map',
        output='screen',
        parameters=[config_path],
        remappings=[
            ('/odometry/filtered', '/odometry/filtered_global')
        ]
    )

    # ------------------------------------------------------------
    # NavSat Transform Node
    # ------------------------------------------------------------
    navsat_transform = Node(
        package='robot_localization',
        executable='navsat_transform_node',
        name='navsat_transform_node',
        output='screen',
        parameters=[config_path],
        remappings=[
            ('imu/data', '/imu/data'),
            ('gps/fix', '/fix'),
            ('odometry/filtered', '/odometry/filtered_global'),
            ('odometry/gps', '/odometry/gps')
        ]
    )

    # ------------------------------------------------------------
    # Launch all nodes
    # ------------------------------------------------------------
    return LaunchDescription([
        ekf_local,
        ekf_global,
        navsat_transform,
        static_tf_imu,
        static_tf_gps
    ])

