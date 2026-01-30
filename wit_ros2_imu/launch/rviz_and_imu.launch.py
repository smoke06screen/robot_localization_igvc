from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    imu_node = Node(
        package='wit_ros2_imu',
        executable='wit_ros2_imu',
        name='imu',
        parameters=[
            {'port': '/dev/imu'},
            {'baudrate': 9600}
        ],
        output='screen'
    )

    return LaunchDescription([
        imu_node
    ])
