from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    gps_node = Node(
        package='nmea_navsat_driver',
        executable='nmea_serial_driver',
        name='gps_driver',
        parameters=[{
            'port': '/dev/gps',
            'baud': 9600,
            'frame_id': 'gps_link'
        }],
        output='screen'
    )

    return LaunchDescription([gps_node])

