from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='create_driver',
            namespace='',
            executable='create_driver',
            name='create_driver',
            output='screen',
            respawn=True,
            respawn_delay=4,
            parameters=[{
                'robot_model': 'CREATE_1',
                'dev': '/dev/ttyUSB0',
                'baud': 57600,
                'base_frame': 'base_footprint',
                'odom_frame': 'odom',
                'latch_cmd_duration': 0.5,
                'loop_hz': 5.0,
                'publish_tf': True,
                'gyro_offset': 0.0,
                'gyro_scale': 1.3

            }],
            remappings=[('cmd_vel', 'diff_cont/cmd_vel'),('odom','diff_cont/odom')]
        ),
        Node(
            package='xv_11_driver',
            namespace='',
            executable='xv_11_driver',
            name='xv_11_driver',
            output='screen',
            respawn=True,
            respawn_delay=4,
            parameters=[{
                'port': '/dev/ttyACM0',
                'baud_rate': 115200,
                'frame_id': 'base_scan',
                'firmware_version': 2,
            }]
        ),
        Node(
            package='bno055',
            namespace='',
            executable='bno055',
            name='bno055',
            output='screen',
            respawn=True,
            respawn_delay=4,
            parameters=[{
                'ros_topic_prefix': '',
                'connection_type': 'i2c',
                'i2c_bus': 1,
                'i2c_addr': 0x28,
                'data_query_frequency': 20,
                'calib_status_frequency': 0.1,
                'frame_id': 'imu_link',
                'operation_mode': 0x0C,
                'placement_axis_remap': 'P2',
                'acc_factor': 100.0,
                'mag_factor': 16000000.0,
                'gyr_factor': 900.0,
                'set_offsets': False, # set to true to use offsets below
                'offset_acc': [0xFFEC, 0x00A5, 0xFFE8],
                'offset_mag': [0xFFB4, 0xFE9E, 0x027D],
                'offset_gyr': [0x0002, 0xFFFF, 0xFFFF]
            }]
        )
    ])
