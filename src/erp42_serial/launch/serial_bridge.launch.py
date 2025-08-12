# serial_bridge.launch.py

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    
    # Serial port settings
    port_path = DeclareLaunchArgument('port_path', default_value = '/dev/ttyUSB0', description='Path to the serial port device')
    baud_rate = DeclareLaunchArgument('baud_rate', default_value = '115200'      , description = 'Serial port baud rate'       )

    # ERP42 racing parameters
    max_speed           = DeclareLaunchArgument('max_speed'          , default_value = '7.00'  , description = 'Maximum speed (m/s)'         )
    max_steering_deg    = DeclareLaunchArgument('max_steering_deg'   , default_value = '28.00' , description = 'Maximum steering angle (deg)')
    steering_offset_deg = DeclareLaunchArgument('steering_offset_deg', default_value = '0.00'  , description = 'Steering offset (deg)'       )

    serial_node = Node(package = 'erp42_serial', executable = 'serial_bridge', name = 'erp42_serial_bridge', output='screen',
        parameters = [{
            'port_path'          : LaunchConfiguration('port_path'),
            'baud_rate'          : LaunchConfiguration('baud_rate'),
            'max_speed'          : LaunchConfiguration('max_speed'),
            'max_steering_deg'   : LaunchConfiguration('max_steering_deg'),
            'steering_offset_deg': LaunchConfiguration('steering_offset_deg'),
        }]
    )

    return LaunchDescription([
        port_path,
        baud_rate,
        max_speed,
        max_steering_deg,
        steering_offset_deg,
        serial_node
    ])