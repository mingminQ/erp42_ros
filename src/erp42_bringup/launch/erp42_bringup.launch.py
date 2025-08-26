# erp42_bringup.launch.py

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node

def generate_launch_description():
    
    # GUI option
    view_feedback_monitor = DeclareLaunchArgument('view_feedback_monitor', default_value = 'false', 
        description = 'Whether to launch the feedback monitor')

    # Serial port settings
    port_path = DeclareLaunchArgument('port_path', default_value = '/dev/ttyUSB0', description = 'Path to the serial port device')
    baud_rate = DeclareLaunchArgument('baud_rate', default_value = '115200'      , description = 'Serial port baud rate'         )

    # ERP42 parameters
    max_speed           = DeclareLaunchArgument('max_speed'          , default_value = '7.00'  , description = 'Maximum speed (m/s)'         )
    max_steering_deg    = DeclareLaunchArgument('max_steering_deg'   , default_value = '28.00' , description = 'Maximum steering angle (deg)')
    steering_offset_deg = DeclareLaunchArgument('steering_offset_deg', default_value = '0.00'  , description = 'Steering offset (deg)'       )

    # Feedback monitor
    erp42_feedback_monitor = Node(package = 'erp42_gui', executable = 'feedback_monitor', name = 'erp42_feedback_monitor', output = 'screen',
        condition = IfCondition(LaunchConfiguration('view_feedback_monitor'))
    )

    # ERP42 serial driver
    erp42_serial_bridge = Node(package = 'erp42_serial', executable = 'serial_bridge', name = 'erp42_serial_bridge', output = 'screen',
        parameters = [{
            'port_path'          : LaunchConfiguration('port_path'),
            'baud_rate'          : LaunchConfiguration('baud_rate'),
            'max_speed'          : LaunchConfiguration('max_speed'),
            'max_steering_deg'   : LaunchConfiguration('max_steering_deg'),
            'steering_offset_deg': LaunchConfiguration('steering_offset_deg'),
        }]
    )

    return LaunchDescription([
        view_feedback_monitor,
        port_path,
        baud_rate,
        max_speed,
        max_steering_deg,
        steering_offset_deg,
        erp42_feedback_monitor,
        erp42_serial_bridge
    ])