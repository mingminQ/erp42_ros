# serial_bridge.launch.py

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    
    # ERP42 serial bridge parameter file
    serial_bridge_parameter = DeclareLaunchArgument('serial_bridge_parameter', 
        default_value=PathJoinSubstitution([
            FindPackageShare('erp42_serial'), 'config', 'serial_bridge.param.yaml'
        ])
    )

    # ERP42 serial bridge
    erp42_serial_bridge = Node(
        package    = 'erp42_serial', 
        executable = 'serial_bridge', 
        name       = 'serial_bridge', 
        output     = 'screen',
        parameters = [{LaunchConfiguration('serial_bridge_parameter')}]
    )

    return LaunchDescription([
        serial_bridge_parameter,
        erp42_serial_bridge
    ])