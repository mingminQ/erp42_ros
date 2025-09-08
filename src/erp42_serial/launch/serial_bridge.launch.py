# serial_bridge.launch.py

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    
    # ERP42 serial driver
    parameter_file = PathJoinSubstitution([
        FindPackageShare('erp42_serial'), 
        'config', 
        'serial_bridge.param.yaml'
    ])
    erp42_serial_bridge = Node(
        package = 'erp42_serial', 
        executable = 'serial_bridge', 
        name = 'erp42_serial_bridge', 
        output = 'screen',
        parameters = [parameter_file]
    )

    return LaunchDescription([erp42_serial_bridge])