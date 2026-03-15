# serial_bridge.launch.py

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():
    
    # ERP42 serial bridge parameter file
    serial_bridge_paraemters = os.path.join(
        get_package_share_directory("erp42_serial"), 
        "config", 
        "serial_bridge.param.yaml"
    )

    # ERP42 serial bridge
    serial_bridge = Node(
        package    = 'erp42_serial', 
        executable = 'serial_bridge', 
        name       = 'serial_bridge', 
        output     = 'screen',
        parameters = [serial_bridge_paraemters]
    )

    return LaunchDescription([
        serial_bridge
    ])