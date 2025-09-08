# erp42_bringup.launch.py

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node

from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    
    # GUI option : Feedback monitor
    view_feedback_monitor = DeclareLaunchArgument(
        'view_feedback_monitor', 
        default_value = 'false', 
        description = 'Whether to launch the feedback monitor'
    )
    erp42_feedback_monitor = Node(
        package = 'erp42_gui', 
        executable = 'feedback_monitor', 
        name = 'erp42_feedback_monitor', 
        output = 'screen',
        condition = IfCondition(LaunchConfiguration('view_feedback_monitor'))
    )

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

    return LaunchDescription([
        view_feedback_monitor,
        erp42_feedback_monitor,
        erp42_serial_bridge
    ])