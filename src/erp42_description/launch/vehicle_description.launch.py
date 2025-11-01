# vehicle_description.launch.py

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, PathJoinSubstitution, LaunchConfiguration

from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

def generate_launch_description():

    # Robot description file
    vehicle_description_file = DeclareLaunchArgument('vehicle_description_file', 
        default_value = PathJoinSubstitution([
            FindPackageShare('erp42_description'), 'urdf', 'erp42.xacro'
        ])
    )

    # Rviz configuration file
    rviz_config_file = DeclareLaunchArgument('rviz_config_file',
        default_value = PathJoinSubstitution([
            FindPackageShare('erp42_description'), 'rviz', 'vehicle_description.rviz'
        ])
    )

    # Robot state publisher node
    robot_state_publisher = Node(
        namespace  = 'erp42',
        package    = 'robot_state_publisher',
        executable = 'robot_state_publisher',
        output     = 'screen',
        parameters = [{
            'robot_description': Command(['xacro ', LaunchConfiguration('vehicle_description_file')]),
            'publish_frequency': 100.0
        }]
    )

    # Joint state publisher node
    joint_state_publisher = Node(
        namespace  = 'erp42',
        package    = 'joint_state_publisher', 
        executable = 'joint_state_publisher', 
        output     = 'screen'
    )

    # Rviz
    rviz = Node(
        namespace  = 'erp42',
        package    = 'rviz2',
        executable = 'rviz2',
        name       = 'rviz2',
        output     = 'screen',
        arguments  = ['-d', LaunchConfiguration('rviz_config_file')],
    )

    return LaunchDescription([
        vehicle_description_file,
        rviz_config_file,
        robot_state_publisher,
        joint_state_publisher,
        rviz
    ])