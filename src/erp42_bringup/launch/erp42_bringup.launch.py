# erp42_bringup.launch.py

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, LogInfo, Shutdown
from launch.substitutions import Command, LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node

from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def check_conflict(context, *args, **kwargs):
    
    view_control_panel    = LaunchConfiguration('view_control_panel').perform(context)
    view_feedback_monitor = LaunchConfiguration('view_feedback_monitor').perform(context)

    if view_control_panel.lower() == 'true' and view_feedback_monitor.lower() == 'true':
        return [
            LogInfo(msg="view_control_panel and view_feedback_monitor cannot both be true. "
                "Since control_panel already includes feedback_monitor, choose only one of them."
            ),
            Shutdown()
        ]

    return []

def generate_launch_description():
    
    # ERP42 serial bridge parameter file
    erp42_bringup_parameter_file = DeclareLaunchArgument('erp42_bringup_parameter_file', 
        default_value=PathJoinSubstitution([
            FindPackageShare('erp42_bringup'), 'config', 'erp42_bringup.param.yaml'
        ])
    )

    # GUI option
    view_control_panel    = DeclareLaunchArgument('view_control_panel'   , default_value = 'false')
    view_feedback_monitor = DeclareLaunchArgument('view_feedback_monitor', default_value = 'false')

    # ERP42 descriptions
    launch_vehicle_description = DeclareLaunchArgument('launch_vehicle_description', default_value = 'false')
    launch_rviz                = DeclareLaunchArgument('launch_rviz'               , default_value = 'false')

    # Description file
    vehicle_description_file = DeclareLaunchArgument('vehicle_description_file', 
        default_value = PathJoinSubstitution([
            FindPackageShare('erp42_description'), 'urdf', 'erp42.xacro'
        ])
    )

    # Rviz config file
    rviz_config_file = DeclareLaunchArgument('rviz_config_file', 
        default_value = PathJoinSubstitution([
            FindPackageShare('erp42_description'), 'rviz', 'vehicle_description.rviz'
        ])
    )

    # ERP42 serial bridge
    erp42_serial_bridge = Node(
        package    = 'erp42_serial', 
        executable = 'serial_bridge', 
        name       = 'erp42_serial_bridge', 
        output     = 'screen',
        parameters = [{LaunchConfiguration('erp42_bringup_parameter_file')}]
    )

    # Control panel GUI
    erp42_control_panel = Node(
        package    = 'erp42_gui', 
        executable = 'control_panel', 
        output     = 'screen',
        condition  = IfCondition(LaunchConfiguration('view_control_panel'))
    )

    # Feedback monitor GUI
    erp42_feedback_monitor = Node(
        package    = 'erp42_gui', 
        executable = 'feedback_monitor', 
        output     = 'screen',
        condition  = IfCondition(LaunchConfiguration('view_feedback_monitor'))
    )

    # Robot state publisher node
    robot_state_publisher = Node(
        namespace  = '/erp42',
        package    = 'robot_state_publisher',
        executable = 'robot_state_publisher',
        output     = 'screen',
        parameters = [{'robot_description': Command(['xacro ', LaunchConfiguration('vehicle_description_file')])}],
        condition  = IfCondition(LaunchConfiguration('launch_vehicle_description'))
    )

    # Joint state publisher node
    joint_state_publisher = Node(
        namespace  = '/erp42',
        package    = 'joint_state_publisher', 
        executable = 'joint_state_publisher', 
        output     = 'screen',
        condition  = IfCondition(LaunchConfiguration('launch_vehicle_description'))
    )

    # Rviz
    rviz = Node(
        namespace  = '/erp42',
        package    = 'rviz2',
        executable = 'rviz2',
        name       = 'rviz2',
        output     = 'screen',
        arguments  = ['-d', LaunchConfiguration('rviz_config_file')],
        condition  = IfCondition(LaunchConfiguration('launch_rviz'))
    )

    return LaunchDescription([

        erp42_bringup_parameter_file,
        view_control_panel,
        view_feedback_monitor,
        OpaqueFunction(function=check_conflict),
        launch_vehicle_description,
        launch_rviz,
        vehicle_description_file,
        rviz_config_file,

        erp42_serial_bridge,
        erp42_control_panel,
        robot_state_publisher,
        joint_state_publisher,
        rviz
    ])