import os
from launch.actions import ExecuteProcess
from launch import LaunchDescription
from launch.substitutions import Command, PathJoinSubstitution, LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():

    use_sim_time = LaunchConfiguration("use_sim_time")
    pkg_path = FindPackageShare('erp42_gazebo').find('erp42_gazebo')
    world_file = os.path.join(pkg_path, 'worlds', 'empty.world')
    xacro_file = PathJoinSubstitution([FindPackageShare('erp42_gazebo'), 'urdf', 'erp42_gazebo.xacro'])
    robot_description = ParameterValue(Command(['xacro ', xacro_file]),value_type=str)

    start_gazebo_server = ExecuteProcess(
        cmd=['gzserver',
            '-s', 'libgazebo_ros_init.so',
            '-s', 'libgazebo_ros_factory.so',
            world_file
        ],
        output='screen'
    )

    start_gazebo_client = ExecuteProcess(
        cmd=['gzclient'],
        output='screen'
    )

    robot_state_publisher = Node(
        namespace='/erp42',
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description, 
            'use_sim_time': use_sim_time
        }]
    )

    gazebo_bridge = Node(
        package='erp42_gazebo',
        executable='gazebo_bridge',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    spawn_entity = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
        arguments=[
            "-topic", "/erp42/robot_description",
            "-entity", "erp42",
            "-x", "0.0", 
            "-y", "0.0", 
            "-z", "0.2"
        ],
    )

    world_to_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='world_to_odom_broadcaster',
        output='screen',
        parameters=[{"use_sim_time": use_sim_time}],
        arguments=['0','0','0','0', '0', '0', 
            'world', 'odom'
        ]
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
            description="Use simulation (Gazebo) clock"
        ),
        start_gazebo_server,
        start_gazebo_client,
        robot_state_publisher,
        gazebo_bridge,
        spawn_entity,
        world_to_odom,
    ])