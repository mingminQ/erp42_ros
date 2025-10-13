# erp42_gazebo.launch.py

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, LogInfo, Shutdown
from launch.substitutions import Command, PathJoinSubstitution, LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    # ERP42 gazebo parameter file
    erp42_gazebo_parameter_file = DeclareLaunchArgument('erp42_gazebo_parameter_file', 
        default_value=PathJoinSubstitution([
            FindPackageShare('erp42_gazebo'), 'config', 'erp42_gazebo.param.yaml'
        ])
    )

    # World file
    world_file = DeclareLaunchArgument('world_file', 
        default_value = PathJoinSubstitution([FindPackageShare('erp42_gazebo'), 'worlds', 'empty.world'])
    )

    # Robot description file
    vehicle_description_file = DeclareLaunchArgument('vehicle_description_file', 
        default_value = PathJoinSubstitution([FindPackageShare('erp42_gazebo'), 'urdf', 'erp42_gazebo.xacro'])
    )

    # Rviz configuration file
    rviz_config_file = DeclareLaunchArgument('rviz_config_file',
        default_value = PathJoinSubstitution([
            FindPackageShare('erp42_gazebo'), 'rviz', 'erp42_gazebo.rviz'
        ])
    )

    # ERP42 spawn location
    spawn_x     = DeclareLaunchArgument('spawn_x'    , default_value = '0.0')
    spawn_y     = DeclareLaunchArgument('spawn_y'    , default_value = '0.0')
    spawn_z     = DeclareLaunchArgument('spawn_z'    , default_value = '0.0')
    spawn_roll  = DeclareLaunchArgument('spawn_roll' , default_value = '0.0')
    spawn_pitch = DeclareLaunchArgument('spawn_pitch', default_value = '0.0')
    spawn_yaw   = DeclareLaunchArgument('spawn_yaw'  , default_value = '0.0')

    # Gazebo server
    gazebo_server = ExecuteProcess(
        output='screen',
        cmd = ['gzserver',
            '-s', 'libgazebo_ros_init.so',
            '-s', 'libgazebo_ros_factory.so',
            LaunchConfiguration('world_file')
        ]
    )

    # Gazebo client
    gazebo_client = ExecuteProcess(
        output='screen',
        cmd=['gzclient'],
    )

    # Robot state publisher node
    robot_state_publisher = Node(
        namespace  = '/erp42',
        package    = 'robot_state_publisher',
        executable = 'robot_state_publisher',
        output     = 'screen',
        parameters = [{
            'robot_description': Command(['xacro ', LaunchConfiguration('vehicle_description_file')]),
            'publish_frequency': 100.0
            },
            LaunchConfiguration('erp42_gazebo_parameter_file')
        ]
    )

    # ERP42 Gazebo bridge node
    gazebo_bridge  = Node(
        package    = 'erp42_gazebo',
        executable = 'gazebo_bridge',
        output     = 'screen',
        parameters = [LaunchConfiguration('erp42_gazebo_parameter_file')],
    )

    # Spawn ERP42 entity in Gazebo
    spawn_entity = Node(
        package    = "gazebo_ros",
        executable = "spawn_entity.py",
        output     = "screen",
        parameters = [LaunchConfiguration('erp42_gazebo_parameter_file')],
        arguments  = [
            "-topic" , "/erp42/robot_description",
            "-entity", "erp42",
            "-x"     , LaunchConfiguration('spawn_x'    ),
            "-y"     , LaunchConfiguration('spawn_y'    ),
            "-z"     , LaunchConfiguration('spawn_z'    ),
            "-R"     , LaunchConfiguration('spawn_roll' ),
            "-P"     , LaunchConfiguration('spawn_pitch'),
            "-Y"     , LaunchConfiguration('spawn_yaw'  )
        ],
    )

    # Rviz
    rviz = Node(
        namespace  = '/erp42',
        package    = 'rviz2',
        executable = 'rviz2',
        name       = 'rviz2',
        output     = 'screen',
        arguments  = ['-d', LaunchConfiguration('rviz_config_file')],
        parameters = [LaunchConfiguration('erp42_gazebo_parameter_file')]
    )

    return LaunchDescription([
        erp42_gazebo_parameter_file,
        world_file,
        vehicle_description_file,
        rviz_config_file,
        spawn_x,
        spawn_y,
        spawn_z,
        spawn_roll,
        spawn_pitch,
        spawn_yaw,
        gazebo_server,
        gazebo_client,
        robot_state_publisher,
        gazebo_bridge,
        spawn_entity,
        rviz
    ])