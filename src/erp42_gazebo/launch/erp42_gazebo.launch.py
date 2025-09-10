# erp42_gazebo.launch.py

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import Command, PathJoinSubstitution, LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    # Simulation time usage flag
    use_sim_time = DeclareLaunchArgument('use_sim_time', default_value = 'true')

    # World file
    world_file = DeclareLaunchArgument('world_file', 
        default_value = PathJoinSubstitution([FindPackageShare('erp42_gazebo'), 'worlds', 'empty.world'])
    )

    # Robot description file
    description_file = DeclareLaunchArgument('description_file', 
        default_value = PathJoinSubstitution([FindPackageShare('erp42_gazebo'), 'urdf', 'erp42_gazebo.xacro'])
    )

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
            'robot_description': Command(['xacro ', LaunchConfiguration('description_file')]),
            'use_sim_time'     : LaunchConfiguration('use_sim_time')
        }]
    )

    # ERP42 Gazebo bridge node
    gazebo_bridge  = Node(
        package    = 'erp42_gazebo',
        executable = 'gazebo_bridge',
        output     = 'screen',
        parameters = [{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )

    # Spawn ERP42 entity in Gazebo
    spawn_entity = Node(
        package    = "gazebo_ros",
        executable = "spawn_entity.py",
        output     = "screen",
        parameters = [{"use_sim_time": LaunchConfiguration('use_sim_time')}],
        arguments  = [
            "-topic" , "/erp42/robot_description",
            "-entity", "erp42",
            "-x"     , "0.0", 
            "-y"     , "0.0", 
            "-z"     , "0.2"
        ],
    )

    return LaunchDescription([
        use_sim_time,
        world_file,
        description_file,
        gazebo_server,
        gazebo_client,
        robot_state_publisher,
        gazebo_bridge,
        spawn_entity
    ])