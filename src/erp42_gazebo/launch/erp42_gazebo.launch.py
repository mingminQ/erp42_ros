import os
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription, ExecuteProcess # ExecuteProcess 추가
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch import LaunchDescription
from launch.substitutions import Command, PathJoinSubstitution, LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import xacro
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    pkg_path = FindPackageShare('erp42_gazebo').find('erp42_gazebo')
    world_file = os.path.join(pkg_path, 'worlds', 'empty.world')

    # --- 수정된 부분: Gazebo를 직접 실행 ---
    # 기존 gazebo_launch 변수 정의를 아래 내용으로 교체합니다.
    
    # 1. Gazebo 서버(gzserver) 실행
    #    -s 옵션과 라이브러리 이름을 별도의 인자로 분리합니다.
    start_gazebo_server_cmd = ExecuteProcess(
        cmd=[
            'gzserver',
            '-s', 'libgazebo_ros_init.so',
            '-s', 'libgazebo_ros_factory.so',
            '--verbose',
            world_file
        ],
        additional_env={
            'GAZEBO_PLUGIN_PATH': '/opt/ros/humble/lib',
            'LD_LIBRARY_PATH': '/opt/ros/humble/lib'
        },
        output='screen'
    )

    # 2. Gazebo 클라이언트(GUI) 실행
    start_gazebo_client_cmd = ExecuteProcess(
        cmd=['gzclient'],
        output='screen'
    )

    xacro_file = PathJoinSubstitution([
        FindPackageShare('erp42_gazebo'), 'urdf', 'erp42_gazebo.xacro'
    ])

    xacro_file = PathJoinSubstitution([
        FindPackageShare('erp42_gazebo'), 'urdf', 'erp42_gazebo.xacro'
    ])
    robot_description = ParameterValue(
        Command(['xacro ', xacro_file]),
        value_type=str
    )
    robot_launch = Node(
        namespace='/erp42',
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description, 'use_sim_time': use_sim_time}],
        output='screen'
    )

    gazebo_bridge = Node(
        package='erp42_gazebo',
        executable='gazebo_bridge',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    spawner = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
        arguments=[
            "-topic", "/erp42/robot_description",
            "-entity", "erp42",
            "-x", "0.0", "-y", "0.0", "-z", "0.2", "-Y", "0.0"    #track1
        ],
    )

    world_to_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='world_to_odom_broadcaster',
        output='screen',
        parameters=[{"use_sim_time": use_sim_time}],
        arguments=[
            '0',   # x offset
            '0',   # y offset
            '0',   # z offset
            '0',   # roll
            '0',   # pitch
            '0',   # yaw
            'world',  # parent frame
            'odom'    # child frame
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
            description="Use simulation (Gazebo) clock"
        ),
        start_gazebo_server_cmd,
        start_gazebo_client_cmd,
        robot_launch,
        gazebo_bridge,
        spawner,
        world_to_odom,
    ])