from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.actions import Node

def generate_launch_description():

    # Simulation time usage flag
    use_sim_time = DeclareLaunchArgument('use_sim_time', default_value='false')

    # Robot description files
    xacro_file = PathJoinSubstitution([FindPackageShare('erp42_description'), 'urdf', 'erp42.xacro'])
    robot_description = ParameterValue(Command(['xacro ', xacro_file]), value_type=str)

    # Robot state publisher node
    robot_state_publisher = Node(
        namespace='/erp42', 
        package='robot_state_publisher', 
        executable='robot_state_publisher', 
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }]   
    )

    # Joint state publisher node
    joint_state_publisher = Node(
        namespace='/erp42',
        package='joint_state_publisher', 
        executable='joint_state_publisher', 
        output='screen',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )

    return LaunchDescription([
        use_sim_time,
        robot_state_publisher,
        joint_state_publisher
    ])