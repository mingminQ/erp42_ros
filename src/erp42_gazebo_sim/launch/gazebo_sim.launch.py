# gazebo_sim.launch.py

from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
from launch.substitutions import Command
from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():

    # Package share file path
    gazebo_sim_package       = get_package_share_directory("ros_gz_sim")
    erp42_gazebo_sim_package = get_package_share_directory("erp42_gazebo_sim")

    # erp42 gazebo sim configuration files
    gazebo_sim_bridge_config = os.path.join(
        erp42_gazebo_sim_package, "config", "gazebo_sim_bridge.config.yaml"
    )

    # Gazebo sim world file
    world = os.path.join(erp42_gazebo_sim_package, "worlds", "empty.world")

    # Robot description file
    erp42_xacro = os.path.join(erp42_gazebo_sim_package, "urdf", "erp42.xacro")

    # Rviz configuration file
    rviz_config = os.path.join(erp42_gazebo_sim_package, "rviz", "erp42_description.rviz")

    # Robot state publisher node
    robot_state_publisher = Node(
        package    = "robot_state_publisher",
        executable = "robot_state_publisher",
        name       = "robot_state_publisher",
        output     = "screen",
        parameters = [{
            "robot_description": Command(["xacro ", erp42_xacro]),
            "publish_frequency": 100.0
        }]
    )

    # Gazebo-sim
    gazebo_sim = IncludeLaunchDescription(

        # Launch source
        launch_description_source = PythonLaunchDescriptionSource(
            os.path.join(gazebo_sim_package, "launch","gz_sim.launch.py")
        ),

        # Launch arguments
        launch_arguments = { "gz_args": f"-r {world}" }.items()
    )

    # ros_gz_bridge node
    ros_gz_bridge = Node(
        package    = "ros_gz_bridge",
        executable = "parameter_bridge",
        name       = "gazebo_sim_bridge",
        output     = "screen",
        parameters = [{"config_file": gazebo_sim_bridge_config}]
    )

    # ERP42 spawner
    erp42_spawner = Node(
        package    = "ros_gz_sim",
        executable = "create",
        output     = "screen",
        arguments  = [
            "-name" , "erp42",
            "-topic", "robot_description",
            "-x"    , "0.0",
            "-y"    , "0.0",
            "-z"    , "0.3",
            "-R"    , "0.0",
            "-P"    , "0.0",
            "-Y"    , "0.0"
        ]
    )

    # Rviz
    rviz = Node(
        package    = 'rviz2',
        executable = 'rviz2',
        name       = 'rviz2',
        output     = 'screen',
        arguments  = ['-d', rviz_config],
    )

    return LaunchDescription([
        robot_state_publisher,
        gazebo_sim,
        ros_gz_bridge,
        erp42_spawner,
        rviz
    ])