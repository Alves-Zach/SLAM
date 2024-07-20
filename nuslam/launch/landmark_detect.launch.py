import os
from launch import LaunchDescription, actions
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import (
    Command, PathJoinSubstitution, NotEqualsSubstitution, EqualsSubstitution,
    AndSubstitution, LaunchConfiguration, TextSubstitution)
from launch.conditions import IfCondition
from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription
from ament_index_python import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(name="config_file", default_value="diff_params.yaml",
                              description="The name of the configuration yaml file"),

        # Including the start_robot launch file
        IncludeLaunchDescription(
            launch_description_source=PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('nuslam'),
                        'launch/slam.launch.py')),
            launch_arguments={
                "use_slam": "true",
                "use_lidar": "true",
            }.items()
        ),

        # # slam node
        # Node(package="nuslam", executable="landmarks",
        #      on_exit=actions.Shutdown()
        #      ),
    ])