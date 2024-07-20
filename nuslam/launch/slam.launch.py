import os
from launch import LaunchDescription, actions
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import (
    Command, PathJoinSubstitution, EqualsSubstitution,
    LaunchConfiguration, TextSubstitution)
from launch.conditions import IfCondition
from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription
from ament_index_python import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(name="config_file", default_value="diff_params.yaml",
                              description="The name of the configuration yaml file"),

        DeclareLaunchArgument(name="use_lidar", default_value="false",
                              description="Used by the slam launch file",
                              choices=["true", "false"]),

        # Including the start_robot launch file
        IncludeLaunchDescription(
            launch_description_source=PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('nuturtle_control'),
                        'launch/start_robot.launch.py')),
            launch_arguments={
                "use_slam": "true",
                "use_lidar": LaunchConfiguration("use_lidar")
            }.items()
        ),

        # rviz node
        Node(package="rviz2", executable="rviz2",
             arguments=["-d",
                        PathJoinSubstitution([FindPackageShare("nuslam"),
                                              "config", "blue_red_green.rviz"])],
             on_exit=actions.Shutdown()
             ),

        # slam node
        Node(package="nuslam", executable="slam",
             parameters=[PathJoinSubstitution([FindPackageShare("nuturtle_description"),
                                                "config",
                                               LaunchConfiguration("config_file")]),
                         {"use_lidar": LaunchConfiguration("use_lidar")}],
             condition=IfCondition(EqualsSubstitution(LaunchConfiguration("use_slam"), "true")),
             on_exit=actions.Shutdown()
             ),

        # Launch green robot
        Node(package="robot_state_publisher", executable="robot_state_publisher", namespace="green",
             parameters=[
                    {"robot_description":
                     Command([TextSubstitution(text="xacro "),
                              PathJoinSubstitution(
                              [FindPackageShare("nuturtle_description"),
                               "urdf", "turtlebot3_burger.urdf.xacro"]),
                              TextSubstitution(text=" color:="), "green"]),
                        "frame_prefix": "green/"}],
             on_exit=actions.Shutdown()
             ),
    ])