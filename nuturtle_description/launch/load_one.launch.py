from launch import LaunchDescription, actions
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, SetLaunchConfiguration
from launch.substitutions import (
    Command, PathJoinSubstitution, TextSubstitution, EqualsSubstitution,
    LaunchConfiguration)
from launch.conditions import IfCondition
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(name="use_jsp", default_value="gui",
                              description="gui (default): use jsp_gui, " +
                              "jsp: use joint_state_publisher, none: no joint states published"),

        DeclareLaunchArgument(name="use_rviz", default_value="true",
                              description="true (default): start rviz otherwise don't start rviz"),

        DeclareLaunchArgument(name="color", default_value="purple",
                              description="purple (default): the color of the turtlebot " +
                              "(purple, red, green, or blue)",
                              choices=['purple', 'red', 'green', 'blue']),
        SetLaunchConfiguration(name="rviz_config", value=["basic_", LaunchConfiguration('color'),
                                                          ".rviz"]),

        Node(package="joint_state_publisher_gui",
             executable="joint_state_publisher_gui",
             namespace=LaunchConfiguration('color'),
             condition=IfCondition(
                    EqualsSubstitution(LaunchConfiguration('use_jsp'), "gui"))),
        Node(package="joint_state_publisher",
             executable="joint_state_publisher",
             namespace=LaunchConfiguration('color'),
             condition=IfCondition(
                    EqualsSubstitution(LaunchConfiguration('use_jsp'), "true"))),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            namespace=LaunchConfiguration('color'),
            parameters=[
                {"robot_description":
                 Command([TextSubstitution(text="xacro "),
                          PathJoinSubstitution(
                              [FindPackageShare("nuturtle_description"),
                               "urdf", "turtlebot3_burger.urdf.xacro"]),
                          TextSubstitution(text=" color:="),
                          LaunchConfiguration('color')]),
                 "frame_prefix": [LaunchConfiguration('color'), "/"]}]
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            namespace=LaunchConfiguration('color'),
            arguments=["-d",
                       PathJoinSubstitution(
                           [FindPackageShare("nuturtle_description"), "config",
                            LaunchConfiguration('rviz_config')])],
            condition=IfCondition(
                    EqualsSubstitution(LaunchConfiguration('use_rviz'), "true")),
            on_exit=actions.Shutdown()
        )
        ])
