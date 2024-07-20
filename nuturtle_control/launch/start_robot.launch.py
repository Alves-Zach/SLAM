from launch import LaunchDescription, actions
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import (
    Command, PathJoinSubstitution, NotEqualsSubstitution, EqualsSubstitution,
    AndSubstitution, LaunchConfiguration, TextSubstitution)
from launch.conditions import IfCondition
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        # Launch agruments
        DeclareLaunchArgument(name="cmd_src", default_value="circle",
                              description="Selects the source of the cmd_vel publisher",
                              choices=["circle", "teleop", "none"]),

        DeclareLaunchArgument(name="robot", default_value="nusim",
                              description="Selects whether the robot will be simulated or real",
                              choices=["nusim", "localhost", "none"]),

        DeclareLaunchArgument(name="use_rviz", default_value="true",
                              description="Whether rviz2 should be launched or not",
                              choices=["true", "false"]),

        DeclareLaunchArgument(name="config_file", default_value="diff_params.yaml",
                              description="The name of the configuration yaml file"),

        DeclareLaunchArgument(name="draw_only", default_value="false",
                              description="Selects whether to just draw the configured objects or not",
                              choices=["true", "false"]),

        DeclareLaunchArgument(name="use_slam", default_value="false",
                              description="Used by the slam launch file",
                              choices=["true", "false"]),

        DeclareLaunchArgument(name="use_lidar", default_value="true",
                              description="Used by the slam launch file",
                              choices=["true", "false"]),

        # Nodes to be launched
        # cmd_vel source node
        Node(package="nuturtle_control", executable="circle",
             parameters=[PathJoinSubstitution([FindPackageShare("nuturtle_description"),
                                               "config",
                                               LaunchConfiguration("config_file")]),],
             condition=IfCondition(EqualsSubstitution(LaunchConfiguration('cmd_src'), "circle")),
             on_exit=actions.Shutdown()
             ),

        # Teleop twist keyboard
        Node(package="teleop_twist_keyboard", executable="teleop_twist_keyboard",
             output="screen",
             condition=IfCondition(EqualsSubstitution(LaunchConfiguration("cmd_src"), "teleop")),
             on_exit=actions.Shutdown()
             ),

        # Robot source
        # nusim node
        Node(package="nusim", executable="nusim", remappings=[("/sensor_data", "/red/sensor_data"),
                                                              ("/wheel_cmd", "/red/wheel_cmd")],
             parameters=[PathJoinSubstitution([FindPackageShare("nuturtle_description"),
                                               "config",
                                               LaunchConfiguration("config_file")]),
                         PathJoinSubstitution([FindPackageShare("nusim"),
                                               "config", "basic_world.yaml"]),
                         {"use_lidar": LaunchConfiguration("use_lidar")}],
             condition=IfCondition(AndSubstitution(EqualsSubstitution(LaunchConfiguration("robot"), "nusim"),
                                   EqualsSubstitution(LaunchConfiguration("draw_only"), "false"))),
             on_exit=actions.Shutdown()
             ),

        Node(package="nusim", executable="nusim", name="nuwall",
             remappings=[("/sensor_data", "/red/sensor_data"),
             ("/wheel_cmd", "/red/wheel_cmd")],
             parameters=[PathJoinSubstitution([FindPackageShare("nuturtle_description"),
                                               "config",
                                               LaunchConfiguration("config_file")]),
                         PathJoinSubstitution([FindPackageShare("nusim"),
                                               "config", "basic_world.yaml"])],
             condition=IfCondition(AndSubstitution(EqualsSubstitution(LaunchConfiguration("robot"), "nusim"),
                                   EqualsSubstitution(LaunchConfiguration("draw_only"), "true"))),
             on_exit=actions.Shutdown()
             ),

        # odometry node
        # On computer with sim
        Node(package="nuturtle_control", executable="odometry",
             remappings=[("/joint_states", "/red/joint_states")],
             parameters=[PathJoinSubstitution([FindPackageShare("nuturtle_description"),
                                               "config",
                                               LaunchConfiguration("config_file")])],
             condition=IfCondition(NotEqualsSubstitution(LaunchConfiguration("robot"), "none")),
             on_exit=actions.Shutdown()
             ),
        # On turtlebot without sim
        Node(package="nuturtle_control", executable="odometry",
             parameters=[PathJoinSubstitution([FindPackageShare("nuturtle_description"),
                                               "config",
                                               LaunchConfiguration("config_file")])],
             condition=IfCondition(EqualsSubstitution(LaunchConfiguration("robot"), "localhost")),
             on_exit=actions.Shutdown()
             ),

        # turtle_control node
        # On computer with sim
        Node(package="nuturtle_control", executable="turtle_control",
             remappings=[("/sensor_data", "/red/sensor_data"), ("/wheel_cmd", "/red/wheel_cmd"),
                         ("/joint_states", "/red/joint_states")],
             parameters=[PathJoinSubstitution([FindPackageShare("nuturtle_description"),
                                               "config",
                                               LaunchConfiguration("config_file")])],
             condition=IfCondition(EqualsSubstitution(LaunchConfiguration("robot"), "nusim")),
             on_exit=actions.Shutdown()
             ),
        # On turtlebot without sim
        Node(package="nuturtle_control", executable="turtle_control",
             parameters=[PathJoinSubstitution([FindPackageShare("nuturtle_description"),
                                               "config",
                                               LaunchConfiguration("config_file")])],
             condition=IfCondition(NotEqualsSubstitution(LaunchConfiguration("robot"), "nusim")),
             on_exit=actions.Shutdown()
             ),

        # Rviz node
        # robot == nusim
        Node(package="rviz2", executable="rviz2",
             arguments=["-d",
                        PathJoinSubstitution([FindPackageShare("nuturtle_control"),
                                              "config", "blue_red.rviz"])],
             condition=IfCondition(AndSubstitution(AndSubstitution(AndSubstitution(
                        EqualsSubstitution(LaunchConfiguration("use_rviz"), "true"),
                        EqualsSubstitution(LaunchConfiguration("robot"), "nusim")),
                    EqualsSubstitution(LaunchConfiguration("draw_only"), "false")),
                EqualsSubstitution(LaunchConfiguration("use_slam"), "false"))),
             on_exit=actions.Shutdown()
             ),
        Node(package="rviz2", executable="rviz2",
             arguments=["-d",
                        PathJoinSubstitution([FindPackageShare("nuturtle_control"),
                                              "config", "blue_red_draw_only.rviz"])],
             condition=IfCondition(AndSubstitution(AndSubstitution(
                     EqualsSubstitution(LaunchConfiguration("use_rviz"), "true"),
                     EqualsSubstitution(LaunchConfiguration("robot"), "nusim")),
                 EqualsSubstitution(LaunchConfiguration("draw_only"), "true"))),
             on_exit=actions.Shutdown()
             ),
        # robot != nusim
        Node(package="rviz2", executable="rviz2",
             arguments=["-d",
                        PathJoinSubstitution([FindPackageShare("nuturtle_control"),
                                              "config", "blue.rviz"])],
             condition=IfCondition(AndSubstitution(AndSubstitution(
                 EqualsSubstitution(LaunchConfiguration("use_rviz"), "true"),
                 NotEqualsSubstitution(LaunchConfiguration("robot"), "nusim")),
                 NotEqualsSubstitution(LaunchConfiguration("robot"), "localhost"))),
             on_exit=actions.Shutdown()
             ),

        # Robot state publishers
        # Always launch blue
        Node(package="robot_state_publisher", executable="robot_state_publisher", namespace="blue",
             parameters=[
                    {"robot_description":
                     Command([TextSubstitution(text="xacro "),
                              PathJoinSubstitution(
                              [FindPackageShare("nuturtle_description"),
                               "urdf", "turtlebot3_burger.urdf.xacro"]),
                              TextSubstitution(text=" color:="), "blue"]),
                        "frame_prefix": "blue/"}],
             condition=IfCondition(NotEqualsSubstitution(
                LaunchConfiguration("robot"), "localhost")),
             on_exit=actions.Shutdown()
             ),
        # Launch red if robot:= nusim
        Node(package="robot_state_publisher", executable="robot_state_publisher", namespace="red",
             parameters=[
                {"robot_description":
                 Command([TextSubstitution(text="xacro "),
                          PathJoinSubstitution(
                          [FindPackageShare("nuturtle_description"),
                           "urdf", "turtlebot3_burger.urdf.xacro"]),
                          TextSubstitution(text=" color:="), "red"]),
                    "frame_prefix": "red/"}],
             condition=IfCondition(AndSubstitution(
                EqualsSubstitution(LaunchConfiguration("use_rviz"), "true"),
                EqualsSubstitution(LaunchConfiguration("robot"), "nusim"))),
             on_exit=actions.Shutdown()
             ),

        # Static transformations
        Node(package="tf2_ros", executable="static_transform_publisher",
             name="static_transform_odom",
             arguments=['--frame-id', 'nusim/world', '--child-frame-id', 'uncorrected_odom'],
             condition=IfCondition(NotEqualsSubstitution(
                  LaunchConfiguration("robot"), "localhost")),
             on_exit=actions.Shutdown()),

        # Node only to run on the real robot
        Node(package="numsr_turtlebot", executable="numsr_turtlebot",
             condition=IfCondition(EqualsSubstitution(LaunchConfiguration("robot"), "localhost")),
             on_exit=actions.Shutdown())
    ])
