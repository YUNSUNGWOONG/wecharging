"""Launch Gazebo Garden with the parking-lot world and one SkyvoltRobot."""
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    sim_pkg = FindPackageShare("skyvolt_sim")
    desc_pkg = FindPackageShare("skyvolt_description")
    ros_gz_sim = FindPackageShare("ros_gz_sim")

    world = PathJoinSubstitution([sim_pkg, "worlds", "parking_lot.sdf"])
    robot_xacro = PathJoinSubstitution([desc_pkg, "urdf", "skyvolt_robot.urdf.xacro"])
    bridge_yaml = PathJoinSubstitution([sim_pkg, "config", "bridge.yaml"])

    return LaunchDescription([
        DeclareLaunchArgument("robot_id", default_value="r0"),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([ros_gz_sim, "/launch/gz_sim.launch.py"]),
            launch_arguments={"gz_args": [world, " -r"]}.items(),
        ),

        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            namespace=LaunchConfiguration("robot_id"),
            parameters=[{
                "robot_description": Command([
                    "xacro ", robot_xacro,
                    " robot_id:=", LaunchConfiguration("robot_id"),
                ]),
            }],
        ),

        Node(
            package="ros_gz_sim", executable="create",
            arguments=[
                "-name", LaunchConfiguration("robot_id"),
                "-topic", ["/", LaunchConfiguration("robot_id"), "/robot_description"],
                "-x", "-1.5", "-y", "0.8", "-z", "2.2",
            ],
        ),

        Node(
            package="ros_gz_bridge", executable="parameter_bridge",
            parameters=[{"config_file": bridge_yaml}],
        ),

        # Algorithm nodes — see other packages for the implementations.
        Node(
            package="skyvolt_localization", executable="localizer_node",
            namespace=LaunchConfiguration("robot_id"),
            parameters=[{"robot_id": LaunchConfiguration("robot_id")}],
        ),
        Node(
            package="skyvolt_localization", executable="speed_policy_node",
            namespace=LaunchConfiguration("robot_id"),
            parameters=[{"robot_id": LaunchConfiguration("robot_id")}],
        ),
    ])
