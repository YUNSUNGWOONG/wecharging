"""Headless URDF check: launches robot_state_publisher + joint_state_publisher_gui."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg = FindPackageShare("skyvolt_description")
    xacro_path = PathJoinSubstitution([pkg, "urdf", "skyvolt_robot.urdf.xacro"])

    return LaunchDescription([
        DeclareLaunchArgument("robot_id", default_value="r0"),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[{
                "robot_description": Command([
                    "xacro ", xacro_path,
                    " robot_id:=", LaunchConfiguration("robot_id"),
                ]),
            }],
        ),
        Node(
            package="joint_state_publisher_gui",
            executable="joint_state_publisher_gui",
        ),
    ])
