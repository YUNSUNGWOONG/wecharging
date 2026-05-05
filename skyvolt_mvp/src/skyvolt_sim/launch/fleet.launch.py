"""Launch Gazebo + N robots + the FleetManager.

Usage:
    ros2 launch skyvolt_sim fleet.launch.py num_robots:=3
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _spawn_robots(context):
    n = int(LaunchConfiguration("num_robots").perform(context))
    desc_pkg = FindPackageShare("skyvolt_description")
    robot_xacro = PathJoinSubstitution([desc_pkg, "urdf", "skyvolt_robot.urdf.xacro"])

    actions = []
    for i in range(n):
        rid = f"r{i}"
        actions += [
            Node(
                package="robot_state_publisher", executable="robot_state_publisher",
                namespace=rid,
                parameters=[{"robot_description": Command([
                    "xacro ", robot_xacro, " robot_id:=", rid])}],
            ),
            Node(
                package="ros_gz_sim", executable="create",
                arguments=[
                    "-name", rid,
                    "-topic", f"/{rid}/robot_description",
                    "-x", str(-1.5 + 0.8*i), "-y", "0.8", "-z", "2.2",
                ],
            ),
            Node(
                package="skyvolt_localization", executable="localizer_node",
                namespace=rid,
                parameters=[{"robot_id": rid}],
            ),
            Node(
                package="skyvolt_localization", executable="speed_policy_node",
                namespace=rid,
                parameters=[{"robot_id": rid}],
            ),
        ]
    return actions


def generate_launch_description():
    sim_pkg = FindPackageShare("skyvolt_sim")
    ros_gz_sim = FindPackageShare("ros_gz_sim")
    world = PathJoinSubstitution([sim_pkg, "worlds", "parking_lot.sdf"])

    return LaunchDescription([
        DeclareLaunchArgument("num_robots", default_value="3"),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([ros_gz_sim, "/launch/gz_sim.launch.py"]),
            launch_arguments={"gz_args": [world, " -r"]}.items(),
        ),

        Node(
            package="skyvolt_fleet", executable="fleet_manager_node",
            output="screen",
        ),

        OpaqueFunction(function=_spawn_robots),
    ])
