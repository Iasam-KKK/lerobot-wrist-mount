"""Model-only view - no physics, no Gazebo.

    ros2 launch so101_wrist_cam_description display.launch.py

Useful for eyeballing the camera frame against the arm and for checking the
URDF parses, without waiting for a simulator to start.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


PKG = "so101_wrist_cam_description"


def generate_launch_description():
    pkg = FindPackageShare(PKG)
    xacro_file = PathJoinSubstitution([pkg, "urdf", "so101_wrist_cam.urdf.xacro"])

    robot_description = ParameterValue(
        Command([
            "xacro ", xacro_file,
            " sim:=false",
            " camera_tilt:=", LaunchConfiguration("camera_tilt"),
        ]),
        value_type=str,
    )

    return LaunchDescription([
        DeclareLaunchArgument("camera_tilt", default_value="25.0"),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="screen",
            parameters=[{"robot_description": robot_description}],
        ),
        Node(
            package="joint_state_publisher_gui",
            executable="joint_state_publisher_gui",
            output="screen",
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            output="screen",
            arguments=["-d", PathJoinSubstitution([pkg, "rviz", "wrist_cam.rviz"])],
        ),
    ])
