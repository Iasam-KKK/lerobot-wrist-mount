"""One-command SO-101 wrist-camera simulation.

    ros2 launch so101_wrist_cam_description sim.launch.py

Brings up Gazebo Harmonic with the pick bench, spawns the arm with the wrist
camera under ros2_control, bridges the image to ROS 2, and opens RViz showing
the live wrist feed alongside the TF tree.

Add `demo:=true` to also drive a scripted sweep over the bench.
Use `control:=false` for the older passive arm (no controllers).
"""
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            RegisterEventHandler, TimerAction)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (Command, LaunchConfiguration, PathJoinSubstitution,
                                  PythonExpression)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


PKG = "so101_wrist_cam_description"
WORLD_NAME = "pick_bench"


def generate_launch_description():
    pkg = FindPackageShare(PKG)
    control = LaunchConfiguration("control")

    args = [
        DeclareLaunchArgument("rviz", default_value="true",
                              description="Open RViz with the wrist camera view."),
        DeclareLaunchArgument("gui", default_value="true",
                              description="Show the Gazebo GUI (false = headless)."),
        DeclareLaunchArgument("control", default_value="true",
                              description="Load ros2_control. false = passive arm."),
        DeclareLaunchArgument("demo", default_value="false",
                              description="Run the scripted bench sweep after startup."),
        DeclareLaunchArgument("camera_width", default_value="640"),
        DeclareLaunchArgument("camera_height", default_value="480"),
        DeclareLaunchArgument("camera_fps", default_value="30"),
        DeclareLaunchArgument(
            "camera_tilt", default_value="25.0",
            description="Optical-axis tilt off the jaw axis, degrees. "
                        "25.0 is the measured value of the official mount.",
        ),
    ]

    xacro_file = PathJoinSubstitution([pkg, "urdf", "so101_wrist_cam.urdf.xacro"])
    world_file = PathJoinSubstitution([pkg, "worlds", f"{WORLD_NAME}.sdf"])
    rviz_cfg = PathJoinSubstitution([pkg, "rviz", "wrist_cam.rviz"])
    controllers = PathJoinSubstitution([pkg, "config", "controllers.yaml"])

    # With ros2_control, joint_state_broadcaster owns /joint_states, so the
    # bridge must NOT also publish it. Different file per mode.
    bridge_cfg = PathJoinSubstitution([
        pkg, "config",
        PythonExpression(["'gz_bridge.yaml' if '", control,
                          "'.lower() in ('true', '1') else 'gz_bridge_passive.yaml'"]),
    ])

    robot_description = ParameterValue(
        Command([
            "xacro ", xacro_file,
            " sim:=true",
            " control:=", control,
            " controllers_file:=", controllers,
            " camera_width:=", LaunchConfiguration("camera_width"),
            " camera_height:=", LaunchConfiguration("camera_height"),
            " camera_fps:=", LaunchConfiguration("camera_fps"),
            " camera_tilt:=", LaunchConfiguration("camera_tilt"),
        ]),
        value_type=str,
    )

    # "-s" runs the server only. Needed for CI and for recording without a
    # window; the sensors system still renders, so images keep flowing.
    headless = PythonExpression(
        ["'' if '", LaunchConfiguration("gui"), "'.lower() in ('true', '1') else ' -s'"]
    )

    gz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([FindPackageShare("ros_gz_sim"), "launch", "gz_sim.launch.py"])
        ]),
        launch_arguments={
            "gz_args": [world_file, " -r -v 1", headless],
            "on_exit_shutdown": "true",
        }.items(),
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description, "use_sim_time": True}],
    )

    spawn = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=["-topic", "robot_description",
                   "-name", "so101_wrist_cam",
                   "-z", "0.0"],
    )

    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        output="screen",
        parameters=[{"config_file": bridge_cfg, "use_sim_time": True}],
    )

    # ros_gz_image handles the image stream itself; it understands Gazebo's
    # image transport in a way the generic parameter_bridge does not.
    image_bridge = Node(
        package="ros_gz_image",
        executable="image_bridge",
        output="screen",
        arguments=["/wrist_cam/image"],
        parameters=[{"use_sim_time": True}],
        remappings=[("/wrist_cam/image", "/wrist_cam/image_raw")],
    )

    # Delayed on purpose. RViz and the Gazebo GUI both create GL contexts, and
    # under WSLg two at once has been seen to trigger "D3D12: Removing Device"
    # followed by "Failed to create OpenGL context", which kills Gazebo and
    # (via on_exit_shutdown) the whole launch. Not reproducible on a healthy
    # VM, so treat this as insurance rather than a proven fix; costs 5 s.
    rviz = TimerAction(
        period=5.0,
        actions=[Node(
            package="rviz2",
            executable="rviz2",
            output="screen",
            arguments=["-d", rviz_cfg],
            parameters=[{"use_sim_time": True}],
        )],
        condition=IfCondition(LaunchConfiguration("rviz")),
    )

    # --- controllers -------------------------------------------------------
    # The gz_ros2_control plugin starts controller_manager inside Gazebo, so
    # the spawners must wait for it rather than race it.
    def spawner(name):
        return Node(
            package="controller_manager",
            executable="spawner",
            output="screen",
            arguments=[name, "--controller-manager", "/controller_manager",
                       "--controller-manager-timeout", "60"],
            condition=IfCondition(control),
        )

    jsb = spawner("joint_state_broadcaster")
    arm = spawner("arm_controller")
    grip = spawner("gripper_controller")

    # Chain them: broadcaster first, then the two trajectory controllers.
    after_jsb = RegisterEventHandler(
        OnProcessExit(target_action=jsb, on_exit=[arm, grip]),
        condition=IfCondition(control),
    )

    demo = TimerAction(
        period=12.0,
        actions=[Node(package=PKG, executable="demo_motion.py", output="screen",
                      parameters=[{"use_sim_time": True}])],
        condition=IfCondition(LaunchConfiguration("demo")),
    )

    # control:=false simply omits the spawners - nothing else to add.
    return LaunchDescription(args + [gz, robot_state_publisher, spawn, bridge,
                                     image_bridge, rviz, jsb, after_jsb, demo])
