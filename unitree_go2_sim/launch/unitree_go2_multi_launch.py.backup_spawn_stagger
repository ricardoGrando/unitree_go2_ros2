import os
from typing import List

import launch_ros
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    GroupAction,
    IncludeLaunchDescription,
    OpaqueFunction,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace


def _float_arg(context, name: str, default: float = 0.0) -> float:
    val = LaunchConfiguration(name).perform(context)
    try:
        return float(val)
    except Exception:
        return default


def _int_arg(context, name: str, default: int = 0) -> int:
    val = LaunchConfiguration(name).perform(context)
    try:
        return int(val)
    except Exception:
        return default


def _spawn_robots(context, *args, **kwargs) -> List[object]:
    use_sim_time = LaunchConfiguration("use_sim_time")

    # Launch args
    num_robots = max(1, _int_arg(context, "num_robots", 1))
    namespace_prefix = LaunchConfiguration("namespace_prefix").perform(context) or "go2"
    robot_name_prefix = LaunchConfiguration("robot_name_prefix").perform(context) or "go2"

    x0 = _float_arg(context, "world_init_x", 0.0)
    y0 = _float_arg(context, "world_init_y", 0.0)
    z0 = _float_arg(context, "world_init_z", 0.375)
    heading = _float_arg(context, "world_init_heading", 0.0)

    x_spacing = _float_arg(context, "x_spacing", 2.0)
    y_spacing = _float_arg(context, "y_spacing", 0.0)
    robots_per_row = _int_arg(context, "robots_per_row", 0)
    if robots_per_row <= 0:
        robots_per_row = num_robots

    unitree_go2_sim_share = get_package_share_directory("unitree_go2_sim")

    # Controller definitions (types, joints, gains).
    # In multi-robot mode each /<ns>/controller_manager is independent, so each
    # spawner must provide this YAML (otherwise you will see:
    # "The 'type' param was not defined for 'joint_group_effort_controller'").
    ros_control_yaml = os.path.join(unitree_go2_sim_share, "config", "ros_control", "ros_control.yaml")

    joints_config = os.path.join(unitree_go2_sim_share, "config", "joints", "joints.yaml")
    gait_config = os.path.join(unitree_go2_sim_share, "config", "gait", "gait.yaml")
    links_config = os.path.join(unitree_go2_sim_share, "config", "links", "links.yaml")

    description_path = LaunchConfiguration("unitree_go2_description_path")

    actions: List[object] = []

    for i in range(num_robots):
        ns = f"{namespace_prefix}_{i}"
        robot_name = f"{robot_name_prefix}_{i}"

        col = i % robots_per_row
        row = i // robots_per_row
        x = x0 + col * x_spacing
        y = y0 + row * y_spacing

        # Namespaced URDF. The xacros use robot_namespace to prefix Gazebo topics
        # and to namespace gz_ros2_control.
        robot_description = {
            "robot_description": Command(
                [
                    "xacro ",
                    description_path,
                    " robot_name:=",
                    robot_name,
                    " robot_namespace:=",
                    ns,
                    " ros2_control_params:=",
                    ros_control_yaml,
                ]
            )
        }

        # Robot State Publisher
        robot_state_publisher_node = Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            parameters=[robot_description, {"use_sim_time": use_sim_time}],
            remappings=[
                ("/tf", "tf"),
                ("/tf_static", "tf_static"),
            ],
        )

        # Gazebo spawn (create). Kept inside namespace so it listens on <ns>/robot_description.
        gazebo_spawn_robot = Node(
            package="ros_gz_sim",
            executable="create",
            name=f"spawn_{robot_name}",
            output="screen",
            arguments=[
                "-name",
                robot_name,
                "-topic",
                "robot_description",
                "-x",
                str(x),
                "-y",
                str(y),
                "-z",
                str(z0),
                "-Y",
                str(heading),
            ],
        )

        # Per-robot ROS <-> Gazebo bridges (no /clock here; a single global bridge is created once).
        gazebo_bridge = Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            name="gazebo_bridge",
            output="screen",
            parameters=[{"use_sim_time": use_sim_time}],
            arguments=[
                # Gazebo -> ROS
                f"/{ns}/imu/data@sensor_msgs/msg/Imu@gz.msgs.IMU",
                f"/{ns}/velodyne_points/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked",
                f"/{ns}/unitree_lidar/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked",
                f"/{ns}/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry",
                f"/{ns}/rgb_image@sensor_msgs/msg/Image@gz.msgs.Image",
                # Optional joint state bridge (kept for parity with the original launch)
                f"/{ns}/joint_states@sensor_msgs/msg/JointState@gz.msgs.Model",
                # ROS -> Gazebo
                f"/{ns}/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist",
                f"/{ns}/joint_group_effort_controller/joint_trajectory@trajectory_msgs/msg/JointTrajectory]gz.msgs.JointTrajectory",
            ],
        )

        # CHAMP controller nodes
        quadruped_controller_node = Node(
            package="champ_base",
            executable="quadruped_controller_node",
            name="quadruped_controller_node",
            output="screen",
            parameters=[
                {"use_sim_time": use_sim_time},
                {"gazebo": True},
                {"publish_joint_states": True},
                {"publish_joint_control": True},
                {"publish_foot_contacts": False},
                {"joint_controller_topic": "joint_group_effort_controller/joint_trajectory"},
                {
                    "urdf": Command(
                        [
                            "xacro ",
                            description_path,
                            " robot_name:=",
                            robot_name,
                            " robot_namespace:=",
                            ns,
                            " ros2_control_params:=",
                            ros_control_yaml,
                        ]
                    )
                },
                joints_config,
                links_config,
                gait_config,
                {"hardware_connected": False},
                {"close_loop_odom": True},
            ],
            remappings=[("cmd_vel/smooth", "cmd_vel"), ("/tf", "tf"), ("/tf_static", "tf_static")],
        )

        state_estimator_node = Node(
            package="champ_base",
            executable="state_estimation_node",
            name="state_estimation_node",
            output="screen",
            parameters=[
                {"use_sim_time": use_sim_time},
                {"orientation_from_imu": True},
                {
                    "urdf": Command(
                        [
                            "xacro ",
                            description_path,
                            " robot_name:=",
                            robot_name,
                            " robot_namespace:=",
                            ns,
                            " ros2_control_params:=",
                            ros_control_yaml,
                        ]
                    )
                },
                joints_config,
                links_config,
                gait_config,
            ],
            remappings=[("/tf", "tf"), ("/tf_static", "tf_static")],
        )

        # EKFs (robot_localization)
        base_to_footprint_ekf = Node(
            package="robot_localization",
            executable="ekf_node",
            name="base_to_footprint_ekf",
            output="screen",
            parameters=[
                {"base_link_frame": "base_link"},
                {"use_sim_time": use_sim_time},
                os.path.join(
                    get_package_share_directory("champ_base"),
                    "config",
                    "ekf",
                    "base_to_footprint.yaml",
                ),
            ],
            remappings=[
                ("odometry/filtered", "odom/local"),
                ("/tf", "tf"),
                ("/tf_static", "tf_static"),
            ],
        )

        footprint_to_odom_ekf = Node(
            package="robot_localization",
            executable="ekf_node",
            name="footprint_to_odom_ekf",
            output="screen",
            parameters=[
                {"use_sim_time": use_sim_time},
                {"base_link_frame": "base_footprint"},
                {"odom_frame": "odom"},
                {"world_frame": "odom"},
                {"publish_tf": True},
                {"frequency": 50.0},
                {"two_d_mode": True},
                {"odom0": "odom/raw"},
                {
                    "odom0_config": [
                        False,
                        False,
                        False,
                        False,
                        False,
                        False,
                        True,
                        True,
                        False,
                        False,
                        False,
                        True,
                        False,
                        False,
                        False,
                    ]
                },
                {"imu0": "imu/data"},
                {
                    "imu0_config": [
                        False,
                        False,
                        False,
                        False,
                        False,
                        True,
                        False,
                        False,
                        False,
                        False,
                        False,
                        True,
                        False,
                        False,
                        False,
                    ]
                },
            ],
            remappings=[
                ("odometry/filtered", "odom"),
                ("/tf", "tf"),
                ("/tf_static", "tf_static"),
            ],
        )

        # Static TF connections (kept per-robot; TF topics are remapped to <ns>/tf*)
        map_to_odom_tf_node = Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="map_to_odom_tf_node",
            output="screen",
            parameters=[{"use_sim_time": use_sim_time}],
            arguments=[
                "--x",
                "0",
                "--y",
                "0",
                "--z",
                "0",
                "--roll",
                "0",
                "--pitch",
                "0",
                "--yaw",
                "0",
                "--frame-id",
                "map",
                "--child-frame-id",
                "odom",
            ],
            remappings=[("/tf", "tf"), ("/tf_static", "tf_static")],
        )

        base_footprint_to_base_link_tf_node = Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="base_footprint_to_base_link_tf_node",
            output="screen",
            parameters=[{"use_sim_time": use_sim_time}],
            arguments=[
                "--x",
                "0",
                "--y",
                "0",
                "--z",
                "0",
                "--roll",
                "0",
                "--pitch",
                "0",
                "--yaw",
                "0",
                "--frame-id",
                "base_footprint",
                "--child-frame-id",
                "base_link",
            ],
            remappings=[("/tf", "tf"), ("/tf_static", "tf_static")],
        )

        # Controller bootstrapping per robot
        #
        # In Jazzy, `spawner` expects <controller_name>.type to already exist as a parameter
        # on the target controller_manager. If the controller manager did not load the
        # controller YAML at startup, the spawner fails with:
        #   The 'type' param was not defined for '<controller>'
        #
        # To make the multi-robot launch robust, we explicitly set these parameters once the
        # controller manager service is available, then we spawn the controllers.
        set_controller_manager_params = TimerAction(
            period=18.0 + float(i) * 2.0,
            actions=[
                ExecuteProcess(
                    cmd=[
                        "bash",
                        "-lc",
                        (
                            "set -e; "
                            f"echo '[{ns}] setting controller types on /{ns}/controller_manager ...'; "
                            "for k in $(seq 1 200); do "
                            f"  ros2 service list 2>/dev/null | grep -q \"/{ns}/controller_manager/list_controllers\" && break; "
                            "  sleep 0.1; "
                            "done; "
                            f"ros2 param set /{ns}/controller_manager joint_states_controller.type joint_state_broadcaster/JointStateBroadcaster; "
                            f"ros2 param set /{ns}/controller_manager joint_group_effort_controller.type joint_trajectory_controller/JointTrajectoryController; "),
                    ],
                    output="screen",
                )
            ],
        )

        # Controller spawners per robot (use explicit controller manager namespace)
        controller_spawner_js = TimerAction(
            period=22.0 + float(i) * 2.0,
            actions=[
                Node(
                    package="controller_manager",
                    executable="spawner",
                    name="spawner_joint_states_controller",
                    output="screen",
                    arguments=[
                        "--controller-manager-timeout",
                        "120",
                        "--controller-manager",
                        f"/{ns}/controller_manager",
                        "--param-file",
                        ros_control_yaml,
                        "joint_states_controller",
                    ],
                    parameters=[{"use_sim_time": use_sim_time}],
                )
            ],
        )

        controller_spawner_effort = TimerAction(
            period=32.0 + float(i) * 2.0,
            actions=[
                Node(
                    package="controller_manager",
                    executable="spawner",
                    name="spawner_joint_group_effort_controller",
                    output="screen",
                    arguments=[
                        "--controller-manager-timeout",
                        "120",
                        "--controller-manager",
                        f"/{ns}/controller_manager",
                        "--param-file",
                        ros_control_yaml,
                        "joint_group_effort_controller",
                    ],
                    parameters=[{"use_sim_time": use_sim_time}],
                )
            ],
        )

        controller_status_check = TimerAction(
            period=25.0 + float(i) * 2.0,
            actions=[
                ExecuteProcess(
                    cmd=[
                        "bash",
                        "-c",
                        f"echo '[{ns}] controller status:' && ros2 control list_controllers -c /{ns}/controller_manager",
                    ],
                    output="screen",
                )
            ],
        )

        robot_group = GroupAction(
            actions=[
                PushRosNamespace(ns),
                robot_state_publisher_node,
                gazebo_spawn_robot,
                gazebo_bridge,
                quadruped_controller_node,
                state_estimator_node,
                base_to_footprint_ekf,
                footprint_to_odom_ekf,
                map_to_odom_tf_node,
                base_footprint_to_base_link_tf_node,
                set_controller_manager_params,
                controller_spawner_js,
                controller_spawner_effort,
                controller_status_check,
            ]
        )

        actions.append(robot_group)

    return actions


def generate_launch_description() -> LaunchDescription:
    unitree_go2_description_share = launch_ros.substitutions.FindPackageShare(
        package="unitree_go2_description"
    ).find("unitree_go2_description")
    unitree_go2_sim_share = launch_ros.substitutions.FindPackageShare(
        package="unitree_go2_sim"
    ).find("unitree_go2_sim")

    default_model_path = os.path.join(
        unitree_go2_description_share, "urdf", "unitree_go2_robot.xacro"
    )
    default_world_path = os.path.join(unitree_go2_description_share, "worlds", "default.sdf")

    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Use simulation (Gazebo) clock if true",
    )

    declare_num_robots = DeclareLaunchArgument(
        "num_robots",
        default_value="1",
        description="Number of Go2 robots to spawn",
    )

    declare_namespace_prefix = DeclareLaunchArgument(
        "namespace_prefix",
        default_value="go2",
        description="Namespace prefix. Robots will use <namespace_prefix>_<i>",
    )

    declare_robot_name_prefix = DeclareLaunchArgument(
        "robot_name_prefix",
        default_value="go2",
        description="Gazebo model name prefix. Robots will be named <robot_name_prefix>_<i>",
    )

    declare_x_spacing = DeclareLaunchArgument(
        "x_spacing",
        default_value="2.0",
        description="Spacing between robots along X in meters",
    )

    declare_y_spacing = DeclareLaunchArgument(
        "y_spacing",
        default_value="0.0",
        description="Spacing between rows along Y in meters",
    )

    declare_robots_per_row = DeclareLaunchArgument(
        "robots_per_row",
        default_value="0",
        description="Robots per row in the spawn grid. 0 means a single row.",
    )

    declare_world_init_x = DeclareLaunchArgument("world_init_x", default_value="0.0")
    declare_world_init_y = DeclareLaunchArgument("world_init_y", default_value="0.0")
    declare_world_init_z = DeclareLaunchArgument("world_init_z", default_value="0.375")
    declare_world_init_heading = DeclareLaunchArgument(
        "world_init_heading", default_value="0.0"
    )

    declare_world = DeclareLaunchArgument(
        "world",
        default_value=default_world_path,
        description="Gazebo world SDF path",
    )

    declare_description_path = DeclareLaunchArgument(
        "unitree_go2_description_path",
        default_value=default_model_path,
        description="Path to the robot description xacro file",
    )

    # RViz is disabled by default (user request). If enabled, it visualizes a selected robot namespace.
    declare_rviz = DeclareLaunchArgument(
        "rviz",
        default_value="false",
        description="Launch RViz",
    )

    declare_rviz_namespace = DeclareLaunchArgument(
        "rviz_namespace",
        default_value="go2_0",
        description="Which robot namespace to visualize when rviz:=true (example: go2_0)",
    )

    # Gazebo Sim
    pkg_ros_gz_sim = get_package_share_directory("ros_gz_sim")
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_ros_gz_sim, "launch", "gz_sim.launch.py")),
        launch_arguments={
            "gz_args": [LaunchConfiguration("world"), " -r"],
        }.items(),
    )

    # Single global /clock bridge
    clock_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="clock_bridge",
        output="screen",
        parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
        ],
    )

    # Optional RViz
    rviz2 = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", os.path.join(unitree_go2_sim_share, "rviz", "rviz.rviz")],
        condition=IfCondition(LaunchConfiguration("rviz")),
        remappings=[
            ("/tf", ["/", LaunchConfiguration("rviz_namespace"), "/tf"]),
            ("/tf_static", ["/", LaunchConfiguration("rviz_namespace"), "/tf_static"]),
        ],
    )

    return LaunchDescription(
        [
            declare_use_sim_time,
            declare_num_robots,
            declare_namespace_prefix,
            declare_robot_name_prefix,
            declare_x_spacing,
            declare_y_spacing,
            declare_robots_per_row,
            declare_world_init_x,
            declare_world_init_y,
            declare_world_init_z,
            declare_world_init_heading,
            declare_world,
            declare_description_path,
            declare_rviz,
            declare_rviz_namespace,
            gz_sim,
            clock_bridge,
            OpaqueFunction(function=_spawn_robots),
            rviz2,
        ]
    )
