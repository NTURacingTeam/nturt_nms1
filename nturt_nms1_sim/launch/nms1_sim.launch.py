from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile
from launch_ros.substitutions import FindPackageShare
from nav2_common.launch import ReplaceString, RewrittenYaml


def generate_launch_description():
    # declare arguments
    arguments = []
    arguments.append(
        DeclareLaunchArgument(
            "nms1_sim_ros_ns",
            default_value="nms1_sim",
            description="The namespace of nodes/topics of the simulation.",
        )
    )
    arguments.append(
        DeclareLaunchArgument(
            "nms1_sim_params_file",
            default_value=PathJoinSubstitution([
                    FindPackageShare("nturt_nms1_sim"),
                    "config",
                    "nms1_sim.yaml"
                ]),
            description="The parameter file for all nodes in the simulation.",
        )
    )
    arguments.append(
        DeclareLaunchArgument(
            "tf_ns",
            default_value="",
            description="The namespace of frames of the simulated vehicle.",
        )
    )
    arguments.append(
        DeclareLaunchArgument(
            "map_frame",
            default_value="map",
            description="The name of the map frame, not effected by \"tf_ns\".",
        )
    )
    arguments.append(
        DeclareLaunchArgument(
            "map",
            default_value=PathJoinSubstitution([
                    FindPackageShare("nturt_nms1_sim"),
                    "maps",
                    "spielberg"
                ]),
            description="The map to use in the simulation, without file extension.",
        )
    )

    # initialize arguments
    ros_ns = LaunchConfiguration("nms1_sim_ros_ns")
    params_file = LaunchConfiguration("nms1_sim_params_file")
    tf_ns = LaunchConfiguration("tf_ns")
    map_frame = LaunchConfiguration("map_frame")
    map = LaunchConfiguration("map")

    # variables
    robot_description_file = PathJoinSubstitution([
        FindPackageShare("nturt_nms1_description"),
        "urdf",
        "nms1.xacro",
    ])
    rviz_config_file = PathJoinSubstitution([
        FindPackageShare("nturt_nms1_sim"),
        "config",
        "nms1_sim.rviz",
    ])
    # convert xacro into urdf
    robot_description = Command((
        "xacro",
        " ",
        robot_description_file,
        " ",
        "namespace:=",
        tf_ns,
    ))
    # modify parameter file according to arguments
    fixed_params_file = ParameterFile(
        RewrittenYaml(
            source_file=params_file,
            root_key=ros_ns,
            param_rewrites={
                # f1tenth_gym_ros_bridge
                "tf_ns": tf_ns,
                "map_frame": map_frame,
                "map": (map, ".png"),

                # map_server
                "yaml_filename": (map, ".yaml"),
                "frame_id": map_frame,
            },
            convert_types=True
        ),
        allow_substs=True,
    )
    # modify rviz config file according to arguments
    fixed_rviz_config_file = ReplaceString(
        source_file=rviz_config_file,
        replacements={
            "<fixed_frame>": map_frame,
        },
    )

    # declare nodes
    f1tenth_gym_ros_node = Node(
        package="nturt_nms1_sim",
        executable="f1tenth_gym_ros_bridge",
        namespace=ros_ns,
        output="both",
        parameters=[
            fixed_params_file,
        ],
    )
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        namespace=ros_ns,
        output="both",
        parameters=[
            {"robot_description": robot_description},
        ],
    )
    map_server_node = Node(
        package="nav2_map_server",
        executable="map_server",
        namespace=ros_ns,
        output="both",
        parameters=[
            fixed_params_file,
        ],
    )
    nav_lifecycle_node = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        namespace=ros_ns,
        output="both",
        parameters=[
            fixed_params_file,
        ],
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        namespace=ros_ns,
        output="both",
        arguments=["-d", fixed_rviz_config_file],
    )

    nodes = [
        f1tenth_gym_ros_node,
        robot_state_publisher_node,
        rviz_node,
        map_server_node,
        nav_lifecycle_node,
    ]

    return LaunchDescription(arguments + nodes)
