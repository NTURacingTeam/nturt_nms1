from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression

from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile
from launch_ros.substitutions import FindPackageShare
from nav2_common.launch import ReplaceString, RewrittenYaml


def generate_launch_description():
    # declare arguments
    arguments = []
    arguments.append(
        DeclareLaunchArgument(
            "nms1_nav_ros_ns",
            default_value="nms1_nav",
            description="The namespace of the name of nodes/topics of the navigation system.",
        )
    )
    arguments.append(
        DeclareLaunchArgument(
            "nms1_nav_params_file",
            default_value=PathJoinSubstitution([
                    FindPackageShare("nturt_nms1_bringup"),
                    "config",
                    "nms1_nav.yaml"
                ]),
            description="The parameter file for all nodes in the navigation system.",
        )
    )
    arguments.append(
        DeclareLaunchArgument(
            "tf_ns",
            default_value="",
            description="The namespace of frames of the simulated vehicle.",
        )
    )

    # initialize arguments
    ros_ns = LaunchConfiguration("nms1_nav_ros_ns")
    params_file = LaunchConfiguration("nms1_nav_params_file")
    tf_ns = LaunchConfiguration("tf_ns")
    # declared in nms1_sim launch file
    sim_ros_ns = LaunchConfiguration("nms1_sim_ros_ns")

    # variables
    tf_prefix = PythonExpression(["'", tf_ns, "' + '/' if '", tf_ns, "' != '' else ''"])
    slam_params_file = PathJoinSubstitution([
        FindPackageShare("nturt_nms1_bringup"),
        "config",
        "slam.yaml"
    ])
    rviz_config_file = PathJoinSubstitution([
        FindPackageShare("nturt_nms1_bringup"),
        "config",
        "nms1_nav.rviz"
    ])
    # modify parameter file according to arguments
    fixed_params_file = ParameterFile(
        RewrittenYaml(
            source_file=params_file,
            root_key=ros_ns,
            param_rewrites={
                # slam_toolbox
                "scan_topic": ("/", sim_ros_ns, "/scan"),
                "odom_frame": (tf_prefix, "odom"),
                "base_frame": (tf_prefix, "base_link"),
            },
            convert_types=True
        ),
        allow_substs=True,
    )
    fixed_slam_params_file = ParameterFile(
        RewrittenYaml(
            source_file=slam_params_file,
            root_key=ros_ns,
            param_rewrites={
            },
            convert_types=True,
        ),
        allow_substs=True,
    )
    # modify rviz config file according to arguments
    fixed_rviz_config_file = ReplaceString(
        source_file=rviz_config_file,
        replacements={
            "<sim_ros_ns>": ("/", sim_ros_ns),
        },
    )

    # declare include files
    nms1_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare("nturt_nms1_sim"),
                "launch",
                "nms1_sim.launch.py",
            ]),
        ]),
        launch_arguments={
            "tf_ns": tf_ns,
            "map_frame": (tf_prefix, "odom"),
        }.items(),
    )

    includes = [
        nms1_sim,
    ]

    # declare nodes
    slam_toolbox_node = Node(
        package="slam_toolbox",
        executable="async_slam_toolbox_node",
        namespace=ros_ns,
        output="both",
        parameters=[
            fixed_params_file,
            fixed_slam_params_file,
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
        slam_toolbox_node,
        rviz_node,
    ]


    return LaunchDescription(arguments + includes + nodes)
