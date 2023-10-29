from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
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
                    FindPackageShare("nturt_nms1_nav"),
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
    arguments.append(
        DeclareLaunchArgument(
            "use_simulator",
            default_value="true",
            description="Run in simulation.",
        )
    )

    # initialize arguments
    ros_ns = LaunchConfiguration("nms1_nav_ros_ns")
    params_file = LaunchConfiguration("nms1_nav_params_file")
    tf_ns = LaunchConfiguration("tf_ns")
    use_simulator = LaunchConfiguration("use_simulator")
    # declared in nms1_sim launch file
    sim_ros_ns = LaunchConfiguration("nms1_sim_ros_ns")

    # variables
    tf_prefix = PythonExpression(["'", tf_ns, "' + '/' if '", tf_ns, "' != '' else ''"])
    rviz_config_file = PathJoinSubstitution([
        FindPackageShare("nturt_nms1_nav"),
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
                # nav2
                "odom_topic": ("/", sim_ros_ns, "/odom"),
                # "global_frame": (tf_prefix, "odom"),
                "robot_base_frame": (tf_prefix, "base_link"),
            },
            convert_types=True
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
        condition=IfCondition(use_simulator),
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
        ],
    )
    nav2_behavior_server_node = Node(
        package="nav2_behaviors",
        executable="behavior_server",
        namespace=ros_ns,
        output="both",
        parameters=[
            fixed_params_file,
        ],
    )
    nav2_bt_navigator_node = Node(
        package="nav2_bt_navigator",
        executable="bt_navigator",
        namespace=ros_ns,
        output="both",
        parameters=[
            fixed_params_file,
        ],
    )
    nav2_planner_server_node = Node(
        package="nav2_planner",
        executable="planner_server",
        namespace=ros_ns,
        output="both",
        parameters=[
            fixed_params_file,
        ],
    )
    nav2_smoother_server_node = Node(
        package="nav2_smoother",
        executable="smoother_server",
        namespace=ros_ns,
        output="both",
        parameters=[
            fixed_params_file,
        ],
    )
    nav2_controller_server_node = Node(
        package="nav2_controller",
        executable="controller_server",
        namespace=ros_ns,
        output="both",
        parameters=[
            fixed_params_file,
        ],
        remappings=[('cmd_vel', '/cmd_vel')],
    )
    nav2_waypoint_follower_node = Node(
        package="nav2_waypoint_follower",
        executable="waypoint_follower",
        namespace=ros_ns,
        output="both",
        parameters=[
            fixed_params_file,
        ],
    )
    nav2_lifecycle_node = Node(
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
        slam_toolbox_node,
        nav2_behavior_server_node,
        nav2_bt_navigator_node,
        nav2_planner_server_node,
        nav2_smoother_server_node,
        nav2_controller_server_node,
        nav2_waypoint_follower_node,
        nav2_lifecycle_node,
        rviz_node,
    ]


    return LaunchDescription(arguments + includes + nodes)
