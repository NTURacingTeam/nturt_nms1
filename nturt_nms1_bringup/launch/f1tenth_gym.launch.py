from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution, PythonExpression

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # declare arguments
    arguments = []
    arguments.append(
        DeclareLaunchArgument(
            "namespace",
            default_value="nms1_sim",
            description="Arguement to determine the namespace of the topic and the tramsform of the simulation.",
        )
    )
    arguments.append(
        DeclareLaunchArgument(
            "odom_topic",
            default_value="odom",
            description="Arguement to determine the name of the topic for odom data, without prifixing \"/\".",
        )
    )
    arguments.append(
        DeclareLaunchArgument(
            "scan_topic",
            default_value="scan",
            description="Arguement to determine the name of the topic for lidar scan data, without prifixing \"/\".",
        )
    )
    arguments.append(
        DeclareLaunchArgument(
            "map_frame",
            default_value="map",
            description="Arguement to determine the name of the map frame, without prifixing \"/\".",
        )
    )
    arguments.append(
        DeclareLaunchArgument(
            "map",
            default_value=PathJoinSubstitution([
                FindPackageShare("nturt_nms1_bringup"),
                "maps",
                "levine"
            ]),
            description="Arguement to determine what map to use in f1tenth_gym simulation, without file extension.",
        )
    )

    # initialize arguments
    namespace = LaunchConfiguration("namespace")
    odom_topic = LaunchConfiguration("odom_topic")
    scan_topic = LaunchConfiguration("scan_topic")
    map_frame = LaunchConfiguration("map_frame")
    map = LaunchConfiguration("map")

    # variables
    robot_description_package = FindPackageShare("nturt_nms1_description")
    robot_bringup_package = FindPackageShare("nturt_nms1_bringup")
    robot_description_file = PathJoinSubstitution([
        robot_description_package,
        "urdf",
        "nms1.xacro"
    ])
    rviz_config_file = PathJoinSubstitution([
        robot_bringup_package,
        "config",
        "f1tenth_gym.rviz"
    ])
    # convert xacro into urdf
    robot_description = Command([
        "xacro ",
        robot_description_file,
        " ns:=",
        namespace,
    ])
    # add the settings of namsespace and map_frame to rviz config
    fixed_rviz_config_file = Command([
        "ros2 run nturt_nms1_bringup modify_rviz_config.py ",
        rviz_config_file,
        " ",
        namespace,
        " ",
        scan_topic,
        " ",
        map_frame,
    ])

    # declare nodes
    f1tenth_gym_ros_node = Node(
        package="nturt_nms1_bringup",
        executable="f1tenth_gym_ros_bridge.py",
        namespace=namespace,
        output="both",
        parameters=[
            {"namespace": namespace},
            {"odom_topic": odom_topic},
            {"scan_topic": scan_topic},
            {"map_frame": map_frame},
            {"map": PythonExpression(["'", map, "'", " + '.png'"])},
        ],
    )
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        namespace=namespace,
        output="both",
        parameters=[
            {"robot_description": robot_description},
        ],
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        namespace=namespace,
        output="both",
        arguments=["-d", fixed_rviz_config_file],
    )
    map_server_node = Node(
        package="nav2_map_server",
        executable="map_server",
        namespace=namespace,
        output="both",
        parameters=[
            {"yaml_filename": PythonExpression(["'", map, "'", " + '.yaml'"])},
            {"topic": PythonExpression(["'", namespace, "'", " + '/map'"])},
            {"frame_id": PythonExpression(["'", namespace, "'", " + '/' + ", "'", map_frame, "'"])},
        ],
    )
    nav_lifecycle_node = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        namespace=namespace,
        output="both",
        parameters=[
            {"autostart": True},
            {"node_names": ["map_server"]},
        ],
    )

    nodes = [
        f1tenth_gym_ros_node,
        robot_state_publisher_node,
        rviz_node,
        map_server_node,
        nav_lifecycle_node,
    ]

    return LaunchDescription(arguments + nodes)
