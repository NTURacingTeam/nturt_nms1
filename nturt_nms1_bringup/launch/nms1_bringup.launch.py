from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution, PythonExpression

from launch_ros.actions import Node, SetRemap
from launch_ros.descriptions import ParameterFile
from launch_ros.substitutions import FindPackageShare
from nav2_common.launch import ReplaceString, RewrittenYaml


def generate_launch_description():
    # declare arguments
    arguments = []
    arguments.append(
        DeclareLaunchArgument(
            "nms1_ros_ns",
            default_value="nms1",
            description="The namespace of nodes/topics of the nms1.",
        )
    )
    arguments.append(
        DeclareLaunchArgument(
            "tf_ns",
            default_value="",
            description="The namespace of frames of nms1.",
        )
    )
    arguments.append(
        DeclareLaunchArgument(
            "using_fake_hardware",
            default_value="true",
            description="Wether to use fake hardware.",
        )
    )

    # initialize arguments
    ros_ns = LaunchConfiguration("nms1_ros_ns")
    tf_ns = LaunchConfiguration("tf_ns")
    using_fake_hardware =  LaunchConfiguration("using_fake_hardware")

    # variables
    tf_prefix = PythonExpression(["'", tf_ns, "' + '/' if '", tf_ns, "' != '' else ''"])
    robot_description_file = PathJoinSubstitution([
        FindPackageShare("nturt_nms1_description"),
        "urdf",
        "nms1.xacro",
    ])
    controllers_file = PathJoinSubstitution([
        FindPackageShare("nturt_nms1_bringup"),
        "config",
        "nms1_controllers.yaml"
    ])
    # convert xacro into urdf
    robot_description = Command((
        "xacro",
        " ",
        robot_description_file,
        " ",
        "namespace:=",
        tf_ns,
        " ",
        "using_fake_hardware:=",
        using_fake_hardware,
    ))
    # modify controllers file according to arguments
    namespaced_controllers_file = ParameterFile(
        RewrittenYaml(
            source_file=controllers_file,
            root_key=ros_ns,
            param_rewrites={},
            convert_types=True
        ),
        allow_substs=True,
    )
    # modify rviz config file according to arguments
    fixed_controllers_file = ReplaceString(
        source_file=namespaced_controllers_file,
        replacements={
            "<tf_prefix>": tf_prefix,
        },
    )

    # declare nodes
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        namespace=ros_ns,
        output="both",
        parameters=[
            {"robot_description": robot_description}
        ],
    )
    controller_manager_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        namespace=ros_ns,
        output="both",
        parameters=[
            fixed_controllers_file,
            {"robot_description": robot_description},
        ],
    )
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        namespace=ros_ns,
        output="both",
        arguments=[
            "joint_state_broadcaster",
            "--namespace", ros_ns,
            "--controller-manager", "controller_manager",
        ],
    )
    ackermann_steering_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        namespace=ros_ns,
        output="both",
        arguments=[
            "ackermann_steering_controller",
            "--namespace", ros_ns,
            "--controller-manager", "controller_manager",
        ],
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="both",
        arguments=["-d", rviz_config_file],
    )
    delay_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[
                ackermann_steering_controller_spawner,
                rviz_node,
            ],
        )
    )

    nodes = [
        robot_state_publisher_node,
        controller_manager_node,
        joint_state_broadcaster_spawner,
        delay_after_joint_state_broadcaster_spawner,
    ]

    return LaunchDescription(arguments + nodes)
