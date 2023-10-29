from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution, PythonExpression

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from nav2_common.launch import ReplaceString


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
            "use_rviz",
            default_value="false",
            description="Wether to visualize in rviz2.",
        )
    )
    arguments.append(
        DeclareLaunchArgument(
            "fake_hardware",
            default_value="true",
            description="Wether to use fake hardware.",
        )
    )

    # initialize arguments
    ros_ns = LaunchConfiguration("nms1_ros_ns")
    tf_ns = LaunchConfiguration("tf_ns")
    use_rviz = LaunchConfiguration("use_rviz")
    fake_hardware =  LaunchConfiguration("fake_hardware")

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
    rviz_config_file = PathJoinSubstitution([
        FindPackageShare("nturt_nms1_bringup"),
        "config",
        "nms1_bringup.rviz"
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
        "fake_hardware:=",
        fake_hardware,
    ))
    # modify controllers file according to arguments
    fixed_controllers_file = ReplaceString(
        source_file=controllers_file,
        replacements={
            "<namespace>": ros_ns,
            "<tf_prefix>": tf_prefix,
        },
    )
    # modify rviz config file according to arguments
    fixed_rviz_config_file = ReplaceString(
        source_file=rviz_config_file,
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
        namespace=ros_ns,
        output="both",
        arguments=["-d", fixed_rviz_config_file],
        condition=IfCondition(use_rviz),
    )
    remapper_node = Node(
        package="nturt_nms1_bringup",
        executable="nms1_bringup_remapper_node",
        namespace=ros_ns,
        output="both",
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
        remapper_node,
    ]

    return LaunchDescription(arguments + nodes)
