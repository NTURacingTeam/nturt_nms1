from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # declare arguments
    arguments = []
    arguments.append(
        DeclareLaunchArgument(
            "using_gazebo",
            default_value="true",
            description="Arguement to determine wether to parse nms1 model with gazebo.",
        )
    )

    # initialize arguments
    using_gazebo = LaunchConfiguration("using_gazebo")

    # variables
    robot_description_package = FindPackageShare("nturt_nms1_description")
    robot_description_file = PathJoinSubstitution([
        robot_description_package,
        "urdf",
        "nms1.xacro"
    ])
    # convert xacro into urdf
    robot_description = Command([
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            robot_description_file,
            " ",
            "using_gazebo:=",
            using_gazebo
    ])

    # declare include files
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare("gazebo_ros"),
                "launch",
                "gazebo.launch.py"
            ])
        ]),
        launch_arguments={"verbose": "true"}.items(),
    )

    includes = [
        gazebo,
    ]

    # declare nodes
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[
            {"robot_description": robot_description}
        ],
    )
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
    )
    spawn_entity_node = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        output="screen",
        arguments=["-topic", "robot_description", "-entity", "nms1"],
    )

    nodes = [
        robot_state_publisher_node,
        joint_state_publisher_node,
        spawn_entity_node,
    ]

    return LaunchDescription(arguments + includes + nodes)
