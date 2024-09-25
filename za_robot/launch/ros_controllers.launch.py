from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    Command,
    FindExecutable,
)

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    robot_controllers = PathJoinSubstitution(
        [FindPackageShare("za_robot"), "config", "ros_controllers.yaml"]
    )

    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "prefix",
            default_value="",
            description="The prefix appended to URDF",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "controller",
            default_value="joint_trajectory_controller",
            description="Which controller should be started?",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_mock_hardware",
            default_value="true",
            description="Should mock (simulated) hardware be used?",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "controller_config",
            default_value=robot_controllers,
            description="Path to the configuration file for ros2_control"
        )
    )
    prefix = LaunchConfiguration("prefix")
    controller = LaunchConfiguration("controller")
    use_mock_hardware = LaunchConfiguration("use_mock_hardware")
    controller_config = LaunchConfiguration("controller_config")

    robot_description = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("za_description"), "urdf", "za.xacro"]
            ),
            " use_mock_hardware:=", use_mock_hardware,
            " prefix:=", prefix,
        ]
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            {"robot_description": ParameterValue(robot_description, value_type=str)},
            controller_config,
        ],
        output="both",
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster", 
            "--controller-manager", "controller_manager",
        ],
    )

    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[controller, "--controller-manager", "controller_manager"],
    )

    nodes = [
        control_node,
        joint_state_broadcaster_spawner,
        robot_controller_spawner,
    ]

    return LaunchDescription(declared_arguments + nodes)