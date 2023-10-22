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
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "arm_id",
            default_value="za",
            description="Name (prefix) of the robot to launch",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "controller",
            default_value="joint_trajectory_controller",
            description="Which controller should be started?",
        )
    )

    arm_id = LaunchConfiguration("arm_id")
    controller = LaunchConfiguration("controller")

    robot_description = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("za_description"), "urdf", "za.xacro"]
            ),
        ]
    )

    robot_controllers = PathJoinSubstitution(
        [FindPackageShare("za_robot"), "config", "ros_controllers.yaml"]
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            {"robot_description": ParameterValue(robot_description, value_type=str)},
            robot_controllers,
        ],
        output="both",
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
    )

    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[controller, "--controller-manager", "/controller_manager"],
    )

    nodes = [
        control_node,
        joint_state_broadcaster_spawner,
        robot_controller_spawner,
    ]

    return LaunchDescription(declared_arguments + nodes)