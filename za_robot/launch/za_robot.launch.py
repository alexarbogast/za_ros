from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.substitutions import FindPackageShare


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
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_mock_hardware",
            default_value="true",
            description="Should mock (simulated) hardware be used?",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "rviz",
            default_value="false",
            description="The configuration file to use for RViz",
        )
    )
    controller = LaunchConfiguration("controller")
    use_mock_hardware = LaunchConfiguration("use_mock_hardware")
    rviz = LaunchConfiguration("rviz")

    # fmt: off
    hardware = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                        FindPackageShare("za_robot"),
                        "launch",
                        "ros_controllers.launch.py",
                ]),
        ]), 
        launch_arguments={
            "use_mock_hardware": use_mock_hardware,
            "controller": controller,
        }.items()
    )

    visualization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                        FindPackageShare("za_robot"),
                        "launch",
                        "za_visualization.launch.py",
                ]),
        ]),
        condition=IfCondition(rviz),
    )
    # fmt: on

    nodes_to_start = [
        hardware,
        visualization,
    ]

    return LaunchDescription(declared_arguments + nodes_to_start)
