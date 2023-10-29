from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.substitutions import FindPackageShare


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
            "controller_config",
            default_value=robot_controllers,
            description="Path to the configuration file for ros2_control"
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
            description="The should RViz be launched",
        )
    )
    prefix = LaunchConfiguration("prefix")
    controller = LaunchConfiguration("controller")
    controller_config = LaunchConfiguration("controller_config")
    use_mock_hardware = LaunchConfiguration("use_mock_hardware")
    rviz = LaunchConfiguration("rviz")

    # fmt: off
    hardware = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                        FindPackageShare("za_robot"),
                        "launch/ros_controllers.launch.py",
                ]),
        ]), 
        launch_arguments={
            "prefix": prefix,
            "controller": controller,
            "controller_config": controller_config,
            "use_mock_hardware": use_mock_hardware,
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
        launch_arguments={"prefix": prefix}.items(),
        condition=IfCondition(rviz),
    )
    # fmt: on

    nodes_to_start = [
        hardware,
        visualization,
    ]

    return LaunchDescription(declared_arguments + nodes_to_start)
