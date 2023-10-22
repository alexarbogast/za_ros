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
            "rviz",
            default_value="false",
            description="The configuration file to use for RViz",
        )
    )
    arm_id = LaunchConfiguration("arm_id")
    controller = LaunchConfiguration("controller")
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
        launch_arguments={"arm_id": arm_id, 
                          "controller": controller}.items(),

    )

    visualization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                        FindPackageShare("za_robot"),
                        "launch",
                        "za_visualization.launch.py",
                ]),
        ]),
        launch_arguments={"arm_id": arm_id}.items(),
        condition=IfCondition(rviz),
    )
    # fmt: on

    nodes_to_start = [
        hardware,
        visualization,
    ]

    return LaunchDescription(declared_arguments + nodes_to_start)
