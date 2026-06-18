"""Launch the scanCONTROL LLS driver node with a parameter file."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    default_params = PathJoinSubstitution(
        [FindPackageShare("scancontrol_lls_driver"), "config", "lls_driver.params.yaml"]
    )
    params_file = LaunchConfiguration("params_file")

    return LaunchDescription([
        DeclareLaunchArgument(
            "params_file",
            default_value=default_params,
            description="Path to the ROS 2 parameter file for the lls_driver node.",
        ),
        Node(
            package="scancontrol_lls_driver",
            executable="lls_node",
            name="lls_driver",
            output="screen",
            parameters=[params_file],
        ),
    ])
