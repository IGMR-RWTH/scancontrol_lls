"""Launch the scanCONTROL LLS driver as a composable node."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    default_params = PathJoinSubstitution(
        [FindPackageShare("scancontrol_lls_driver"), "config", "lls_driver.params.yaml"]
    )
    params_file = LaunchConfiguration("params_file")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "params_file",
                default_value=default_params,
                description="Path to the ROS 2 parameter file for the lls_driver component.",
            ),
            ComposableNodeContainer(
                name="lls_driver_container",
                namespace="",
                package="rclcpp_components",
                executable="component_container_mt",
                output="screen",
                composable_node_descriptions=[
                    ComposableNode(
                        package="scancontrol_lls_driver",
                        plugin="scancontrol_lls::LaserLineScanner",
                        name="lls_driver",
                        parameters=[params_file],
                    )
                ],
            ),
        ]
    )
