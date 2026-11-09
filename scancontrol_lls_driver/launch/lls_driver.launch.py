"""Launch the scanCONTROL LLS driver node with a parameter file."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):
    default_params = PathJoinSubstitution(
        [FindPackageShare("scancontrol_lls_driver"), "config", "lls_driver.params.yaml"]
    )
    params_file = LaunchConfiguration("params_file")

    fake_mode_str = LaunchConfiguration('fake_mode').perform(context)
    fake_mode_value = fake_mode_str.strip().lower()
    if fake_mode_value in {"true", "1", "yes", "on"}:
        fake_mode_bool = True
    elif fake_mode_value in {"false", "0", "no", "off"}:
        fake_mode_bool = False
    else:
        # Fail rather than silently connecting to a real sensor on a typo.
        raise ValueError(
            f"Invalid fake_mode value {fake_mode_str!r}; expected true or false"
        )
     
    launch_actions = [
        DeclareLaunchArgument(
            "params_file",
            default_value = default_params,
            description = "Path to the ROS 2 parameter file for the lls_driver node.",
        ),
        Node(
            package = "scancontrol_lls_driver",
            executable = "lls_node",
            name = "lls_driver",
            output = "screen",
            parameters = [
                params_file,
                {'fake_mode': fake_mode_bool}],
        ),
    ]

    return launch_actions


def generate_launch_description():

    arg_fake_mode = DeclareLaunchArgument(
        'fake_mode',
        default_value='false',
        description='Boolean [true/false] to toggle the fake mode of the driver node. If true, skips real sensor setup & connection and publishes a predefined fake scan profile.'
    )

    actions = [
        arg_fake_mode,
        OpaqueFunction(function = launch_setup)
    ]

    return LaunchDescription(actions)