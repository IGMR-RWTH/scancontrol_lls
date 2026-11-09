"""Launch the scanCONTROL LLS driver as a composable node with a parameter file."""

from launch import LaunchDescription, Action, LaunchContext
from launch.actions import DeclareLaunchArgument, OpaqueFunction, ExecuteProcess, RegisterEventHandler
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import ComposableNodeContainer, LoadComposableNodes
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare
from launch.event_handlers import OnProcessExit, OnExecutionComplete
from launch.events.process import ProcessExited


def launch_setup(context: LaunchContext, *args, **kwargs):

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

    container = ComposableNodeContainer(
        name = "lls_driver_container",
        namespace = "",
        package = "rclcpp_components",
        executable = "component_container_mt",
        output = "screen",
        # Load separately so launch can wait for completion.
        composable_node_descriptions=[],
    )

    load_driver = LoadComposableNodes(
        target_container=container,
        composable_node_descriptions=[
            ComposableNode(
                package="scancontrol_lls_driver",
                plugin="scancontrol_lls::LaserLineScanner",
                name="lls_driver",
                parameters=[params_file, {"fake_mode": fake_mode_bool}],
            ),
        ],
    )

    configure = ExecuteProcess(
        cmd=["ros2", "lifecycle", "set", "/lls_driver", "configure"],
        output="screen",
    )
    activate = ExecuteProcess(
        cmd=["ros2", "lifecycle", "set", "/lls_driver", "activate"],
        output="screen",
    )

    def on_configure_exit(event: ProcessExited, context: LaunchContext):
        # Do not attempt activation if the configure command failed.
        return [activate] if event.returncode == 0 else []

    
    launch_actions = [

        DeclareLaunchArgument(
            "params_file",
            default_value = default_params,
            description = "Path to the ROS 2 parameter file for the lls_driver component.",
        ),
        container,
        RegisterEventHandler(OnExecutionComplete(target_action=load_driver, on_completion=[configure])),
        RegisterEventHandler(OnProcessExit(target_action=configure, on_exit=on_configure_exit)),
        load_driver,
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