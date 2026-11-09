"""Launch the scanCONTROL LLS driver node with a parameter file."""

from launch import LaunchDescription, Action, LaunchContext
from launch.actions import DeclareLaunchArgument, OpaqueFunction, EmitEvent, RegisterEventHandler
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import LifecycleNode
from launch_ros.substitutions import FindPackageShare
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from launch.event_handlers import OnProcessStart
from launch.events import matches_action
from lifecycle_msgs.msg import Transition


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

    driver = LifecycleNode(
        package = "scancontrol_lls_driver",
        executable = "lls_node",
        name = "lls_driver",
        namespace = '',
        output = "screen",
        parameters = [
            params_file, 
            {"fake_mode": fake_mode_bool}
        ],
    )

    configure = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher = matches_action(driver),
            transition_id = Transition.TRANSITION_CONFIGURE,
        )
    )
    activate = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher = matches_action(driver),
            transition_id = Transition.TRANSITION_ACTIVATE,
        )
    )
     
    launch_actions = [
        DeclareLaunchArgument(
            "params_file",
            default_value = default_params,
            description = "Path to the ROS 2 parameter file for the lls_driver node.",
        ),
        # Register handlers before starting the process.
        RegisterEventHandler(
            OnProcessStart(
                target_action=driver, 
                on_start=[configure]
            )
        ),
        RegisterEventHandler(
            OnStateTransition(
                target_lifecycle_node = driver,
                start_state = "configuring",
                goal_state = "inactive",
                entities = [activate],
            )
        ),
        driver,
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