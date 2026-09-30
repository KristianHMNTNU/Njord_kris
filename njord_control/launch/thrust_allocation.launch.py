import launch
import launch_ros.actions

VESSEL_NAME = "munin"


def generate_launch_description():

    thrust_allocation = launch_ros.actions.Node(
        namespace=VESSEL_NAME,
        package="njord_control",
        executable="thrust_allocation.py",
        name="thrust_allocation",
        output="screen",
        parameters=[{"max_thrust": 1.0}],
    )

    return launch.LaunchDescription([
        thrust_allocation,
    ])
