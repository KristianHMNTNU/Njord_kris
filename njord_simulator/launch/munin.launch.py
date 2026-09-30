import launch
import launch.actions
import launch.conditions
import launch.substitutions
import launch_ros.actions
import launch_ros.parameter_descriptions
import launch_ros.substitutions

VESSEL_NAME = "munin"


def generate_launch_description():

    pkg_share = launch_ros.substitutions.FindPackageShare("njord_simulator")

    use_gui = launch.actions.DeclareLaunchArgument(
        "use_gui", default_value="true", description="Launch RViz"
    )

    # The simulator publishes /clock itself, so it must run on wall time.
    sim_node = launch_ros.actions.Node(
        namespace=VESSEL_NAME,
        package="njord_simulator",
        executable="munin.py",
        name="sim",
        output="screen",
        parameters=[
            launch.substitutions.PathJoinSubstitution([pkg_share, "config", "simulation.yaml"]),
            {"tf_prefix": VESSEL_NAME},
            {"use_sim_time": False},
        ],
    )

    xacro_file = launch.substitutions.PathJoinSubstitution(
        [pkg_share, "urdf", "munin.urdf.xacro"]
    )

    robot_state_publisher = launch_ros.actions.Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        namespace=VESSEL_NAME,
        output="screen",
        parameters=[
            {"use_sim_time": True},
            {"robot_description": launch_ros.parameter_descriptions.ParameterValue(
                launch.substitutions.Command(["xacro ", xacro_file]), value_type=str)},
            {"frame_prefix": f"{VESSEL_NAME}/"},
        ],
    )

    rviz = launch_ros.actions.Node(
        package="rviz2",
        executable="rviz2",
        namespace=VESSEL_NAME,
        output="screen",
        arguments=["-d", launch.substitutions.PathJoinSubstitution([pkg_share, "rviz", "munin.rviz"])],
        parameters=[{"use_sim_time": True}],
        condition=launch.conditions.IfCondition(launch.substitutions.LaunchConfiguration("use_gui")),
    )

    return launch.LaunchDescription([
        use_gui,
        sim_node,
        robot_state_publisher,
        rviz,
    ])
