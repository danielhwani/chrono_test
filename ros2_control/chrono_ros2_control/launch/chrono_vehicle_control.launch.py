"""
Step 6 of the ros2_control prep plan: bring up the real controller_manager
(ros2_control_node) with ChronoFmuSystemInterface (step 4/5) +
ackermann_steering_controller + joint_state_broadcaster, driving the
vehicle from real controller commands -- unlike display.launch.py (step 3's
side task), which only ever used joint_state_publisher_gui sliders and
never touched controller_manager or the FMU at all.

Structure checked against the real, official ros2_control_demos
example_2's diffbot.launch.py (github.com/ros-controls/ros2_control_demos,
humble branch) rather than guessed: controller_manager's own
"~/robot_description" topic is remapped to "/robot_description" instead of
passing the URDF as a parameter to both nodes separately -- robot_state_publisher
is the one node that actually turns the URDF parameter into that topic. The
controller spawners run through controller_manager's `spawner` executable,
and the second spawner is deliberately delayed until after the first
(joint_state_broadcaster) exits successfully, same event-handler pattern as
the real example.

URDF loaded the same way display.launch.py already does (Command(["cat ",
path]) + ParameterValue, not a raw CLI -p argument -- see that file's own
comment for why the raw-argument approach breaks on multi-line URDF text).
No RViz here on purpose -- this launch file's job is controller_manager
+ controllers only; run display.launch.py separately if visualization is
wanted (don't run both robot_state_publisher instances against the same
URDF at once, though -- pick one).

`six_wheel` launch argument (default "false", same session as the 6x6
upgrade): when "true", loads chrono_vehicle_6x6.urdf instead of
chrono_vehicle.urdf -- that URDF's own <param name="six_wheel">true</param>
is what actually drives ChronoFmuSystemInterface's six_wheel behavior; this
argument only picks which URDF file to load. It ALSO switches the
controller_manager YAML to chrono_vehicle_controllers_6x6.yaml, which
differs from the 4-wheel YAML only in `wheelbase` (3.4 vs 2.6) --
ackermann_steering_controller uses that value directly in its Ackermann
steering-angle/odometry math, so reusing the 4-wheel YAML against the 6x6
URDF's real 3.4m wheelbase would silently command wrong steering angles.
Uses an OpaqueFunction because both paths depend on a launch argument's
runtime value, which plain module-level Python can't see.

Run:
    cd ros2_control && colcon build --packages-select chrono_ros2_control  # step 5
    source /opt/ros/humble/setup.bash && source install/setup.bash
    ros2 launch chrono_ros2_control/launch/chrono_vehicle_control.launch.py
    ros2 launch chrono_ros2_control/launch/chrono_vehicle_control.launch.py six_wheel:=true
"""
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

HERE = os.path.dirname(os.path.abspath(__file__))
CONFIG_DIR = os.path.join(HERE, "..", "config")


def launch_setup(context, *args, **kwargs):
    six_wheel = LaunchConfiguration("six_wheel").perform(context) == "true"
    urdf_name = "chrono_vehicle_6x6.urdf" if six_wheel else "chrono_vehicle.urdf"
    urdf_path = os.path.join(HERE, "..", "..", "urdf", urdf_name)
    robot_description = {
        "robot_description": ParameterValue(Command(["cat ", urdf_path]), value_type=str)
    }
    controllers_yaml_name = (
        "chrono_vehicle_controllers_6x6.yaml" if six_wheel else "chrono_vehicle_controllers.yaml"
    )
    controllers_yaml = os.path.join(CONFIG_DIR, controllers_yaml_name)

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, controllers_yaml],
        output="both",
        remappings=[
            ("~/robot_description", "/robot_description"),
            # ackermann_steering_controller publishes its odom->base_link
            # transform to its own private "tf_odometry" topic (confirmed
            # live: enable_odom_tf defaults to true, the transform itself
            # was correct, but RViz never saw the car move because it only
            # ever listens on /tf, not a controller-private topic) --
            # remap it onto /tf so RViz/tf2 actually pick it up. spawner has
            # no per-controller remap flag in Humble, so this has to live on
            # the node that actually hosts the controller.
            ("/ackermann_steering_controller/tf_odometry", "/tf"),
        ],
    )
    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )
    ackermann_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["ackermann_steering_controller", "--controller-manager", "/controller_manager"],
    )
    delay_ackermann_after_joint_state_broadcaster = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[ackermann_spawner],
        )
    )

    return [
        control_node,
        robot_state_pub_node,
        joint_state_broadcaster_spawner,
        delay_ackermann_after_joint_state_broadcaster,
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "six_wheel",
            default_value="false",
            description="Use chrono_vehicle_6x6.urdf instead of chrono_vehicle.urdf",
        ),
        OpaqueFunction(function=launch_setup),
    ])
