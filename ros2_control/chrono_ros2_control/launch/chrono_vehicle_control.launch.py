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

Run:
    cd ros2_control && colcon build --packages-select chrono_ros2_control  # step 5
    source /opt/ros/humble/setup.bash && source install/setup.bash
    ros2 launch chrono_ros2_control/launch/chrono_vehicle_control.launch.py
"""
import os

from launch import LaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

HERE = os.path.dirname(os.path.abspath(__file__))
URDF_PATH = os.path.join(HERE, "..", "..", "urdf", "chrono_vehicle.urdf")
CONTROLLERS_YAML = os.path.join(HERE, "..", "config", "chrono_vehicle_controllers.yaml")


def generate_launch_description():
    robot_description = {
        "robot_description": ParameterValue(Command(["cat ", URDF_PATH]), value_type=str)
    }

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, CONTROLLERS_YAML],
        output="both",
        remappings=[("~/robot_description", "/robot_description")],
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

    return LaunchDescription([
        control_node,
        robot_state_pub_node,
        joint_state_broadcaster_spawner,
        delay_ackermann_after_joint_state_broadcaster,
    ])
