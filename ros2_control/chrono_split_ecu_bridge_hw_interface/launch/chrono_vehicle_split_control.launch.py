"""
Split-version (스플릿 버전) counterpart of
chrono_ros2_control/launch/chrono_vehicle_control.launch.py (the
in-process/인프로세스 버전's launch file) -- that file is untouched. Brings
up all 5 processes of the split architecture together:

    controller_manager (ChronoEcuBridgeSystemInterface, Bridge 1 side)
      <-> chrono_split_ecu (P-loop/deadband/unit conversion)
      <-> chrono_split_dynamics_node (fmu_client, Bridge 2 side)

plus robot_state_publisher and the same joint_state_broadcaster/
ackermann_steering_controller spawners the in-process version uses (the
controller config YAML is shared unchanged -- joint names/interface types
are identical between chrono_vehicle.urdf and chrono_vehicle_split.urdf).

Run:
    cd ros2_control && colcon build
    source /opt/ros/humble/setup.bash && source install/setup.bash
    ros2 launch chrono_split_ecu_bridge_hw_interface/launch/chrono_vehicle_split_control.launch.py
"""
import os

from launch import LaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

HERE = os.path.dirname(os.path.abspath(__file__))
URDF_PATH = os.path.join(HERE, "..", "..", "urdf", "chrono_vehicle_split.urdf")
CONTROLLERS_YAML = os.path.join(
    HERE, "..", "..", "chrono_ros2_control", "config", "chrono_vehicle_controllers.yaml"
)
FMU_DIR = os.path.join(HERE, "..", "..", "..", "fmu", "cpp", "native_vehicle_fmu")


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

    ecu_node = Node(
        package="chrono_split_ecu",
        executable="chrono_split_ecu_node",
        output="both",
    )
    dynamics_node = Node(
        package="chrono_split_dynamics_node",
        executable="chrono_split_dynamics_node",
        output="both",
        parameters=[{"fmu_dir": FMU_DIR}],
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
        dynamics_node,
        ecu_node,
        control_node,
        robot_state_pub_node,
        joint_state_broadcaster_spawner,
        delay_ackermann_after_joint_state_broadcaster,
    ])
