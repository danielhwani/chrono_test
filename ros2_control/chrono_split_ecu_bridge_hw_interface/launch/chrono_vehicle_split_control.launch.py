"""
Split-version (스플릿 버전) counterpart of
chrono_ros2_control/launch/chrono_vehicle_control.launch.py (the
in-process/인프로세스 버전's launch file) -- that file is untouched. Brings
up all 5 processes of the split architecture together:

    controller_manager (ChronoEcuBridgeSystemInterface, Bridge 1 side)
      <-> chrono_split_ecu (P-loop/deadband/unit conversion)
      <-> chrono_fmu_dynamics_node (fmu_client, Bridge 2 side)

plus robot_state_publisher and the same joint_state_broadcaster/
ackermann_steering_controller spawners the in-process version uses.

`six_wheel` launch argument (default "false", same session as the 6x6
upgrade, mirrors the in-process launch file's own argument of the same
name): when "true",
  - loads chrono_vehicle_split_6x6.urdf instead of chrono_vehicle_split.urdf
    (no <param name="six_wheel"> in that URDF -- ChronoEcuBridgeSystemInterface
    has zero FMU-specific knowledge, unlike ChronoFmuSystemInterface)
  - switches to chrono_vehicle_controllers_6x6.yaml (wheelbase: 3.4 vs 2.6 --
    see that file's own comment for why the in-process controller YAML
    can't just be reused as-is)
  - passes six_wheel:=true as a ROS2 node parameter to BOTH chrono_split_ecu
    (drives its divide-by-3-axles torque fan-out) and
    chrono_fmu_dynamics_node (drives the FMU's own structural six_wheel
    flag via fmu_client_open_begin()/_finish()) -- unlike the in-process
    version, which only needs the one URDF <param>, this track has no
    single file both processes share, so the same launch-time value has to
    reach two independent node parameter sets.
Uses an OpaqueFunction for the same reason as the in-process launch file:
these paths/parameters depend on a launch argument's runtime value, which
plain module-level Python can't see.

Run:
    cd ros2_control && colcon build
    source /opt/ros/humble/setup.bash && source install/setup.bash
    ros2 launch chrono_split_ecu_bridge_hw_interface/launch/chrono_vehicle_split_control.launch.py
    ros2 launch chrono_split_ecu_bridge_hw_interface/launch/chrono_vehicle_split_control.launch.py six_wheel:=true
"""
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

HERE = os.path.dirname(os.path.abspath(__file__))
CONFIG_DIR = os.path.join(HERE, "..", "..", "chrono_ros2_control", "config")
FMU_DIR = os.path.join(HERE, "..", "..", "..", "fmu", "cpp", "native_vehicle_fmu")


def launch_setup(context, *args, **kwargs):
    six_wheel = LaunchConfiguration("six_wheel").perform(context) == "true"
    urdf_name = "chrono_vehicle_split_6x6.urdf" if six_wheel else "chrono_vehicle_split.urdf"
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
            # See chrono_ros2_control's matching launch file comment --
            # ackermann_steering_controller's odom->base_link transform goes
            # to a private "tf_odometry" topic by default, invisible to
            # RViz's own /tf subscription unless remapped.
            ("/ackermann_steering_controller/tf_odometry", "/tf"),
        ],
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
        parameters=[{"six_wheel": six_wheel}],
    )
    dynamics_node = Node(
        package="chrono_fmu_dynamics_node",
        executable="chrono_fmu_dynamics_node",
        output="both",
        parameters=[{"fmu_dir": FMU_DIR, "six_wheel": six_wheel}],
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
        dynamics_node,
        ecu_node,
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
            description="Use the 6x6 URDF/controller YAML and pass six_wheel:=true to the ECU/dynamics nodes",
        ),
        OpaqueFunction(function=launch_setup),
    ])
