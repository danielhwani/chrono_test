"""
분산 버전 (distributed version) launch file -- PC1 side only.

This brings up ros2_control (controller_manager + ChronoSupervisoryFmuSystemInterface
+ joint_state_broadcaster + ackermann_steering_controller) and
robot_state_publisher -- everything that's meant to run on PC1, alongside
whatever supervisory controller/Nav2 stack eventually sits on top. It does
NOT bring up the FMU dynamics -- that's chrono_fmu_dynamics_node,
reused completely unchanged, meant to run as its own separate process on a
separate PC2 (see the split launch file's use of the same node for what
that looks like when it IS colocated). Run PC2's side directly, no launch
file needed for a single node with two parameters:

    ros2 run chrono_fmu_dynamics_node chrono_fmu_dynamics_node \\
      --ros-args -p fmu_dir:=<path to native_vehicle_fmu on PC2> -p six_wheel:=<true|false>

The two sides find each other over ordinary ROS2/DDS discovery (the
`vehicle_command`/`vehicle_status` topics, chrono_split_msgs) -- on a real
2-machine setup that means both machines need to be on the same DDS
domain/network; for local testing on one machine (this repo's own
verification so far), it works exactly the same way over localhost, no
different from running the split version's chrono_fmu_dynamics_node.

`six_wheel` launch argument: same meaning/mechanism as the in-process and
split launch files' own six_wheel argument -- picks chrono_vehicle_supervisory_6x6.urdf
and chrono_vehicle_controllers_6x6.yaml instead of the 4-wheel ones. IMPORTANT:
this only affects THIS plugin's own axle-divisor math (see
ChronoSupervisoryFmuSystemInterface's header comment) -- it does NOT reach PC2.
PC2's chrono_fmu_dynamics_node needs its OWN --six_wheel:=true passed
separately when launched there; the two sides must be set to agree.

Run (PC1):
    cd ros2_control && colcon build
    source /opt/ros/humble/setup.bash && source install/setup.bash
    ros2 launch chrono_supervisory_fmu_hw_interface/launch/chrono_vehicle_supervisory_control.launch.py
    ros2 launch chrono_supervisory_fmu_hw_interface/launch/chrono_vehicle_supervisory_control.launch.py six_wheel:=true
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


def launch_setup(context, *args, **kwargs):
    six_wheel = LaunchConfiguration("six_wheel").perform(context) == "true"
    urdf_name = "chrono_vehicle_supervisory_6x6.urdf" if six_wheel else "chrono_vehicle_supervisory.urdf"
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
            # See the in-process/split launch files' matching comment --
            # ackermann_steering_controller's odom->base_link transform
            # needs remapping onto /tf for RViz to see the vehicle move.
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
            description=(
                "Use chrono_vehicle_supervisory_6x6.urdf instead of chrono_vehicle_supervisory.urdf "
                "(PC2's chrono_fmu_dynamics_node needs its own matching six_wheel:=true)"
            ),
        ),
        OpaqueFunction(function=launch_setup),
    ])
