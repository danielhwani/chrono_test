"""
Quick URDF visualization for chrono_vehicle.urdf -- robot_state_publisher +
joint_state_publisher_gui + rviz2, using chrono_vehicle.rviz's saved config
(Fixed Frame=base_link, TF display on). No visual geometry on the URDF's
links (it's the step-3 <ros2_control>-only URDF, not meant for rendering),
so what you'll actually see in RViz is TF coordinate-frame axes at each
joint -- move the joint_state_publisher_gui sliders to see the front
steering joints rotate.

A launch file (not a bare `ros2 run ... -p robot_description:=...` CLI
call) is used deliberately: passing the URDF's full multi-line XML content
as a raw `-p key:=value` CLI argument breaks rcl's argument parser (tried
first, failed with "Couldn't parse parameter override rule"). The
Command/ParameterValue substitution below reads the file properly instead.

Run:
    ros2 launch ros2_control/urdf/display.launch.py
"""
import os

from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

HERE = os.path.dirname(os.path.abspath(__file__))


def generate_launch_description():
    urdf_path = os.path.join(HERE, "chrono_vehicle.urdf")
    rviz_path = os.path.join(HERE, "chrono_vehicle.rviz")
    robot_description = ParameterValue(Command(["cat ", urdf_path]), value_type=str)

    return LaunchDescription([
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[{"robot_description": robot_description}],
        ),
        Node(
            package="joint_state_publisher_gui",
            executable="joint_state_publisher_gui",
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            arguments=["-d", rviz_path],
        ),
    ])
