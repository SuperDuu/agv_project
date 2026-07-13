"""
full_stack.launch.py
====================
Launches the complete AGV arm ROS2 stack:
  1. agv_arm_bridge   — Java UDP bridge (port 5010) + AGV arm planner
  2. agv_arm_moveit   — MoveIt2 move_group + state publishers
  NOTE: moveit_planner (one-shot demo) is NOT included here.
"""
import os
import yaml
import subprocess

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def load_file(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_path = os.path.join(package_path, file_path)
    try:
        with open(absolute_path, "r") as f:
            return f.read()
    except EnvironmentError:
        return None


def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_path = os.path.join(package_path, file_path)
    try:
        with open(absolute_path, "r") as f:
            return yaml.safe_load(f)
    except EnvironmentError:
        return None


def generate_launch_description():
    # ------------------------------------------------------------------ #
    # URDF / SRDF / config                                                #
    # ------------------------------------------------------------------ #
    geometry_yaml = os.path.join(
        get_package_share_directory("agv_arm_description"),
        "config",
        "agv_arm_geometry.yaml",
    )
    urdf_content = subprocess.check_output(
        ["ros2", "run", "agv_arm_description", "generate_agv_arm_urdf",
         "--config", geometry_yaml]
    ).decode("utf-8")

    robot_description            = {"robot_description": urdf_content}
    robot_description_semantic   = {"robot_description_semantic":  load_file("agv_arm_moveit", "config/agv_dual_arm.srdf")}
    robot_description_kinematics = {"robot_description_kinematics": load_yaml("agv_arm_moveit", "config/kinematics.yaml")}
    joint_limits                 = {"robot_description_planning":   load_yaml("agv_arm_moveit", "config/joint_limits.yaml")}

    ompl_planning_yaml = load_yaml("agv_arm_moveit", "config/ompl_planning.yaml")
    ompl_config = {
        "planning_pipelines": ["ompl"],
        "ompl": ompl_planning_yaml,
    }

    planning_scene_monitor_parameters = {
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
    }

    common_moveit_params = [
        robot_description,
        robot_description_semantic,
        robot_description_kinematics,
        joint_limits,
        {"use_sim_time": False},
    ]

    # ------------------------------------------------------------------ #
    # MoveIt2 nodes                                                        #
    # ------------------------------------------------------------------ #
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description],
    )

    joint_state_publisher_node = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        output="screen",
        parameters=[{"use_sim_time": False}],
    )

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            joint_limits,
            ompl_config,
            planning_scene_monitor_parameters,
            {"use_sim_time": False},
        ],
    )


    # ------------------------------------------------------------------ #
    # Bridge nodes (Java UDP ↔ ROS2)                                      #
    # ------------------------------------------------------------------ #
    java_udp_bridge_node = Node(
        package="agv_arm_bridge",
        executable="java_udp_bridge",
        output="screen",
        parameters=[{
            "listen_host": "0.0.0.0",
            "listen_port": 5010,
            "default_reply_host": "127.0.0.1",
            "default_reply_port": 5011,
        }],
    )

    agv_arm_planner_node = Node(
        package="agv_arm_bridge",
        executable="agv_arm_planner",
        output="screen",
    )

    # ------------------------------------------------------------------ #
    # Compose everything                                                   #
    # ------------------------------------------------------------------ #
    return LaunchDescription([
        robot_state_publisher_node,
        joint_state_publisher_node,
        move_group_node,
        java_udp_bridge_node,
        agv_arm_planner_node,
    ])
