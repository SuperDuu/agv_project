import os
import yaml
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import subprocess

def load_file(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_path = os.path.join(package_path, file_path)
    try:
        with open(absolute_path, "r") as file:
            return file.read()
    except EnvironmentError:
        return None

def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_path = os.path.join(package_path, file_path)
    try:
        with open(absolute_path, "r") as file:
            return yaml.safe_load(file)
    except EnvironmentError:
        return None

def generate_launch_description():
    # Generate URDF
    geometry_yaml = os.path.join(
        get_package_share_directory("agv_arm_description"),
        "config",
        "agv_arm_geometry.yaml"
    )
    urdf_content = subprocess.check_output([
        "ros2", "run", "agv_arm_description", "generate_agv_arm_urdf",
        "--config", geometry_yaml
    ]).decode("utf-8")

    robot_description = {"robot_description": urdf_content}

    # Load SRDF
    srdf_content = load_file("agv_arm_moveit", "config/agv_dual_arm.srdf")
    robot_description_semantic = {"robot_description_semantic": srdf_content}

    # Load Kinematics
    kinematics_yaml = load_yaml("agv_arm_moveit", "config/kinematics.yaml")
    robot_description_kinematics = {"robot_description_kinematics": kinematics_yaml}

    # Load Joint Limits
    joint_limits_yaml = load_yaml("agv_arm_moveit", "config/joint_limits.yaml")
    joint_limits = {"robot_description_planning": joint_limits_yaml}

    # Load OMPL Planning config
    ompl_planning_yaml = load_yaml("agv_arm_moveit", "config/ompl_planning.yaml")
    ompl_config = {
        "planning_pipelines": ["ompl"],
        "ompl": ompl_planning_yaml
    }

    # Planning Scene Monitor Parameters
    planning_scene_monitor_parameters = {
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
    }

    # Start move_group node
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
            {"use_sim_time": False}
        ],
    )

    # Start robot state publisher
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description],
    )

    # Start joint state publisher
    joint_state_publisher_node = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        output="screen",
        parameters=[{"use_sim_time": False}]
    )

    # Start moveit_planner node
    moveit_planner_node = Node(
        package="agv_arm_moveit",
        executable="moveit_planner",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            joint_limits,
            {"use_sim_time": False}
        ],
    )

    return LaunchDescription([
        robot_state_publisher_node,
        joint_state_publisher_node,
        move_group_node,
        moveit_planner_node
    ])
