from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    default_config = PathJoinSubstitution([
        FindPackageShare("agv_arm_description"),
        "config",
        "agv_arm_geometry.yaml",
    ])

    config_arg = DeclareLaunchArgument(
        "config",
        default_value=default_config,
        description="Robot geometry YAML file.",
    )
    rviz_config = PathJoinSubstitution([
        FindPackageShare("agv_arm_description"),
        "rviz",
        "display.rviz",
    ])
    rviz_config_arg = DeclareLaunchArgument(
        "rviz_config",
        default_value=rviz_config,
        description="RViz configuration file.",
    )

    robot_description = {
        "robot_description": Command([
            "ros2 run agv_arm_description generate_agv_arm_urdf --config ",
            LaunchConfiguration("config"),
        ])
    }
    home_joint_positions = {
        "zeros.right_joint_1": 0.0,
        "zeros.right_joint_2": 0.0,
        "zeros.right_joint_3": 0.349066,
        "zeros.right_joint_4": -0.610865,
        "zeros.right_joint_5": 0.0,
        "zeros.right_joint_6": 0.0,
        "zeros.left_joint_2": 0.0,
        "zeros.left_joint_3": -0.349066,
        "zeros.left_joint_4": 0.610865,
        "zeros.left_joint_5": 0.0,
        "zeros.left_joint_6": 0.0,
    }

    return LaunchDescription([
        config_arg,
        rviz_config_arg,
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[robot_description],
            output="screen",
        ),
        Node(
            package="joint_state_publisher_gui",
            executable="joint_state_publisher_gui",
            parameters=[home_joint_positions],
            output="screen",
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            arguments=["-d", LaunchConfiguration("rviz_config")],
            output="screen",
        ),
    ])
