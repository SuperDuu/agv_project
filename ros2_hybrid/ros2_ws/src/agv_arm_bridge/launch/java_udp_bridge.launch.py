from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("listen_host", default_value="0.0.0.0"),
        DeclareLaunchArgument("listen_port", default_value="5010"),
        DeclareLaunchArgument("default_reply_host", default_value="127.0.0.1"),
        DeclareLaunchArgument("default_reply_port", default_value="5011"),
        Node(
            package="agv_arm_bridge",
            executable="java_udp_bridge",
            parameters=[{
                "listen_host": LaunchConfiguration("listen_host"),
                "listen_port": LaunchConfiguration("listen_port"),
                "default_reply_host": LaunchConfiguration("default_reply_host"),
                "default_reply_port": LaunchConfiguration("default_reply_port"),
            }],
            output="screen",
        ),
    ])
