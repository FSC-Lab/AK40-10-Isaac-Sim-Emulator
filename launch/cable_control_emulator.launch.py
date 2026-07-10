from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg = get_package_share_directory("ak_motor_cable_control_emulator")
    params = os.path.join(pkg, "config", "cable_control_emulator_params.yaml")

    winch_prefix_arg = DeclareLaunchArgument(
        "isaac_winch_prefix", default_value="cable_winch_0/",
        description="Topic prefix ROS2CableWinchBackend publishes/subscribes under in Isaac Sim"
    )

    return LaunchDescription([
        winch_prefix_arg,
        Node(
            package="ak_motor_cable_control_emulator",
            executable="ak_motor_cable_control_node",
            name="ak_motor_cable_control_node",
            output="screen",
            parameters=[params, {"isaac_winch_prefix": LaunchConfiguration("isaac_winch_prefix")}],
        ),
    ])
