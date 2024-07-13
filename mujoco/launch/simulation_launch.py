import os
from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    xml_file_name = "model/xml/universal_robots_ur5e/scene.xml"
    xml_file = os.path.join(
        get_package_share_path("description"), xml_file_name)

    return LaunchDescription(
        [
            Node(
                package="mujoco_ros2",
                executable="mujoco_node",
                name="simulation_mujoco",
                output="screen",
                parameters=[
                    {"simulation/model_file": xml_file},
                ],
                emulate_tty=True,
                arguments=[("__log_level:=debug")],
            ),
        ]
    )
