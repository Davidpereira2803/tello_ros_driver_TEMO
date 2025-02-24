from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    esp32_controller_cmd = Node(
        package="esp32_controller",
        executable="esp32_controller",
        output="screen",
    )

    ld = LaunchDescription()
    ld.add_action(esp32_controller_cmd)

    return ld
