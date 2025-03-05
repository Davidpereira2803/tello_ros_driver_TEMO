from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    interface_cmd = Node(
        package="interface",
        executable="interface",
        output="screen",
    )

    ld = LaunchDescription()
    ld.add_action(interface_cmd)

    return ld
