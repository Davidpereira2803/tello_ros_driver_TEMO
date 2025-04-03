from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    tello_game_interface_cmd = Node(
        package="tello_game_interface",
        executable="tello_game_interface",
        output="screen",
    )

    ld = LaunchDescription()
    ld.add_action(tello_game_interface_cmd)

    return ld
