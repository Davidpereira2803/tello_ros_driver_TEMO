from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    smartphone_inclinometer_cmd = Node(
        package="smartphone_inclinometer",
        executable="smartphone_inclinometer",
        output="screen",
    )


    ld = LaunchDescription()
    ld.add_action(smartphone_inclinometer_cmd)

    return ld
