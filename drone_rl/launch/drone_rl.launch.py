from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    drone_rl = Node(
        package="drone_rl",
        executable="drone_rl"
    )

    ld.add_action(drone_rl)

    return ld
