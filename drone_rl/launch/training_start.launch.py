import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    ld = LaunchDescription()

    training_parameters = os.path.join(
        get_package_share_directory('hospital_robot_spawner'),
        'config',
        'training_parameters.yaml'
    )

    drone_rl = Node(
        package='drone_rl',
        executable='',
        #name='hospitalbot_training',
        parameters=[training_parameters]
    )

    ld.add_action(drone_rl)

    return ld