import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    # Gets path for joy_teleop.yaml to get key mapping for the joystick
    joy_config = os.path.join(
        get_package_share_directory('f1tenth_core'),
        'config',
        'joy_teleop.yaml'
    )

    #Declaring nodes for launch
    joy_node = Node(
        package = 'joy',
        namespace = 'f1tenth',
        name = 'joy_node',
        executable = 'joy_node',
    )
    joy_teleop_node = Node(
        package = 'joy_teleop',
        namespace = 'f1tenth',
        name = 'joy_teleop',
        executable = 'joy_teleop',
        parameters = [joy_config]
    )

    ld.add_action(joy_node)
    ld.add_action(joy_teleop_node)

    
    return ld