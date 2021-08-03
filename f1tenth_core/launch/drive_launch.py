import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    ld = LaunchDescription()

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    
    # Opens the model file for robot_state_publisher to create the tf tree.
    # URDF file is under f1tenth_core/urdf/f1tenth_urdf.xml
    urdf_file_name = 'f1tenth_urdf.xml'
    urdf = os.path.join(
        get_package_share_directory('f1tenth_core'),
        'urdf',
        urdf_file_name
    )
    with open(urdf, 'r') as infp:
        robot_desc = infp.read()

    # Gets path for to params.yaml for vesc nodes.
    # Any configurable parameter can be found under the config folder
    # of this package.
    config = os.path.join(
        get_package_share_directory('f1tenth_core'),
        'config',
        'params.yaml'
    )
    
    #Declaring nodes for launch
    throttle_interpolator_node = Node(
        package = 'throttle_interpolator',
        namespace = 'f1tenth',
        name = 'throttle_interpolator',
        executable = 'throttle_interpolator',
        parameters = [config]
    )
    ackermann_to_vesc_node = Node(
        package = 'vesc_ackermann',
        namespace = 'f1tenth',
        name = 'ackermann_to_vesc',
        executable = 'ackermann_to_vesc_node',
        parameters = [config],
        remappings=[
            ("commands/motor/speed", "commands/motor/unsmoothed_speed"),
            ("commands/servo/position", "commands/servo/unsmoothed_position")
        ]
    )
    vesc_driver_node = Node(
        package = 'vesc_driver',
        namespace = 'f1tenth',
        name = 'vesc_driver',
        executable = 'vesc_driver_node',
        parameters = [config]
    )
    vesc_to_odom_node = Node(
        package = 'vesc_ackermann',
        namespace = 'f1tenth',
        name = 'vesc_to_odom',
        executable = 'vesc_to_odom_node',
        parameters = [config]
    )
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        namespace = 'f1tenth',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time, 'robot_description': robot_desc}],
        arguments=[urdf]
    )
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        namespace = 'f1tenth',
        executable='joint_state_publisher',
        name='joint_state_publisher',
    )
    hokuyo_node = Node(
        package='urg_node',
        namespace = 'f1tenth',
        executable='urg_node_driver',
        output='screen',
        parameters=[config]
    )
        
    ld.add_action(throttle_interpolator_node)
    ld.add_action(ackermann_to_vesc_node)
    ld.add_action(vesc_driver_node)
    ld.add_action(vesc_to_odom_node)
    ld.add_action(robot_state_publisher_node)
    ld.add_action(joint_state_publisher_node)
    ld.add_action(hokuyo_node)
    
    return ld