from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Start the robot action server
        Node(
            package='vacuum_robot_action',
            executable='robot_node',
            name='robot',
            output='screen',
            emulate_tty=True,
        ),
        # Start the charging station action client
        Node(
            package='vacuum_robot_action',
            executable='charging_station_node',
            name='charging_station',
            output='screen',
            emulate_tty=True,
        ),
    ])