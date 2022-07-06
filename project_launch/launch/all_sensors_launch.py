from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='absolute_altimeter',
            # namespace='turtlesim1',
            executable='absolute_altimeter',
            name='absolute_altimeter',
            output='screen'
        ),
        Node(
            package='system_monitor',
            # namespace='turtlesim2',
            executable='system_monitor',
            name='system_monitor',
            output='screen'
        ),
        Node(
            package='magnetometer',
            executable='magnetometer',
            name='magnetometer',
            output='screen'
        ),
        ExecuteProcess(
            cmd = ['ros2', 'bag', 'record', '/absolute_height', '/battery_voltage', '/magnetometer'],
            output='screen'
        )
    ])