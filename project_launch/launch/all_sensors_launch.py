from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Node(
        #     package='absolute_altimeter',
        #     # namespace='turtlesim1',
        #     executable='absolute_altimeter',
        #     name='absolute_altimeter',
        #     output='screen'
        # ),
        # Node(
        #     package='system_monitor',
        #     # namespace='turtlesim2',
        #     executable='system_monitor',
        #     name='system_monitor',
        #     output='screen'
        # ),
        # Node(
        #     package='magnetometer',
        #     executable='magnetometer',
        #     name='magnetometer',
        #     output='screen'
        # ),
        Node(
            package='imu_gyro',
            executable='imu_gyro',
            name='imu_gyro',
            output='screen'
        ),
        Node(
            package='state_estimator',
            executable='state_estimator',
            name='state_estimator',
            output='screen'
        ),
        ExecuteProcess(
            cmd=[
                'ros2', 'bag', 'record',
                # all relevant topics
                '-a',
                '-x', '/rosout',  # exclude certain

                # # all topics
                # '-a',

                # # list of topics
                # '/absolute_height',
                # '/battery_voltage',
                # '/magnetometer',
            ],
            output='screen'
        )
    ])