from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='diaros_package',
            executable='pub',
            output='screen'
        ),
        Node(
            package='diaros_package',
            executable='relay',
            output='screen'
        ),
        Node(
            package='diaros_package',
            executable='sub',
            output='screen'
        ),
    ])