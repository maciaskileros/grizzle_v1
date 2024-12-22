from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='gazebo_ros',
            executable='gzserver',
            arguments=['-s', 'libgazebo_ros_factory.so'],
            output='screen'
        ),
        Node(
            package='gazebo_ros',
            executable='gzclient',
            output='screen'
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            arguments=['urdf_assembly.urdf']
        )
    ])
