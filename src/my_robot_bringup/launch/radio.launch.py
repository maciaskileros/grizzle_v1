from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    for name in ["giskard", "bb8", "daneel", "jander", "c3po"]:
        robot_news_station_node = Node(
            package="my_py_pkg",
            executable="robot_news_station",
            name=f"robot_news_station_{name}",
            parameters=[
                {"robot_name":name}
            ]
        )
        ld.add_action(robot_news_station_node)

    smartphone_node = Node(
        package="my_py_pkg",
        executable="smartphone",
        name="smartphone"
    )
    ld.add_action(smartphone_node)

    return ld