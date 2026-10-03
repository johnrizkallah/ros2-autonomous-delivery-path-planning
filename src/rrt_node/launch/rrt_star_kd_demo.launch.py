from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    map_yaml = LaunchConfiguration("map_yaml")

    return LaunchDescription([
        DeclareLaunchArgument(
            "map_yaml",
            default_value="/home/john-rizkallah/thesis_ros2_public/src/rrt_node/maps/map8.yaml",
        ),
        Node(
            package="rrt_node",
            executable="rrt_star_kd_node",
            name="rrt_star_kd_node",
            output="screen",
            parameters=[{
                "map_yaml": map_yaml,
                "map_frame": "map",
                "start_option": 0,
                "goal_option": 2,
            }],
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
        ),
    ])
