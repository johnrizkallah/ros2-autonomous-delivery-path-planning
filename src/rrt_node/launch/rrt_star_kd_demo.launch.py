from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    map_yaml = LaunchConfiguration("map_yaml")
    start_option = LaunchConfiguration("start_option")
    goal_option = LaunchConfiguration("goal_option")

    return LaunchDescription([
        DeclareLaunchArgument(
            "map_yaml",
            default_value=(
                "/home/john-rizkallah/thesis_ros2_public/"
                "src/rrt_node/maps/map8.yaml"
            ),
            description="Path to the occupancy-grid YAML file",
        ),

        DeclareLaunchArgument(
            "start_option",
            default_value="0",
            description="Legacy start-position preset index",
        ),

        DeclareLaunchArgument(
            "goal_option",
            default_value="3",
            description="Legacy goal-position preset index",
        ),

        Node(
            package="nav2_map_server",
            executable="map_server",
            name="map_server",
            output="screen",
            parameters=[{
                "yaml_filename": map_yaml,
                "topic_name": "map",
                "frame_id": "map",
                "use_sim_time": False,
            }],
        ),

        Node(
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            name="lifecycle_manager_map",
            output="screen",
            parameters=[{
                "autostart": True,
                "node_names": ["map_server"],
                "use_sim_time": False,
            }],
        ),

        Node(
            package="rrt_node",
            executable="rrt_star_kd_node",
            name="rrt_star_kd_node",
            output="screen",
            parameters=[{
                "map_yaml": map_yaml,
                "map_frame": "map",
                "start_option": start_option,
                "goal_option": goal_option,
                "animate_tree": True,
                "edges_per_batch": 100,
                "animation_period_ms": 40,
            }],
        ),

        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
            arguments=[
                "-d",
                "/home/john-rizkallah/thesis_ros2_public/src/"
                "rrt_node/launch/rrt_star_kd.rviz",
            ],
        ),
    ])
