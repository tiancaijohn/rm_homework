from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    map_file = LaunchConfiguration("map")

    map_server = Node(
        package="nav2_map_server",
        executable="map_server",
        output="screen",
        parameters=[{
            "yaml_filename": ParameterValue(map_file, value_type=str),
        }],
    )

    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=["--frame-id", "map", "--child-frame-id", "base_link"],
    )

    costmap = Node(
        package="nav2_costmap_2d",
        executable="nav2_costmap_2d",
        output="screen",
        parameters=[{
            "global_frame": "map",
            "robot_base_frame": "base_link",
            "track_unknown_space": True,
            "always_send_full_costmap": True,
            "update_frequency": 2.0,
            "publish_frequency": 1.0,
            "robot_radius": 0.15,
            "plugins": ["static_layer", "inflation_layer"],
            "static_layer.plugin": "nav2_costmap_2d::StaticLayer",
            "static_layer.map_topic": "/map",
            "inflation_layer.plugin": "nav2_costmap_2d::InflationLayer",
            "inflation_layer.inflation_radius": 0.30,
        }],
    )

    manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        output="screen",
        parameters=[{
            "autostart": True,
            "node_names": ["/map_server", "/costmap/costmap"],
            "bond_timeout": 0.0,
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument("map"),
        map_server,
        static_tf,
        costmap,
        manager,
    ])