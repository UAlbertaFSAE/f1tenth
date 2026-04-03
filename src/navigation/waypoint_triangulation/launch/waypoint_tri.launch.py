from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    cones_topic = LaunchConfiguration("cones_topic")
    waypoint_topic = LaunchConfiguration("waypoint_topic")
    marker_topic = LaunchConfiguration("marker_topic")
    frame_id = LaunchConfiguration("frame_id")

    rviz_config = PathJoinSubstitution(
        [FindPackageShare("waypoint_triangulation"),
         "config", "waypoint_viz.rviz"]
    )

    triangulator_node = Node(
        package="waypoint_triangulation",
        executable="triangulator",
        name="triangulator_node",
        output="screen",
        parameters=[
            {
                "cones_topic": cones_topic,
                "waypoint_topic": waypoint_topic,
                "marker_topic": marker_topic,
                "frame_id": frame_id,
            }
        ],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("cones_topic", default_value="/cone_data"),
            DeclareLaunchArgument(
                "waypoint_topic", default_value="/waypoints"),
            DeclareLaunchArgument(
                "marker_topic", default_value="/triangulation_markers"),
            DeclareLaunchArgument("frame_id", default_value="map"),
            triangulator_node,
            rviz_node,
        ]
    )
