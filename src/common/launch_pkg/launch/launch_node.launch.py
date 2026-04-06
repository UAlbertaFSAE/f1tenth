from datetime import datetime
from pathlib import Path

from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch.actions import ExecuteProcess, SetLaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
    run_log_dir = Path.cwd() / 'recordings' / timestamp
    run_log_dir.mkdir(parents=True, exist_ok=True)
    node_log_dir = run_log_dir / 'node_logs'
    node_log_dir.mkdir(parents=True, exist_ok=True)

    bag_output_dir = run_log_dir / 'rosbag'
    zed_launch_log = node_log_dir / 'zed_camera_launch.txt'
    detection_launch_log = node_log_dir / 'camera_detection_launch.txt'
    waypoint_launch_log = node_log_dir / 'new_triangulator_launch.txt'
    cone_transformer_log = node_log_dir / 'cone_transformer.txt'
    pure_pursuit_log = node_log_dir / 'pure_pursuit.txt'
    rosbag_topics = [
        '/clicked_point',
        '/cone_positions',
        '/cone_transformed',
        '/current_waypoint',
        '/detection_visualization/depth',
        '/detection_visualization/detections',
        '/diagnostics',
        '/drive',
        '/lookahead_waypoint',
        '/odom',
        '/parameter_events',
        '/planned_path',
        '/rosout',
        '/tf',
        '/tf_static',
        '/waypoints',
        '/zed/zed_node/depth/depth_registered',
        '/zed/zed_node/depth/depth_registered/camera_info',
        '/zed/zed_node/imu/data',
        '/zed/zed_node/odom',
        '/zed/zed_node/rgb/color/rect/image',
    ]

    zed_launch = ExecuteProcess(
        cmd=[
            'bash',
            '-lc',
            f'ros2 launch zed_wrapper zed_camera.launch.py camera_model:=zed2i 2>&1 | tee -a "{zed_launch_log}"',
        ],
        name='zed_camera_launch',
        output='screen',
    )

    detection_launch = ExecuteProcess(
        cmd=[
            'bash',
            '-lc',
            f'ros2 launch detection_camera camera_detection.launch.py 2>&1 | tee -a "{detection_launch_log}"',
        ],
        name='camera_detection_launch',
        output='screen',
    )

    waypoint_launch = ExecuteProcess(
        cmd=[
            'bash',
            '-lc',
            f'ros2 launch waypoint_new new_triangulator.launch.py 2>&1 | tee -a "{waypoint_launch_log}"',
        ],
        name='new_triangulator_launch',
        output='screen',
    )

    pkg_share = FindPackageShare("launch_pkg")

    rviz_config_path = PathJoinSubstitution(
        [pkg_share, "config", "waypoint_viz.rviz"])

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="launch_rviz",
        arguments=["-d", rviz_config_path],
        output="screen",
    )


    return LaunchDescription(
        [
            SetLaunchConfiguration('log_dir', str(run_log_dir)),
            zed_launch,
            detection_launch,
            ExecuteProcess(
                cmd=['ros2', 'run', 'tf2_ros', 'static_transform_publisher', '-0.05',
                    '-0.15', '0.40', '0.0', '0.0', '0.0', 'base_link', 'zed_camera_link'],
                output='screen'
            ),
            ExecuteProcess(
                cmd=['ros2', 'run', 'tf2_ros', 'static_transform_publisher', '0.00',
                    '0.00', '0.00', '0.0', '0.0', '0.0', 'map', 'base_link'],
                output='screen'
            ),
            ExecuteProcess(
                cmd=[
                    'bash',
                    '-lc',
                    f'ros2 run cone_transformer cone_transformer 2>&1 | tee -a "{cone_transformer_log}"',
                ],
                name='cone_transformer',
                output='screen',
            ),
            waypoint_launch,
            ExecuteProcess(
                cmd=[
                    'bash',
                    '-lc',
                    f'ros2 run pure_pursuit pure_pursuit 2>&1 | tee -a "{pure_pursuit_log}"',
                ],
                name='pure_pursuit',
                output='screen',
            ),
            ExecuteProcess(
                cmd=['ros2', 'bag', 'record', '-o', str(bag_output_dir), *rosbag_topics],
                name='rosbag_record',
                output='log',
            ),
            # rviz_node,
        ]
    )
