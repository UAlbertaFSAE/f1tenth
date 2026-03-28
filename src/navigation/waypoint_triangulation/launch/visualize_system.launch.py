from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import ExecuteProcess


def generate_launch_description():
    # launch arguments allow overriding package/executable names if needed
    zed_pkg = LaunchConfiguration('zed_pkg', default='zed_wrapper')
    zed_exe = LaunchConfiguration('zed_exe', default='zed_wrapper_node')
    detection_pkg = LaunchConfiguration(
        'detection_pkg', default='detection_camera')
    detection_exe = LaunchConfiguration(
        'detection_exe', default='detection_node')
    cone_pkg = LaunchConfiguration('cone_pkg', default='cone_transformer')
    cone_exe = LaunchConfiguration('cone_exe', default='cone_transformer')
    waypoint_pkg = LaunchConfiguration(
        'waypoint_pkg', default='waypoint_triangulation')
    waypoint_exe = LaunchConfiguration('waypoint_exe', default='waypoint_new')
    pure_pursuit_pkg = LaunchConfiguration(
        'pure_pursuit_pkg', default='pure_pursuit')
    pure_pursuit_exe = LaunchConfiguration(
        'pure_pursuit_exe', default='pure_pursuit')

    rviz_config = PathJoinSubstitution([
        FindPackageShare(
            'waypoint_triangulation'), 'config', 'waypoint_viz.rviz'
    ])

    ld = LaunchDescription()

    # Declare args
    for name in [
        ('zed_pkg', 'Package that runs the ZED camera'),
        ('zed_exe', 'Executable for ZED camera node'),
        ('detection_pkg', 'Package for detection node'),
        ('detection_exe', 'Executable for detection node'),
        ('cone_pkg', 'Package for cone_transformer'),
        ('cone_exe', 'Executable for cone_transformer'),
        ('waypoint_pkg', 'Package for waypoint node'),
        ('waypoint_exe', 'Executable for waypoint node'),
        ('pure_pursuit_pkg', 'Package for pure-pursuit'),
        ('pure_pursuit_exe', 'Executable for pure-pursuit'),
    ]:
        ld.add_action(DeclareLaunchArgument(name[0], default_value=LaunchConfiguration(
            name[0]).perform({}), description=name[1]))

    # 1) ZED camera
    zed_node = Node(
        package=zed_pkg,
        executable=zed_exe,
        name='zed_camera',
        output='screen'
    )

    # 2) Detection node
    detection_node = Node(
        package=detection_pkg,
        executable=detection_exe,
        name='detection_node',
        output='screen'
    )

    # 3) static transform publisher (as given)
    static_tf = ExecuteProcess(
        cmd=['ros2', 'run', 'tf2_ros', 'static_transform_publisher', '-0.05',
             '-0.15', '0.40', '0.0', '0.0', '0.0', 'base_link', 'zed_camera_link'],
        output='screen'
    )

    # 4) cone_transformer
    cone_node = Node(
        package=cone_pkg,
        executable=cone_exe,
        name='cone_transformer',
        output='screen'
    )

    # 5) waypoint node
    waypoint_node = Node(
        package=waypoint_pkg,
        executable=waypoint_exe,
        name='waypoint_new',
        output='screen'
    )

    # 6) pure-pursuit
    pure_node = Node(
        package=pure_pursuit_pkg,
        executable=pure_pursuit_exe,
        name='pure_pursuit',
        output='screen'
    )

    # RViz single visualizer
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config]
    )

    # Add nodes to launch description in the requested order
    ld.add_action(zed_node)
    ld.add_action(detection_node)
    ld.add_action(static_tf)
    ld.add_action(cone_node)
    ld.add_action(waypoint_node)
    ld.add_action(pure_node)
    ld.add_action(rviz_node)

    return ld
