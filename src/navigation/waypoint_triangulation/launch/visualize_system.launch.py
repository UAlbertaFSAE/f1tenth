from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


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

    # Declare args with concrete default values (avoid using LaunchConfiguration as default_value)
    ld.add_action(DeclareLaunchArgument('zed_pkg', default_value='zed_wrapper', description='Package that runs the ZED camera'))
    ld.add_action(DeclareLaunchArgument('zed_exe', default_value='zed_wrapper_node', description='Executable for ZED camera node'))
    ld.add_action(DeclareLaunchArgument('detection_pkg', default_value='detection_camera', description='Package for detection node'))
    ld.add_action(DeclareLaunchArgument('detection_exe', default_value='detection_node', description='Executable for detection node'))
    ld.add_action(DeclareLaunchArgument('cone_pkg', default_value='cone_transformer', description='Package for cone_transformer'))
    ld.add_action(DeclareLaunchArgument('cone_exe', default_value='cone_transformer', description='Executable for cone_transformer'))
    ld.add_action(DeclareLaunchArgument('waypoint_pkg', default_value='waypoint_triangulation', description='Package for waypoint node'))
    ld.add_action(DeclareLaunchArgument('waypoint_exe', default_value='waypoint_new', description='Executable for waypoint node'))
    ld.add_action(DeclareLaunchArgument('pure_pursuit_pkg', default_value='pure_pursuit', description='Package for pure-pursuit'))
    ld.add_action(DeclareLaunchArgument('pure_pursuit_exe', default_value='pure_pursuit', description='Executable for pure-pursuit'))

    # Include zed_camera.launch.py
    zed_camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('zed_wrapper'),
                'launch',
                'zed_camera.launch.py'
            ])
        ]),
        launch_arguments={'camera_model': 'zed2i'}.items()
    )

    # Include camera_detection.launch.py
    camera_detection_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('detection_camera'),
                'launch',
                'camera_detection.launch.py'
            ])
        ])
    )

    # 1) ZED camera node is now included only via zed_camera.launch.py with camera_model:=zed2i

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

    # Add nodes and included launch files to launch description in the requested order
    ld.add_action(zed_camera_launch)
    ld.add_action(camera_detection_launch)
    ld.add_action(static_tf)
    ld.add_action(cone_node)
    ld.add_action(waypoint_node)
    ld.add_action(pure_node)
    ld.add_action(rviz_node)

    return ld
