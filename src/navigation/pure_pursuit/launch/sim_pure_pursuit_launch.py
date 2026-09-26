from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description() -> LaunchDescription:
    """Retain the old entry point as a wrapper around the canonical launcher."""
    share = get_package_share_directory("pure_pursuit")
    return LaunchDescription(
        [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    f"{share}/launch/pure_pursuit_launch.py"
                ),
                launch_arguments={"config_file": "sim_config.yaml"}.items(),
            )
        ]
    )
