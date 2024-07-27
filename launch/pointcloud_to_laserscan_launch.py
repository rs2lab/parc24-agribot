from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from parc24_phase1_agribot.perceiver import SensorType


def generate_launch_description():
    launch_config = LaunchConfiguration(variable_name="cloud_scanner")
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                name="cloud_scanner",
                default_value="cloud_scanner",
                description="Namespace for points clouds to laser scanner topics",
            ),
            Node(
                package="pointcloud_to_laserscan",
                executable="pointcloud_to_laserscan_node",
                remappings=[
                    ("cloud_in", SensorType.POINT_CLOUD.value),
                    ("scan", [launch_config, "/scan"]),
                ],
                parameters=[
                    {
                        "target_frame": "zed_camera_link",
                        "transform_tolerance": 0.01,
                        "min_height": 0.0,
                        "max_height": 1.0,
                        "angle_min": -1.5708,  # -M_PI/2
                        "angle_max": 1.5708,  # M_PI/2
                        "angle_increment": 0.0087,  # M_PI/360.0
                        "scan_time": 0.1,
                        "range_min": 0.45,
                        "range_max": 4.0,
                        "use_inf": True,
                        "inf_epsilon": 1.0,
                    }
                ],
                name="pointcloud_to_laserscan",
            ),
        ]
    )
