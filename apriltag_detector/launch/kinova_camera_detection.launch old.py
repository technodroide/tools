import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node

from launch.substitutions import (
    PathJoinSubstitution
)
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # YAML param
    #controllers_yaml_path = PathJoinSubstitution(
    #    [
    #        FindPackageShare("visual_servoing"),            # package name
    #        "config",                                       # file localization
    #        "visual_servoing_param.yaml",                   # Yaml file name
    #    ]
    #)

    pkg_share = get_package_share_directory('apriltag_detector')
    apriltag_config_file = os.path.join(pkg_share, 'config', 'tags_params.yaml')

    AprilTagDetector = Node(
        package="apriltag_detector",
        executable="apriltag_detector_node",
        name="apriltag_detector",
        output="screen",
        parameters=[apriltag_config_file]
    )
    
    AprilTagBridge = Node(
        package="apriltag_detector",
        executable="apriltag_bridge_node",
        name="apriltag_detector",
        output="screen",
        parameters=[{
            'target_frame': 'base_link' 
        }]
    )

    return LaunchDescription([
        AprilTagDetector,
        AprilTagBridge,
        #shared_control_visualization_node,
    ])