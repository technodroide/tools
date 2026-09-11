import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    pkg_share = get_package_share_directory('apriltag_detector')

    # Config file paths
    apriltag_config_file = os.path.join(pkg_share, 'config', 'tags_params.yaml')
    #usb_cam_info_url = 'file://' + os.path.join(pkg_share, 'config', 'explorer_camera_calib.yaml')

    container = ComposableNodeContainer(
        name='vision_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            
            # 1. USB Camera Component
            #ComposableNode(
            #    package='usb_cam',
            #    plugin='usb_cam::UsbCamNode',
            #    name='camera_explorer',
            #    parameters=[
            #        {'camera_name': 'camera_explorer'},
            #        {'video_device': '/dev/video2'},
            #        {'camera_info_url': usb_cam_info_url},
            #        {'frame_id': 'camera_explorer'}
            #    ],
            #    extra_arguments=[{'use_intra_process_comms': True}]
            #),

            # 2. AprilTag Detector Component
            ComposableNode(
                package='apriltag_detector',
                plugin='vision_tools::AprilTagDetector',
                name='apriltag_detector',
                parameters=[apriltag_config_file],
                extra_arguments=[{'use_intra_process_comms': True},
                                 {'allow_undeclared_parameters': True},
                                 {'automatically_declare_parameters_from_overrides': True}]
            ),

            # 3. Goal Bridge Component
            ComposableNode(
                package='apriltag_detector',
                plugin='vision_tools::AprilTagBridge',
                name='apriltag_bridge',
                parameters=[{
                    'target_frame': 'base_link',
                    'base_frame': 'camera_link'
                }],
                extra_arguments=[{'use_intra_process_comms': True}]
            ),
        ],
        output='screen',
    )

    return LaunchDescription([container])