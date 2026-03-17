import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

def generate_launch_description():
    strawberry_pkg = FindPackageShare('harvest_bot')
    easy_handeye_pkg = FindPackageShare('easy_handeye2')

    # Arguments for ArUco
    arg_marker_id = DeclareLaunchArgument('marker_id', default_value='582')
    arg_marker_size = DeclareLaunchArgument('marker_size', default_value='0.15', description='Marker size in m')
    arg_marker_dict = DeclareLaunchArgument('marker_dictionary', default_value='DICT_4X4_1000', description='Dictionary type')

    # 1. Launch ArUco Publisher (Required for marker_frame)
    # We use our existing launch file for this
    aruco_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            strawberry_pkg, '/launch/aruco_publisher.launch.py'
        ]),
        launch_arguments={
            'marker_id': LaunchConfiguration('marker_id'),
            'marker_size': LaunchConfiguration('marker_size'),
            'marker_dictionary': LaunchConfiguration('marker_dictionary'),
            'reference_frame': 'camera_color_optical_frame',
            'camera_frame': 'camera_color_optical_frame',
        }.items(),
    )

    # 2. Launch EasyHandEye2 Calibration
    # We call the standard calibrate.launch.py with OUR parameters hardcoded
    calibration_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            easy_handeye_pkg, '/launch/calibrate.launch.py'
        ]),
        launch_arguments={
            'name': 'ur10_realsense_eih',
            'calibration_type': 'eye_in_hand',
            'robot_base_frame': 'base_link',
            'robot_effector_frame': 'tool0',
            'tracking_base_frame': 'camera_link',
            'tracking_marker_frame': 'marker_frame',
            'freehand_robot_movement': 'true', 
            'calibration_file': os.path.join(os.environ['HOME'], 'strawberry_ws/src/harvest_bot/config/ur10_realsense_eih.calib'),
        }.items(),
    )

    return LaunchDescription([
        arg_marker_id,
        arg_marker_size,
        arg_marker_dict,
        aruco_launch,
        calibration_launch,
    ])
