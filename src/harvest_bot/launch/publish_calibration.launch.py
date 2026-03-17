import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    return LaunchDescription([
        # 1. Launch RealSense
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                FindPackageShare("harvest_bot"), "/launch/realsense.launch.py"
            ]),
        ),

        # 2. Publish Calibration
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                FindPackageShare("easy_handeye2"), "/launch/publish.launch.py"
            ]),
            launch_arguments={
                "name": "ur10_realsense_eih",
                "calibration_file": [FindPackageShare("harvest_bot"), "/config/ur10_realsense_eih.calib"]
            }.items(),
        ),
    ])
