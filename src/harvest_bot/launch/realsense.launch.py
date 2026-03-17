import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    realsense_dir = get_package_share_directory('realsense2_camera')

    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(realsense_dir, 'launch', 'rs_launch.py')),
        launch_arguments={
            'pointcloud.enable': 'true',
            'align_depth.enable': 'true',
            'enable_sync': 'true',
            'camera_name': 'camera',
            'device_type': 'd435',
            'depth_module.profile': '640x480x30',
            'rgb_camera.profile': '640x480x30',
            'rgb_camera.qos': 'reliable',
            'initial_reset': 'true',
            'filters': 'spatial,temporal,hole_filling',
        }.items()
    )

    return LaunchDescription([
        realsense_launch
    ])