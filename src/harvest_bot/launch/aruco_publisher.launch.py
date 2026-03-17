from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Arguments
    arg_marker_id = DeclareLaunchArgument('marker_id', default_value='582', description='Marker ID to detect')
    arg_marker_size = DeclareLaunchArgument('marker_size', default_value='0.15', description='Marker size in meters')
    arg_ref_frame = DeclareLaunchArgument('reference_frame', default_value='camera_color_optical_frame', description='Frame to reference marker to')
    arg_camera_frame = DeclareLaunchArgument('camera_frame', default_value='camera_color_optical_frame', description='Camera optical frame')
    arg_marker_dict = DeclareLaunchArgument('marker_dictionary', default_value='DICT_4X4_1000', description='Marker dictionary type')
    
    # Aruco Node
    aruco_node = Node(
        package='aruco_ros',
        executable='single',
        name='aruco_single',
        parameters=[{
            'image_is_rectified': False,
            'marker_size': LaunchConfiguration('marker_size'),
            'marker_id': LaunchConfiguration('marker_id'),
            'reference_frame': LaunchConfiguration('reference_frame'),
            'camera_frame': LaunchConfiguration('camera_frame'),
            'marker_frame': 'marker_frame',
            'marker_dictionary': LaunchConfiguration('marker_dictionary'),
        }],
        remappings=[
            ('/camera_info', '/camera/camera/color/camera_info'),
            ('/image', '/camera/camera/color/image_raw'),
        ]
    )

    return LaunchDescription([
        arg_marker_id,
        arg_marker_size,
        arg_ref_frame,
        arg_camera_frame,
        arg_marker_dict,
        aruco_node,
    ])
