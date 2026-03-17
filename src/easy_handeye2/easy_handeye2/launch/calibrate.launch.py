from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import LaunchConfigurationEquals
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    arg_name = DeclareLaunchArgument('name', default_value='ur10_realsense_eob')
    arg_calibration_type = DeclareLaunchArgument('calibration_type', default_value='eye_on_base')
    arg_tracking_base_frame = DeclareLaunchArgument('tracking_base_frame', default_value='camera_link')
    arg_tracking_marker_frame = DeclareLaunchArgument('tracking_marker_frame', default_value='marker_frame')
    arg_robot_base_frame = DeclareLaunchArgument('robot_base_frame', default_value='base_link')
    arg_robot_effector_frame = DeclareLaunchArgument('robot_effector_frame', default_value='tool0')
    arg_freehand_robot_movement = DeclareLaunchArgument('freehand_robot_movement', default_value='true')

    node_dummy_calib_eih = Node(package='tf2_ros', executable='static_transform_publisher', name='dummy_publisher',
                                condition=LaunchConfigurationEquals('calibration_type', 'eye_in_hand'),
                                arguments=f'--x 0 --y 0 --z 0.1 --qx 0 --qy 0 --qz 0 --qw 1'.split(' ') + ['--frame-id', LaunchConfiguration('robot_effector_frame'),
                                                                                                           '--child-frame-id', LaunchConfiguration('tracking_base_frame')])

    node_dummy_calib_eob = Node(package='tf2_ros', executable='static_transform_publisher', name='dummy_publisher',
                                condition=LaunchConfigurationEquals('calibration_type', 'eye_on_base'),
                                arguments=f'--x 1 --y 0 --z 0 --qx 0 --qy 0 --qz 0 --qw 1'.split(' ') + ['--frame-id', LaunchConfiguration('robot_base_frame'),
                                                                                                         '--child-frame-id', LaunchConfiguration('tracking_base_frame')])

    handeye_server = Node(package='easy_handeye2', executable='handeye_server', name='handeye_server', parameters=[{
        'name': LaunchConfiguration('name'),
        'calibration_type': LaunchConfiguration('calibration_type'),
        'tracking_base_frame': LaunchConfiguration('tracking_base_frame'),
        'tracking_marker_frame': LaunchConfiguration('tracking_marker_frame'),
        'robot_base_frame': LaunchConfiguration('robot_base_frame'),
        'robot_effector_frame': LaunchConfiguration('robot_effector_frame'),
        'freehand_robot_movement': LaunchConfiguration('freehand_robot_movement'),
    }])

    handeye_rqt_calibrator = Node(package='easy_handeye2', executable='rqt_calibrator.py',
                                  name='handeye_rqt_calibrator',
                                  # arguments=['--ros-args', '--log-level', 'debug'],
                                  parameters=[{
                                      'name': LaunchConfiguration('name'),
                                      'calibration_type': LaunchConfiguration('calibration_type'),
                                      'tracking_base_frame': LaunchConfiguration('tracking_base_frame'),
                                      'tracking_marker_frame': LaunchConfiguration('tracking_marker_frame'),
                                      'robot_base_frame': LaunchConfiguration('robot_base_frame'),
                                      'robot_effector_frame': LaunchConfiguration('robot_effector_frame'),
                                      'freehand_robot_movement': LaunchConfiguration('freehand_robot_movement'),
                                  }])

    return LaunchDescription([
        arg_name,
        arg_calibration_type,
        arg_tracking_base_frame,
        arg_tracking_marker_frame,
        arg_robot_base_frame,
        arg_robot_effector_frame,
        arg_freehand_robot_movement,
        node_dummy_calib_eih,
        node_dummy_calib_eob,
        handeye_server,
        handeye_rqt_calibrator,
    ])
