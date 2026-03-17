from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "ur_type",
            default_value="ur10",
            description="Type/series of used UR robot.",
        )
    )

    ur_type = LaunchConfiguration("ur_type")

    # 1. UR Robot Driver (Fake Hardware)
    # This provides robot_state_publisher and ros2_control (publishing /joint_states)
    ur_control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare("ur_robot_driver"), "/launch/ur_control.launch.py"
        ]),
        launch_arguments={
            "ur_type": ur_type,
            "robot_ip": "0.0.0.0", # Irrelevant for fake
            "use_fake_hardware": "true",
            "launch_rviz": "false",
        }.items(),
    )

    # 2. UR MoveIt (With Fake Hardware & RViz)
    ur_moveit = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare("strawberry_moveit_config"), "/launch/ur_moveit.launch.py"
        ]),
        launch_arguments={
            "use_fake_hardware": "true",
            "launch_rviz": "true",
            "ur_type": ur_type,
        }.items(),
    )

    # 3. Add Pipe Collision Object
    #pipe_adder_node = Node(
     #   package="harvest_bot",
      #  executable="add_pipe.py",
       # name="pipe_adder",
        #output="screen",
    #)



    # 5. Publish Hand-Eye Calibration (Static Transform)
    # Calib: x=-0.017675 y=-0.243391 z=0.030015 | qx=0.508181 qy=-0.482978 qz=0.518120 qw=0.489934
    # Frame: tool0 -> camera_link
    calibration_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="camera_link_broadcaster",
        arguments=[
            "--x", "-0.017675",
            "--y", "-0.243391",
            "--z", "0.030015",
            "--qx", "0.508181",
            "--qy", "-0.482978",
            "--qz", "0.518120",
            "--qw", "0.489934",
            "--frame-id", "tool0",
            "--child-frame-id", "camera_link"
        ],
        output="screen",
    )

    # 6. Static Transform Publisher (World -> Base Link)
    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher_world_base",
        arguments=["0", "0", "0", "0", "0", "0", "world", "base_link"],
        output="screen",
    )

    return LaunchDescription(declared_arguments + [
        ur_control,
        ur_moveit,
        #pipe_adder_node,
        calibration_node,
        static_tf,
    ])
