from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition

def generate_launch_description():
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_fake_hardware",
            default_value="false",
            description="Start robot with fake hardware mirroring controller commands",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "launch_rviz",
            default_value="true",
            description="Launch RViz?",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_ip",
            default_value="192.168.11.100",
            description="IP address of the UR robot",
        )
    )

    use_fake_hardware = LaunchConfiguration("use_fake_hardware")
    launch_rviz = LaunchConfiguration("launch_rviz")
    robot_ip = LaunchConfiguration("robot_ip")
    ur_type = "ur10"

    # 1. RealSense
    realsense = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare("harvest_bot"), "/launch/realsense.launch.py"
        ]),
    )

    # 2. UR Robot Driver (Only if use_fake_hardware is false)
    ur_driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare("ur_robot_driver"), "/launch/ur_control.launch.py"
        ]),
        condition=UnlessCondition(use_fake_hardware),
        launch_arguments={
            "ur_type": ur_type,
            "robot_ip": robot_ip,
            "launch_rviz": "false",
        }.items(),
    )

    # 3. UR MoveIt
    ur_moveit = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare("strawberry_moveit_config"), "/launch/ur_moveit.launch.py"
        ]),
        launch_arguments={
            "use_fake_hardware": use_fake_hardware,
            "launch_rviz": launch_rviz,
            "ur_type": ur_type,
        }.items(),
    )

    # 4. Vision Broadcaster (The NEW node)
    vision_node = Node(
        package="harvest_bot",
        executable="vision.py",
        name="vision",
        output="screen",
    )

    # 5. Static Transform Publisher (World -> Base Link)
    static_tf_world = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher_world_base",
        arguments=["0", "0", "0", "0", "0", "0", "world", "base_link"],
        output="screen",
    )

    # 6. Static Transform Publisher (Tool0 -> Camera Link)
    # Corrects Camera Extrinsics: 2cm above (+Y), 8.5cm in front (+Z) of tool flange
    # Rotation: 0 0 0 (Assuming camera aligns with tool frame, or handled by realsense internal)
    static_tf_camera = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher_tool_camera",
        arguments=["0", "0.02", "0.085", "0", "0", "0", "tool0", "camera_link"],
        output="screen",
    )

    # NOTE: The Harvester Planner is NOT launched here because it runs a single cycle and exits.
    # User should run: ros2 run harvest_bot harvester_planner.py manually.

    return LaunchDescription(declared_arguments + [
        realsense,
        ur_driver,
        ur_moveit,
        vision_node,
        static_tf_world,
        static_tf_camera,
    ])
