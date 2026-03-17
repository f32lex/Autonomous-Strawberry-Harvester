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
    # ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur10 robot_ip:=192.168.11.100 launch_rviz:=false
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
    # ros2 launch strawberry_moveit_config ur_moveit.launch.py ur_type:=ur10 use_fake_hardware:=false launch_rviz:=true
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

    # 4. Perception Node
    vision_node = Node(
        package="harvest_bot",
        executable="vision.py",
        name="vision_node",
        output="screen",
    )

    # 5. Harvester Node
    harvest_node = Node(
        package="harvest_bot",
        executable="harvest.py",
        name="harvester_node",
        output="screen",
    )

    return LaunchDescription(declared_arguments + [
        realsense,
        ur_driver,
        ur_moveit,
        vision_node,
        harvest_node,
    ])
