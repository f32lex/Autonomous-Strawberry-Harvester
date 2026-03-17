import os
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import Command

def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    try:
        with open(absolute_file_path, 'r') as file:
            return yaml.safe_load(file)
    except EnvironmentError:
        return None

def generate_launch_description():
    # Declare arguments
    ur_type_arg = DeclareLaunchArgument(
        'ur_type', default_value='ur10', description='Type of UR robot.'
    )
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='false', description='Use simulation (Gazebo) clock if true'
    )
    launch_rviz_arg = DeclareLaunchArgument(
        'launch_rviz', default_value='true', description='Launch RViz?'
    )

    use_fake_hardware_arg = DeclareLaunchArgument(
        'use_fake_hardware', default_value='true', description='Use fake hardware?'
    )
    fake_sensor_commands_arg = DeclareLaunchArgument(
        'fake_sensor_commands', default_value='true', description='Fake sensor commands?'
    )

    use_sim_time = LaunchConfiguration('use_sim_time')
    launch_rviz = LaunchConfiguration('launch_rviz')
    use_fake_hardware = LaunchConfiguration('use_fake_hardware')
    fake_sensor_commands = LaunchConfiguration('fake_sensor_commands')

    moveit_config_pkg = 'strawberry_moveit_config'
    robot_desc_pkg = 'strawberry_harvesting_bot'

    # Robot Description
    urdf_path = os.path.join(get_package_share_directory(robot_desc_pkg), 'urdf', 'ur10_strawberry.urdf.xacro')
    robot_description_content = Command([
        'xacro ', urdf_path,
        ' use_fake_hardware:=', use_fake_hardware,
        ' fake_sensor_commands:=', fake_sensor_commands
    ])
    robot_description = {'robot_description': robot_description_content}

    # SRDF
    srdf_path = os.path.join(get_package_share_directory(moveit_config_pkg), 'config', 'ur10_strawberry.srdf')
    with open(srdf_path, 'r') as f:
        semantic_content = f.read()
    robot_description_semantic = {'robot_description_semantic': semantic_content}

    # Config Files
    kinematics_content = load_yaml(moveit_config_pkg, 'config/kinematics.yaml')
    joint_limits_content = load_yaml(moveit_config_pkg, 'config/joint_limits.yaml')
    ompl_planning_content = load_yaml(moveit_config_pkg, 'config/ompl_planning.yaml')
    moveit_controllers = load_yaml(moveit_config_pkg, 'config/moveit_controllers.yaml')
    
    # MoveGroup Params
    move_group_params = {}
    move_group_params.update(robot_description)
    move_group_params.update(robot_description_semantic)
    move_group_params.update(kinematics_content)
    move_group_params.update({'joint_limits': joint_limits_content})
    move_group_params.update({'planning_pipelines': ['ompl']})
    move_group_params.update({'ompl': ompl_planning_content})
    move_group_params.update(moveit_controllers)
    move_group_params.update({'use_sim_time': use_sim_time})
    
    # Node: Move Group
    run_move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[move_group_params],
    )

    # Node: RViz
    rviz_config_file = os.path.join(get_package_share_directory(moveit_config_pkg), 'config', 'demo.rviz')
    
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[
            robot_description,
            robot_description_semantic,
            kinematics_content,
            {'ompl': ompl_planning_content},
            {'use_sim_time': use_sim_time}
        ],
        condition=IfCondition(launch_rviz)
    )

    # Node: Static TF (World to Base Link)
    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        output="log",
        arguments=["0.0", "0.0", "0.0", "0.0", "0.0", "0.0", "world", "base_link"],
    )

    # Publish Calibration
    publish_calibration = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare("strawberry_harvesting_bot"), "/launch/publish_calibration.launch.py"
        ]),
    )

    return LaunchDescription([
        ur_type_arg,
        use_sim_time_arg,
        launch_rviz_arg,
        use_fake_hardware_arg,
        fake_sensor_commands_arg,
        static_tf,
        publish_calibration,
        run_move_group_node,
        rviz_node,
    ])
