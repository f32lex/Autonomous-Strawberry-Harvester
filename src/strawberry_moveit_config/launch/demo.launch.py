import os
import yaml
from launch import LaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node
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
    moveit_config_pkg = 'strawberry_moveit_config'
    robot_desc_pkg = 'strawberry_harvesting_bot'

    # 1. Robot Description (URDF)
    urdf_path = os.path.join(get_package_share_directory(robot_desc_pkg), 'urdf', 'ur10_strawberry.urdf.xacro')
    robot_description_content = Command(['xacro ', urdf_path])
    robot_description = {'robot_description': robot_description_content}

    # 2. SRDF
    srdf_path = os.path.join(get_package_share_directory(moveit_config_pkg), 'config', 'ur10_strawberry.srdf')
    with open(srdf_path, 'r') as f:
        semantic_content = f.read()
    robot_description_semantic = {'robot_description_semantic': semantic_content}

    # 3. Config Files
    kinematics_content = load_yaml(moveit_config_pkg, 'config/kinematics.yaml')
    joint_limits_content = load_yaml(moveit_config_pkg, 'config/joint_limits.yaml')
    ompl_planning_content = load_yaml(moveit_config_pkg, 'config/ompl_planning.yaml')
    moveit_controllers = load_yaml(moveit_config_pkg, 'config/moveit_controllers.yaml')
    ros2_controllers_path = os.path.join(get_package_share_directory(moveit_config_pkg), 'config', 'ros2_controllers.yaml')

    # 4. MoveGroup Params
    move_group_params = {}
    move_group_params.update(robot_description)
    move_group_params.update(robot_description_semantic)
    move_group_params.update(kinematics_content)
    move_group_params.update({'joint_limits': joint_limits_content})
    move_group_params.update({'planning_pipelines': ['ompl']})
    move_group_params.update({'ompl': ompl_planning_content})
    move_group_params.update(moveit_controllers)
    move_group_params.update({'use_sim_time': False})
    
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, ros2_controllers_path],
        output="screen",
    )

    # Node: Robot State Publisher
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    # Node: Spawners 
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["ur_manipulator_controller", "--controller-manager", "/controller_manager"],
    )
    
    gripper_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["gripper_controller", "--controller-manager", "/controller_manager"],
    )

    # Node: Move Group
    run_move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[move_group_params],
    )

    # Node: RViz
    rviz_config_file = os.path.join(get_package_share_directory(moveit_config_pkg), 'config', 'demo.rviz')
    if not os.path.exists(rviz_config_file):
        rviz_config_file = "" 

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
            {'ompl': ompl_planning_content}
        ],
    )

    delay_rviz_after_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[rviz_node, run_move_group_node],
        )
    )

    # Node: Static TF
    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        output="log",
        arguments=["0.0", "0.0", "0.0", "0.0", "0.0", "0.0", "world", "base_link"],
    )

    return LaunchDescription([
        static_tf,
        robot_state_publisher,
        control_node,
        joint_state_broadcaster_spawner,
        robot_controller_spawner,
        gripper_controller_spawner,
        delay_rviz_after_spawner,
    ])
