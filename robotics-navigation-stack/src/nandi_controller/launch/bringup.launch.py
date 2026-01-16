from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    robot_description_package = 'nandi_controller'

    # --- URDF path ---
    urdf_file = PathJoinSubstitution([
        FindPackageShare(robot_description_package),
        'urdf',
        'nandi.urdf.xacro'
    ])

    # --- Process xacro ---
    robot_description_content = Command(['xacro', ' ', urdf_file])
    robot_description = {'robot_description': robot_description_content}

    # --- Controller YAML ---
    config_path = os.path.join(
        get_package_share_directory('nandi_controller'),
        'config',
        'diff_drive_controller_params.yaml'
    )
    
    robot_controllers = PathJoinSubstitution([
        FindPackageShare('nandi_controller'),
        'config',
        'diff_drive_controller_params.yaml'
    ])

    # --- Controller Manager ---
    controller_manager_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            robot_description,
            {'use_sim_time': use_sim_time},
            # {'use_stamped_vel': 'false'},
            robot_controllers
        ],
        output='screen'
    )

    # --- Robot State Publisher ---
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[robot_description, {'use_sim_time': use_sim_time}],
        output='screen'
    )

    # --- Controller Spawners ---
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        output='screen'
    )

    diff_drive_spawner = Node(
    package='controller_manager',
    executable='spawner',
    arguments=[
        'mobile_base_controller',
        '--controller-manager', '/controller_manager',
        '--param-file', robot_controllers,
        "--controller-ros-args",
        "-r /mobile_base_controller/cmd_vel:=/cmd_vel",
    ],
    output='screen'
)

    # --- Timing ---
    delayed_controller_manager = TimerAction(period=1.0, actions=[controller_manager_node])
    delayed_jsb = TimerAction(period=4.0, actions=[joint_state_broadcaster_spawner])
    delayed_diff = TimerAction(period=6.0, actions=[diff_drive_spawner])

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        robot_state_publisher_node,
        delayed_controller_manager,
        delayed_jsb,
        delayed_diff,
    ])
