import os
import xacro
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction, ExecuteProcess, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    package_name = "rc_gazebo"
    package_path = get_package_share_directory(package_name)

    # --- Launch arguments ---
    declared_arguments = [
        DeclareLaunchArgument(
            'world',
            default_value=PathJoinSubstitution([package_path, 'worlds', 'maze.sdf']),
            description='Path to the Gazebo world file'
        ),
        DeclareLaunchArgument(
            'xacro_file',
            default_value=PathJoinSubstitution([package_path, 'urdf', 'mentorpi.xacro']),
            description='Path to the xacro file'
        ),
        DeclareLaunchArgument(
            'gui',
            default_value='true',
            description='Whether to open Gazebo GUI'
        ),
    ]

    world = LaunchConfiguration('world')
    xacro_file = LaunchConfiguration('xacro_file')
    gui = LaunchConfiguration('gui')

    # --- Convert Xacro to URDF at launch time ---
    robot_description = xacro.process_file(os.path.join(package_path, 'urdf', 'mentorpi.xacro')).toxml()

    models_var = SetEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        os.path.join(package_path, 'models')
    )

    # --- Start Gazebo (ros_gz_sim) ---
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                get_package_share_directory('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py'
            ])
        ]),
        launch_arguments={
            'gz_args': [world, ' -r -v 4'],  # verbose level,
            'on_exit_shutdown': 'true'
        }.items(),
    )

    # --- Publish robot_description ---
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}],
        output='screen'
    )

    # --- Spawn robot into Gazebo ---
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'my_robot', 
            '-string', robot_description,
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.1'
        ],
        output='screen'
    )

    delayed_spawn = TimerAction(period=2.0, actions=[spawn_robot])    

    return LaunchDescription(
        declared_arguments + [
        models_var,
        gazebo,
        robot_state_publisher,
        delayed_spawn,
    ])
