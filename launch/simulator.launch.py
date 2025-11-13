import os
import xacro
import yaml

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction, ExecuteProcess, SetEnvironmentVariable, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def start_vehicle_controllers():
    """Lag ExecuteProcess for alle ROS2-kontrollere"""
    joint_state = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state',
             'active', 'joint_state_broadcaster'],
        output='screen'
    )
    forward_velocity = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state',
             'active', 'forward_velocity_controller'],
        output='screen'
    )
    forward_position = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state',
             'active', 'forward_position_controller'],
        output='screen'
    )
    return joint_state, forward_velocity, forward_position



def generate_launch_description():
    package_name = "rc_gazebo"
    package_path = get_package_share_directory(package_name)

    gz_bridge_params_path = os.path.join(package_path, 'config',
                                        'ros_gz_bridge.yaml')

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
            'gz_args': [world, ' -r -v 1'],  # verbose level,
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

    # Kontrollere til bilen
    joint_state, forward_velocity, forward_position = start_vehicle_controllers()

    vehicle_controller_node = Node(
        package='rc_gazebo',
        executable='vehicle_controller',
        parameters=[{
            'timer_period': 0.01,          # 100 Hz controller loop
            'use_sim_time': True,
        }],
        output='screen'
    )

    # Node til ROS-Gazebo bridge for å håndtere message passing mellom gazebo og ROS
    gz_bridge_node = Node(package='ros_gz_bridge',
                          executable='parameter_bridge',
                          arguments=['--ros-args', '-p',
                                     f'config_file:={gz_bridge_params_path}'],
                          output='screen')

    
    return LaunchDescription(
        declared_arguments + [
        RegisterEventHandler(
            event_handler=OnProcessExit(target_action=spawn_robot,
                                        on_exit=[joint_state])),
        RegisterEventHandler(
            event_handler=OnProcessExit(target_action=joint_state,
                                        on_exit=[forward_velocity,
                                                 forward_position])),
        models_var,
        gazebo,
        robot_state_publisher,
        spawn_robot,
        vehicle_controller_node,
        gz_bridge_node,
    ])
