import os
import xacro
import yaml

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import SetEnvironmentVariable, IncludeLaunchDescription, DeclareLaunchArgument, RegisterEventHandler, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import LaunchConfiguration
from launch.event_handlers import OnProcessExit

def load_robot_description(xacro_path, params_path):
    """Last Xacro + YAML-parametre og generer URDF"""
    with open(params_path, 'r') as f:
        params = yaml.safe_load(f)['/**']['ros__parameters']
    return xacro.process_file(
        xacro_path,
        mappings={k: str(v) for k, v in params.items()}
    ).toxml()


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

    """
         _______________________
        |                       |
        |   PATH defines START  |
        |_______________________|
    """

    #Path til pakken
    package_name = "rc_gazebo"
    package_path = get_package_share_directory(package_name)

    # Paths til Xacro og parametre
    robot_xacro = os.path.join(package_path, 'models', 'vehicle.xacro')
    vehicle_params = os.path.join(package_path, 'config', 'parameters.yaml')
    robot_description = load_robot_description(robot_xacro, vehicle_params)

    # Path til default world
    default_world_path = os.path.join(package_path, 'worlds', 'world_demo.sdf')
    default_world_path = os.path.join(package_path, 'worlds', 'maze.sdf')
    
    # Path til gazebo launch
    gz_launch_path = os.path.join(
        get_package_share_directory('ros_gz_sim'),
                                    'launch', 
                                    'gz_sim.launch.py')
    
    #Path til bridge params
    gz_bridge_params_path = os.path.join(package_path, 'config',
                                        'ros_gz_bridge.yaml')

    # Setter miljøvareabler
    models_var = SetEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        PathJoinSubstitution([package_path, 'models'])
    )

    """
         _______________________
        |                       |
        |   PATH defines SLUTT  |
        |_______________________|
    """


    """
         _______________________
        |                       |
        |   ARG defines START   |
        |_______________________|
    """

    # Define a launch argument for the world file, defaulting to "world_demo.sdf"
    world_arg = DeclareLaunchArgument('world', default_value=default_world_path)
    # Launch-argumenter for initial pose
    x_arg = DeclareLaunchArgument('x', default_value='0.0')
    y_arg = DeclareLaunchArgument('y', default_value='0.0')
    z_arg = DeclareLaunchArgument('z', default_value='0.1')
    R_arg = DeclareLaunchArgument('R', default_value='0.0')
    P_arg = DeclareLaunchArgument('P', default_value='0.0')
    Y_arg = DeclareLaunchArgument('Y', default_value='0.0')

    world_file = LaunchConfiguration('world')
    x = LaunchConfiguration('x')
    y = LaunchConfiguration('y')
    z = LaunchConfiguration('z')
    R = LaunchConfiguration('R')
    P = LaunchConfiguration('P')
    Y = LaunchConfiguration('Y')

    """
         _______________________
        |                       |
        |   ARG defines SLUTT   |
        |_______________________|
    """


    """
         _______________________
        |                       |
        |   NODES defines START |
        |_______________________|
    """

    # --- Robot state publisher ---
    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}],
        output='screen'
    )

    # --- Spawn robot i Gazebo ---
    spawn_vehicle_gazebo_node = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'ackermann_steering_vehicle',
            '-string', robot_description,
            '-x', x, '-y', y, '-z', z,
            '-R', R, '-P', P, '-Y', Y,
            '-allow_renaming', 'true'
        ],
        output='screen'
    )

    # --- Controller nodes ---
    joint_state, forward_velocity, forward_position = start_vehicle_controllers()

    vehicle_controller_node = Node(
        package='gazebo_ackermann_steering_vehicle',
        executable='vehicle_controller',
        parameters=[vehicle_params, {
            'timer_period': 0.01,          # 10 ms loop
            'timeout_duration': 0.02        # 20 ms timeout
        }],
        output='screen'
    )

    # Starter gazebo
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gz_launch_path),
        launch_arguments={'gz_args': [f'-r -v 4 ', world_file],
                          'on_exit_shutdown': 'true'}.items(),
    )

    # Node til ROS-Gazebo bridge for å håndtere message passing mellom gazebo og ROS
    gz_bridge_node = Node(package='ros_gz_bridge',
                          executable='parameter_bridge',
                          arguments=['--ros-args', '-p',
                                     f'config_file:={gz_bridge_params_path}'],
                          output='screen')
    
    """
         _______________________
        |                       |
        |   NODES defines SLUTT |
        |_______________________|
    """
 

    launch_description = LaunchDescription([
        models_var,
        RegisterEventHandler(
            event_handler=OnProcessExit(target_action=spawn_vehicle_gazebo_node,
                                        on_exit=[joint_state])),
        RegisterEventHandler(
            event_handler=OnProcessExit(target_action=joint_state,
                                        on_exit=[forward_velocity,
                                                 forward_position])),
        world_arg,
        gazebo_launch,
        x_arg,
        y_arg,
        z_arg,
        R_arg,
        P_arg,
        Y_arg,
        spawn_vehicle_gazebo_node,
        rsp_node,
        vehicle_controller_node,
        gz_bridge_node])
    

    return launch_description