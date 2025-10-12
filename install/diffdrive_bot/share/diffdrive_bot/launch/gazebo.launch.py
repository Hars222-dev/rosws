from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction, SetEnvironmentVariable
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
import xacro

def generate_launch_description():

    # Get package path
    pkg_share = get_package_share_directory('diffdrive_bot')
    xacro_path = os.path.join(pkg_share, 'urdf', 'diff_bot.urdf.xacro')
    controller_yaml = os.path.join(pkg_share, 'config', 'diffdrive_bot_controller.yaml')

    # Process Xacro to URDF
    robot_desc = xacro.process_file(xacro_path).toxml()

    # Set Ignition Gazebo resource path
    set_env_var = SetEnvironmentVariable(
        name='IGN_GAZEBO_RESOURCE_PATH',
        value=f"{pkg_share}:{os.environ.get('IGN_GAZEBO_RESOURCE_PATH', '')}"
    )

    # Launch Gazebo
    gazebo = ExecuteProcess(
        cmd=['ign', 'gazebo', '-r', '-v', '4', 'empty.sdf'],
        output='screen'
    )

    # Reset the world to remove old robots
    reset_world = ExecuteProcess(
        cmd=['ign', 'world', '-r'],
        output='screen'
    )

    # Spawn the diff-drive robot after a short delay
    spawn_entity = TimerAction(
        period=6.0,  # wait for Gazebo to fully initialize
        actions=[
            Node(
                package='ros_ign_gazebo',
                executable='create',
                arguments=[
                    '-topic', '/robot_description',
                    '-name', 'diffdrive_bot',
                    '-x', '0', '-y', '0', '-z', '0.5'
                ],
                output='screen'
            )
        ]
    )

    # Robot state publisher
    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_desc}],
        output='screen'
    )

    # ros2_control node
    ros2_control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[{'robot_description': robot_desc}, controller_yaml],
        output='screen'
    )

    # Joint state broadcaster spawner
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        output='screen'
    )

    # Diff drive controller spawner
    diff_drive_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['diff_drive_controller', '--controller-manager', '/controller_manager'],
        output='screen'
    )

    # LaunchDescription
    return LaunchDescription([
        set_env_var,
        rsp_node,
        gazebo,
        reset_world,
        spawn_entity,
        ros2_control_node,
        joint_state_broadcaster_spawner,
        diff_drive_controller_spawner
    ])
