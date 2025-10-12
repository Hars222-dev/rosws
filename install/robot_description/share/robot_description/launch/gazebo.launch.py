from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction, DeclareLaunchArgument, SetEnvironmentVariable
from launch_ros.actions import Node
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
import os
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory, get_package_prefix
import xacro
from os import environ


def generate_launch_description():

    # Get package directory
    pkg_share = get_package_share_directory('robot_description')
    
    xacro_path = os.path.join(pkg_share, 'urdf', 'robot.urdf.xacro')

    # Process the Xacro file into URDF with proper substitution
    doc = xacro.process_file(xacro_path, mappings={'package_path': pkg_share})
    robot_desc = doc.toxml()

    lidar_node = Node(
        package='ros_ign_bridge',
        executable='parameter_bridge',
        arguments=[
            '/lidar_scan@sensor_msgs/msg/LaserScan@ignition.msgs.LaserScan'
        ],
        remappings=[
            ('/lidar_scan', '/scan')
        ],
        output='screen'
    )

    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_desc}],
        output='screen'
    )

    # Set environment variable for Ignition Gazebo
    set_env_var = SetEnvironmentVariable(
        name='IGN_GAZEBO_RESOURCE_PATH',
        value=f"{pkg_share}:{os.environ.get('IGN_GAZEBO_RESOURCE_PATH', '')}"
    )

    # Launch Gazebo Sim
    gazebo = ExecuteProcess(
        cmd=['ign', 'gazebo', '-r', '-v', '4', 'empty.sdf'],
        output='screen'
    )

    # Spawn robot entity in Gazebo
    spawn_entity = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='ros_ign_gazebo',
                executable='create',
                arguments=[
                    '-topic', '/robot_description',
                    '-name', 'my_robot',
                    '-x', '0', '-y', '0', '-z', '5'
                ],
                output='screen'
            )
        ]
    )

      

    return LaunchDescription([
        set_env_var,
        rsp_node,
        lidar_node,
        gazebo,
        spawn_entity,
       
    ])