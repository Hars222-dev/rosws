from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction, DeclareLaunchArgument, SetEnvironmentVariable
from launch_ros.actions import Node
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
import os
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory, get_package_prefix
import xacro

def generate_launch_description():

    # Path to your xacro file
    xacro_path = os.path.join(
        get_package_share_directory('mobilebot'),
        'urdf',
        'robot.urdf.xacro'
    )

    # Convert Xacro to URDF
    robot_desc = xacro.process_file(xacro_path).toxml()

    # robot_state_publisher node
    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_desc}],
        output='screen'
    )

    # joint_state_publisher node (no GUI)
    joint_state_pub_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen'
    )

    # Path to the RViz config
    rviz_config_path = os.path.join(
        get_package_share_directory('mobilebot'),
        'rviz',
        'view_robot.rviz'
    )

    # rviz2 node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_path],
        output='screen'
    )

    return LaunchDescription([
        joint_state_pub_node,
        rsp_node,
        rviz_node
    ])
