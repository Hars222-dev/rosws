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

    xacro_path = os.path.join(
        get_package_share_directory('mobilebot'),
        'urdf',
        'robot.urdf.xacro'
    )

    # Process the Xacro file into URDF
    robot_desc = xacro.process_file(xacro_path).toxml()

    lidar_node = Node(
        package='ros_gz_bridge',
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

    # Launch Gazebo Sim with default world
    gazebo = ExecuteProcess(
        cmd=[
        'ign', 'gazebo', '-r', '-v', '4',
        os.path.join(
            get_package_share_directory('mobilebot'),
            'worlds',
            'my_world.world'
        )
    ],
    output='screen',
    additional_env={
        'LD_LIBRARY_PATH': environ['LD_LIBRARY_PATH'],
        'GZ_SIM_RESOURCE_PATH': os.path.expanduser('~/rosws/src/meshes')
    }   
)


    # Spawn robot entity in Gazebo (with a delay to ensure Gazebo is ready)
    spawn_entity = TimerAction(
        period=5.0,  # wait 5 seconds before spawning
        actions=[
            Node(
                package='ros_gz_sim',
                executable='create',
                arguments=[
                    '-topic', '/robot_description',
                    '-name', 'my_robot',
                    '-x', '0', '-y', '0', '-z', '0.1'
                ],
                output='screen'
            )
        ]
    )

    joint_state_broadcaster_spawner = Node(
        package = "controller_manager",
        executable = "spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager"
        ]
    )
    arm_controller_spawner = Node(
        package = "controller_manager",
        executable = "spawner",
        arguments=[
            "arm_controller",
            "--controller-manager",
            "/controller_manager"
        ]
    )
    wheel_controller_spawner = Node(
        package = "controller_manager",
        executable = "spawner",
        arguments=[
            "wheel_controller",
            "--controller-manager",
            "/controller_manager"
        ]
    )

    return LaunchDescription([
        rsp_node,
        lidar_node,
        gazebo,
        spawn_entity,
        joint_state_broadcaster_spawner,
        arm_controller_spawner,
        wheel_controller_spawner
    ])
