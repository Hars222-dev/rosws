from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction, DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
import os
from ament_index_python import get_package_share_directory
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():

    model_arg = DeclareLaunchArgument(
        name="model",
        default_value=os.path.join(get_package_share_directory("robot_description"),"urdf","robot.urdf.xacro"),
        description="path to urdf file"
    )
    robot_description = ParameterValue(Command(["xacro",LaunchConfiguration("model")]))
    
    # Robot State Publisher node
    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}],
        output='screen'
    )

    # Launch Gazebo Sim with default world
    gazebo = ExecuteProcess(
        cmd=['ign', 'gazebo', '-r', '-v', '4', 'empty.sdf'],
        output='screen'
    )

    # Spawn robot entity in Gazebo (with a delay to ensure Gazebo is ready)
    spawn_entity = TimerAction(
        period=5.0,  # wait 5 seconds before spawning
        actions=[
            Node(
                package='ros_gz_sim',
                executable='create',
                arguments=[
                    '-topic', 'robot_description',
                    '-name', 'my_robot',
                    '-x', '0', '-y', '0', '-z', '0.1'
                ],
                output='screen'
            )
        ]
    )

    return LaunchDescription([
        model_arg,
        rsp_node,
        gazebo,
        spawn_entity
    ])
