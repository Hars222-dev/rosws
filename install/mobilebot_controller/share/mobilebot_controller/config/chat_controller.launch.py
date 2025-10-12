from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare("robot_description")
    default_controller_config = PathJoinSubstitution(
        [pkg_share, "config", "manipulator_controller.yaml"]
    )

    controller_config_arg = DeclareLaunchArgument(
        "controllers_file",
        default_value=default_controller_config,
        description="YAML file with the controller configuration",
    )

    # Start controller_manager (ros2_control)
    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[LaunchConfiguration("controllers_file")],
        output="screen",
    )

    # Load joint_state_broadcaster
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    # Load arm_controller
    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_controller", "--controller-manager", "/controller_manager"],
    )

    return LaunchDescription([
        controller_config_arg,
        controller_manager,
        joint_state_broadcaster_spawner,
        arm_controller_spawner,
    ])
