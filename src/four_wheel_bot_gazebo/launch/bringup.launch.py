from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from launch.substitutions import Command
import os
from ament_index_python.packages import get_package_share_directory
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    pkg_share = get_package_share_directory('four_wheel_bot_description')
    urdf_path = os.path.join(pkg_share, 'urdf', 'four_wheel_bot.urdf.xacro')

    # Process xacro file to generate URDF
    robot_description = ParameterValue(Command(['xacro ', urdf_path]), value_type=str)

    # Publish robot_description (xacro processed) using robot_state_publisher node
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}]
    )

    # Joint state publisher (optional - for manual joint control)
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen'
    )

    # Launch gazebo classic
    gazebo_cmd = ExecuteProcess(
        cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_factory.so'],
        output='screen'
    )

    # Spawn URDF into gazebo using gazebo_ros spawn_entity
    spawn_cmd = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'four_wheel_bot'],
        output='screen'
    )

    return LaunchDescription([
        gazebo_cmd,
        robot_state_publisher,
        joint_state_publisher,
        spawn_cmd
    ])