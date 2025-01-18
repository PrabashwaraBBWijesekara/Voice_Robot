import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

def generate_launch_description():
    voice_robot_pkg_dir = get_package_share_directory('voice_control_robot_package')
    

    # World file
    world = os.path.join(voice_robot_pkg_dir, "worlds", "empty.world")


    # Launch Gazebo with empty world
    start_gazebo_cmd = ExecuteProcess(
        cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_factory.so', world],
        output='screen'
    )


    # Launch description
    return LaunchDescription([
        start_gazebo_cmd,
    ])
