from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    robot = LaunchConfiguration('robotName')
    params_file = LaunchConfiguration('params')

    return LaunchDescription([
        DeclareLaunchArgument('robotName', default_value='mobile_manipulator'),
        DeclareLaunchArgument('params', default_value=os.path.join(
            get_package_share_directory('mpc_cartesian_planner'), 'config', 'tt_params.yaml')),

        Node(
            package='mpc_cartesian_planner',
            executable='trajectory_tt_publisher_node',
            name='trajectory_tt_publisher',
            output='screen',
            parameters=[params_file, {'robotName': robot}],
        ),

        Node(
            package='mpc_cartesian_planner',
            executable='trajectory_progress_monitor_node',
            name='trajectory_progress_monitor',
            output='screen',
            parameters=[params_file, {'robotName': robot}],
        ),
    ])
