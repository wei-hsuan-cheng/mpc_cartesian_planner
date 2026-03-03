from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    tt_params = LaunchConfiguration('tt_params')

    declared_arguments = [
        DeclareLaunchArgument('robotName', default_value='mobile_manipulator'),
        DeclareLaunchArgument('tt_params', default_value=os.path.join(
            get_package_share_directory('mpc_cartesian_planner'), 'config', 'tt_params.yaml')),
        DeclareLaunchArgument("taskFile", default_value=""),
        DeclareLaunchArgument("libFolder", default_value=""),
        DeclareLaunchArgument("urdfFile", default_value=""),
        DeclareLaunchArgument("globalFrame", default_value=""),
        DeclareLaunchArgument("baseCmdTopic", default_value="/cmd_vel"),
        DeclareLaunchArgument("publishZeroBaseCmdOnIntervention", default_value="true"),
        DeclareLaunchArgument("zeroBaseCmdBurstCount", default_value="10"),
        DeclareLaunchArgument("zeroBaseCmdBurstRate", default_value="50.0"),
        DeclareLaunchArgument("activateMpcControllerOnGoal", default_value="false"),
    ]

    tt_action = Node(
        package="mpc_cartesian_planner",
        executable="trajectory_tt_action_server_node",
        name="trajectory_tt_action_server",
        output="screen",
        parameters=[tt_params,
                    {
                     "robotName": LaunchConfiguration("robotName"),
                     "taskFile": LaunchConfiguration("taskFile"),
                     "libFolder": LaunchConfiguration("libFolder"),
                     "urdfFile": LaunchConfiguration("urdfFile"),
                     "trajectoryGlobalFrame": LaunchConfiguration("globalFrame"),
                     "baseCmdTopic": LaunchConfiguration("baseCmdTopic"),
                     "publishZeroBaseCmdOnIntervention": ParameterValue(LaunchConfiguration("publishZeroBaseCmdOnIntervention"), value_type=bool),
                     "zeroBaseCmdBurstCount": ParameterValue(LaunchConfiguration("zeroBaseCmdBurstCount"), value_type=int),
                     "zeroBaseCmdBurstRate": ParameterValue(LaunchConfiguration("zeroBaseCmdBurstRate"), value_type=float),
                     "activateMpcControllerOnGoal": ParameterValue(LaunchConfiguration("activateMpcControllerOnGoal"), value_type=bool),
                    },
                    ],
    )

    progress_monitor = Node(
        package="mpc_cartesian_planner",
        executable="trajectory_progress_monitor_node",
        name="trajectory_progress_monitor",
        output="screen",
        parameters=[tt_params,
                    {
                     "robotName": LaunchConfiguration("robotName"),
                     "taskFile": LaunchConfiguration("taskFile"),
                     "libFolder": LaunchConfiguration("libFolder"),
                     "urdfFile": LaunchConfiguration("urdfFile"),
                     "trajectoryGlobalFrame": LaunchConfiguration("globalFrame"),
                    },
                    ],
    )

    return LaunchDescription(declared_arguments + [tt_action, progress_monitor])
