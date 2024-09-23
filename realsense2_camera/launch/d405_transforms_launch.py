from launch import LaunchDescription
# from launch.actions import (
#     DeclareLaunchArgument,
#     IncludeLaunchDescription,
#     OpaqueFunction,
#     RegisterEventHandler,
# )
from launch.actions import (
    DeclareLaunchArgument,
    OpaqueFunction,
)
# from launch.conditions import IfCondition, UnlessCondition
# from launch.event_handlers import OnProcessExit
# from launch.launch_description_sources import PythonLaunchDescriptionSource
# from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
# from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):
    # Simulation arguments
    use_sim_time = LaunchConfiguration("use_sim_time")
    
    cam_in_hand_transform_pub = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_transform_publisher',
            output="screen",
            #arguments=['0.012', '0.083', '0.034', '0.005', '-0.140', '0.990', '-0.001', 'wrist_3_link', 'camera_color_optical_frame'], # Tobias
            #arguments=['0.012', '0.085', '0.034', '0.005', '-0.140', '0.990', '-0.001', 'wrist_3_link', 'camera_color_optical_frame'], # Tobias adjusted y
            arguments=['0.012', '0.085', '0.035', '0.005', '-0.149', '0.989', '-0.001', 'wrist_3_link', 'camera_color_optical_frame'], #Adjusted roll
            # 0.012, 0.100, 0.050 0.005, -0.149, 0.989, -0.001 #Adjusted z and roll
            parameters=[{"use_sim_time": use_sim_time}],
        )

    nodes_to_start = [
        cam_in_hand_transform_pub,
    ]

    return nodes_to_start


def generate_launch_description():
    declared_arguments = []

    declared_arguments.append(
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Use simulation time.",
        )
    )

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])