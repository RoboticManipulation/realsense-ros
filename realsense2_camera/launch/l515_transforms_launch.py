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
            # nils
            arguments=['-0.005', '0.1370', '0.00548', '0.0148379', '-0.1494206', '0.9886599', '-0.0021957', 'wrist_3_link', 'camera_color_optical_frame'],
            # old
            # arguments=['0.0060', '0.0900', '0.04', '-0.018', '-0.148', '0.987', '-0.053', 'wrist_3_link', 'camera_color_optical_frame'],
            # don't know
            # arguments=['0.0060', '0.0900', '0.04', '-3.0313102', '-0.0198611', '0.2987771', 'wrist_3_link', 'camera_color_optical_frame'],
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
