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
    
    camera_base_link_pub = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_transform_publisher',
            output="screen",
            arguments=['0.128610','-0.000620291','0.025456', '0.0923', '-0.0923', '-0.7011', '0.7011', 'mount_reference', 'camera_link'], # Guess for UR3e Baselink - Using the Orignal Mesh of 3D Mount Print 
            parameters=[{"use_sim_time": use_sim_time}],
        )
    
    # +90 degree to YAW of the Original Estimate 
    # camera_base_link_pub = Node(
    #         package='tf2_ros',
    #         executable='static_transform_publisher',
    #         name='static_transform_publisher',
    #         output="screen",
    #         arguments=['0.128610','-0.000620291','0.025456', '0.1305', '0.0','0.0', '0.9914', 'mount_reference', 'camera_color_base_link'], # Guess for UR3e Baselink - Using the Orignal Mesh of 3D Mount Print 
    #         parameters=[{"use_sim_time": use_sim_time}],
    #     )
    
    
    # -90 degree to YAW of the Original Estimate 
    # camera_base_link_pub = Node(
    #         package='tf2_ros',
    #         executable='static_transform_publisher',
    #         name='static_transform_publisher',
    #         output="screen",
    #         arguments=['0.128610','-0.000620291','0.025456', '0', '-0.1305','-0.9914', '-0.00006', 'mount_reference', 'camera_color_base_link'], # Guess for UR3e Baselink - Using the Orignal Mesh of 3D Mount Print 
    #         parameters=[{"use_sim_time": use_sim_time}],
    #     )
    
    #  # +180 degree to YAW of the Original Estimate 
    # camera_base_link_pub = Node(
    #         package='tf2_ros',
    #         executable='static_transform_publisher',
    #         name='static_transform_publisher',
    #         output="screen",
    #         arguments=['0.128610','-0.000620291','0.025456', '0.0922', '0.0922', '0.7011', '0.7011', 'mount_reference', 'camera_color_base_link'], # Guess for UR3e Baselink - Using the Orignal Mesh of 3D Mount Print 
    #         parameters=[{"use_sim_time": use_sim_time}],
    #     )
    
    # -180 degree to YAW of the Original Estimate 
    # camera_base_link_pub = Node(
    #         package='tf2_ros',
    #         executable='static_transform_publisher',
    #         name='static_transform_publisher',
    #         output="screen",
    #         arguments=['0.128610','-0.000620291','0.025456', '-0.0922', '-0.0922', '-0.7011', '-0.7011', 'mount_reference', 'camera_color_base_link'], # Guess for UR3e Baselink - Using the Orignal Mesh of 3D Mount Print 
    #         parameters=[{"use_sim_time": use_sim_time}],
    #     )
    
    # cam_in_hand_transform_pub = Node(
    #         package='tf2_ros',
    #         executable='static_transform_publisher',
    #         name='static_transform_publisher',
    #         output="screen",
    #         #arguments=['0.012', '0.083', '0.034', '0.005', '-0.140', '0.990', '-0.001', 'wrist_3_link', 'camera_color_optical_frame'], # Tobias
    #         #arguments=['0.012', '0.085', '0.034', '0.005', '-0.140', '0.990', '-0.001', 'wrist_3_link', 'camera_color_optical_frame'], # Tobias adjusted y
    #         # arguments=['0.012', '0.085', '0.035', '0.005', '-0.149', '0.989', '-0.001', 'wrist_3_link', 'camera_color_optical_frame'], #Adjusted roll - Calibration for UR5
    #         # arguments=['0.140708', '-0.001375882', '0.045030005', '-0.126', '0.034', '0.956', '-0.262', 'mount_reference', 'camera_color_optical_frame'], # Guess for UR3e - Original1
    #         # arguments=['0.140708', '-0.001375882', '0.045030005', '-0.113159', '-0.065066', '0.861423', '0.490827', 'mount_reference', 'camera_color_optical_frame'], # Guess for UR3e - Original2
    #          arguments=['0', '0', '0.023', '0', '0', '0', '1', 'camera_color_base_link', 'camera_color_optical_frame'], # Z set to 23mm, all other transforms removed
    #         # 0.012, 0.100, 0.050 0.005, -0.149, 0.989, -0.001 #Adjusted z and roll
    #         parameters=[{"use_sim_time": use_sim_time}],
    #     )

    nodes_to_start = [
        camera_base_link_pub,
        # cam_in_hand_transform_pub,
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