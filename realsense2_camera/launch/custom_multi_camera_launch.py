# Copyright 2023 Intel Corporation. All Rights Reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# DESCRIPTION #
# ----------- #
# Use this launch file to launch 2 devices.
# The Parameters available for definition in the command line for each camera are described in rs_launch.configurable_parameters
# For each device, the parameter name was changed to include an index.
# For example: to set camera_name for device1 set parameter camera_name1.
# command line example:
# ros2 launch realsense2_camera rs_multi_camera_launch.py camera_name1:=D400 device_type2:=l5. device_type1:=d4..

"""Launch realsense2_camera node."""
import os
import copy
from ament_index_python.packages import get_package_share_directory
import launch
from launch import LaunchDescription
import launch_ros.actions
from launch.actions import IncludeLaunchDescription, GroupAction
from launch_ros.actions import SetRemap
from launch.substitutions import LaunchConfiguration, ThisLaunchFileDir
from launch.launch_description_sources import PythonLaunchDescriptionSource
import sys
import pathlib
sys.path.append(str(pathlib.Path(__file__).parent.absolute()))
import rs_launch

local_parameters = [
                    #L515 camera
                    {'name': 'camera_name1', 'default': 'L515', 'description': 'camera L515'},
                    {'name': 'device_type1',  'default': 'l515', 'description': 'device L515'},

                    #D405 camera
                    {'name': 'camera_name2', 'default': 'D405', 'description': 'camera l515'},
                    {'name': 'device_type2',  'default': 'd405', 'description': 'device d405'},
                   ]

use_l515_custom = 'false'
use_d405_custom = 'false'

def set_configurable_parameters(local_params):
    return dict([(param['original_name'], LaunchConfiguration(param['name'])) for param in local_params])

def duplicate_params(general_params, posix):
    local_params = copy.deepcopy(general_params)
    for param in local_params:
        param['original_name'] = param['name']
        param['name'] += posix
    return local_params
    

def generate_launch_description():

    params1 = duplicate_params(rs_launch.configurable_parameters, '1')
    params2 = duplicate_params(rs_launch.configurable_parameters, '2')

    l515_custom = LaunchConfiguration('l515_custom', default= use_l515_custom)
    d405_custom = LaunchConfiguration('d405_custom', default= use_d405_custom)

    launch_l515_custom = GroupAction(
        actions=[

            SetRemap(src='/L515/color/camera_info',dst='/L515/color/camera_info_factory'),

            IncludeLaunchDescription(
            PythonLaunchDescriptionSource([ThisLaunchFileDir(), '/rs_launch.py']),
            launch_arguments=set_configurable_parameters(params1).items(),
            ),
            launch_ros.actions.Node(
                    package='realsense2_camera',
                    namespace=LaunchConfiguration("camera_name1"),
                    name=LaunchConfiguration("device_type1"),
                    executable='republish_cam_info',
                    parameters=[os.path.join(
                    get_package_share_directory('realsense2_camera'),
                    'config', 'l515_camera_parameters.yaml')
                        #{"use_sim_time": False},
                        ],
                    output='screen',
                    arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
                    emulate_tty=True,
                    ),
        ],
        condition=launch.conditions.IfCondition(l515_custom),

    )

    launch_d405_custom = GroupAction(
        actions=[

            SetRemap(src='/D405/color/camera_info',dst='/D405/color/camera_info_factory'),

            IncludeLaunchDescription(
            PythonLaunchDescriptionSource([ThisLaunchFileDir(), '/rs_launch.py']),
            launch_arguments=set_configurable_parameters(params2).items(),
            ),
            launch_ros.actions.Node(
                    package='realsense2_camera',
                    namespace=LaunchConfiguration("camera_name2"),
                    name=LaunchConfiguration("device_type2"),
                    executable='republish_cam_info',
                    parameters=[os.path.join(
                    get_package_share_directory('realsense2_camera'),
                    'config', 'd405_camera_parameters.yaml')
                        #{"use_sim_time": False},
                        ],
                    output='screen',
                    arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
                    emulate_tty=True,
                    ),
        ],
        condition=launch.conditions.IfCondition(d405_custom),
    )

    launch_l515 = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([ThisLaunchFileDir(), '/rs_launch.py']),
            launch_arguments=set_configurable_parameters(params1).items(),
            condition=launch.conditions.UnlessCondition(l515_custom),
        )
    
    launch_d405 = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([ThisLaunchFileDir(), '/rs_launch.py']),
            launch_arguments=set_configurable_parameters(params2).items(),
            condition=launch.conditions.UnlessCondition(d405_custom),
        )


    return LaunchDescription(
        rs_launch.declare_configurable_parameters(local_parameters) +
        rs_launch.declare_configurable_parameters(params1) + 
        rs_launch.declare_configurable_parameters(params2) + 
        [
            
        launch_l515,
        launch_l515_custom,
        launch_d405,
        launch_d405_custom,
    ])
