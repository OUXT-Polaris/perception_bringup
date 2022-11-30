# Copyright (c) 2020 OUXT Polaris
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

import ament_index_python.packages
import launch
from launch.actions.declare_launch_argument import DeclareLaunchArgument
from launch.substitutions.launch_configuration import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


import os
import yaml

def generate_launch_description():
    container = ComposableNodeContainer(
            name='preception_bringup_container',
            namespace='perception',
            package='rclcpp_components',
            executable='component_container_mt',
            composable_node_descriptions=[
                getVelodyneConvertComponent(),
                getVelodyneDriverComponent()
                #getVelodyneConvertComponent('front_lidar'),
                #getgetVelodyneDriverComponent('rear_lidar')
            ],
            output='screen'
    )
    return launch.LaunchDescription([
        container
        ])



def getVelodyneConvertComponent():
    config_directory = os.path.join(
        ament_index_python.get_package_share_directory('velodyne_pointcloud'))
    
    param_config = os.path.join(config_directory, 'config', 'VLP16-velodyne_convert_node-params.yaml')
    with open(param_config, 'r') as f:
        params = yaml.safe_load(f)['velodyne_convert_node']['ros__parameters']
    params['calibration'] = os.path.join(config_directory, 'params', 'VLP16db.yaml')    
    component = ComposableNode(
        package='velodyne_pointcloud',
        plugin='velodyne_pointcloud::Convert',
        namespace='perception',
        name='velodyne_convert_node',
        parameters=[params])
    return component

def getVelodyneDriverComponent():
    component = ComposableNode(
        package='velodyne_driver',
        plugin='velodyne_driver::VelodyneDriver',
        namespace='/perception',
        name='velodyne_driver_node')
    return component