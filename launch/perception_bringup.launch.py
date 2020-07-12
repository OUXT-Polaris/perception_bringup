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

import launch
from launch_ros.actions import ComposableNodeContainer
from launch_ros.actions import Node
from launch_ros.descriptions import ComposableNode

import os

import ament_index_python.packages

import yaml

def generate_launch_description():
    container = ComposableNodeContainer(
            node_name='preception_bringup_container',
            node_namespace='perception',
            package='rclcpp_components',
            node_executable='component_container_mt',
            composable_node_descriptions=[
                getImageDecompressorComponent('front_camera')
            ],
            output='screen',
    )
    return launch.LaunchDescription([container])


def getImageDecompressorComponent(camera_name):
    config_directory = os.path.join(
        ament_index_python.packages.get_package_share_directory('perception_bringup'),
        'config')
    param_config = os.path.join(config_directory, camera_name+'_image_decompressor.yaml')
    with open(param_config, 'r') as f:
        params = yaml.safe_load(f)[camera_name+'_image_decompressor_node']['ros__parameters']
    component = ComposableNode(
        package='image_processing_utils',
        node_plugin='image_processing_utils::ImageDecompressorComponent',
        node_namespace='/'+camera_name,
        node_name='image_decompressor_node',
        parameters=[params])
    return component