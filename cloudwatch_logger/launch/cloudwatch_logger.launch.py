# Copyright 2019 Amazon.com, Inc. or its affiliates. All Rights Reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License").
# You may not use this file except in compliance with the License.
# A copy of the License is located at
#
#  http://aws.amazon.com/apache2.0
#
# or in the "license" file accompanying this file. This file is distributed
# on an "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either
# express or implied. See the License for the specific language governing
# permissions and limitations under the License.

"""Launch a lifecycle cloudwatch_logger node"""

import os
import yaml

import launch
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    default_config = os.path.join(get_package_share_directory('cloudwatch_logger'), 'config', 'sample_configuration.yaml')
    with open(default_config, 'r') as f:
        config_text = f.read()
    config_yaml = yaml.safe_load(config_text)
    default_log_group_name = config_yaml['cloudwatch_logger']['ros__parameters']['log_group_name']
    default_aws_region = config_yaml['cloudwatch_logger']['ros__parameters']['aws_client_configuration']['region']

    parameters = [launch.substitutions.LaunchConfiguration("config_file")]
    parameters.append({"log_group_name": launch.substitutions.LaunchConfiguration("log_group_name")})
    parameters.append({"aws_client_configuration": { "region": launch.substitutions.LaunchConfiguration("aws_region") }})
  
    return LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            "node_name",
            default_value="cloudwatch_logger",
        ),
        launch.actions.DeclareLaunchArgument(
            "config_file",
            default_value=default_config
        ),
        launch.actions.DeclareLaunchArgument(
            "aws_region",
            default_value=default_aws_region
        ),
        launch.actions.DeclareLaunchArgument(
            "log_group_name",
            default_value=default_log_group_name
        ),
        Node(
            package='cloudwatch_logger',
            node_executable='cloudwatch_logger',
            node_name=launch.substitutions.LaunchConfiguration('node_name'),
            parameters=parameters,
            # workaround until https://github.com/ros2/rmw_fastrtps/issues/265 is resolved
            arguments=["__log_disable_rosout:=true"],
            output='screen'
        ),
    ])
