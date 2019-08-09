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

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    parameters_file_path = os.path.join(
        get_package_share_directory('cloudwatch_logger'), 'config', 'sample_configuration.yaml')
    return LaunchDescription([
        Node(package='cloudwatch_logger',
             node_executable='cloudwatch_logger',
             parameters=[parameters_file_path],
             # workaround until https://github.com/ros2/rmw_fastrtps/issues/265 is resolved
             arguments=["__log_disable_rosout:=true"],
             output='screen'),
])
