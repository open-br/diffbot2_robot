#!/usr/bin/env python3
#
# Copyright 2020 Open BR
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

import os

from ament_index_python import get_package_share_directory
import launch
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro


def generate_launch_description():
    # Get dir names
    pkg_share = get_package_share_directory('diffbot2_description')

    # Compute robot_description string
    xacro_file = os.path.join(pkg_share, 'urdf/diffbot2.xacro')
    robot_description = xacro.process(xacro_file)

    # Launch description
    launch_description = LaunchDescription()

    # Define parameters
    common_params = {
        'robot_description': robot_description,
    }

    # Add nodes
    launch_description.add_action(
        launch.actions.DeclareLaunchArgument(
            name='namespace', default_value='', description='Node namespace'
        )
    )
    launch_description.add_action(
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            namespace=LaunchConfiguration('namespace'),
            parameters=[common_params],
        )
    )
    launch_description.add_action(
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            output='screen',
            namespace=LaunchConfiguration('namespace'),
            parameters=[common_params],
        ),
    )

    return launch_description
