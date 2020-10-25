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


import ament_index_python
import launch
import launch_ros
import xacro


def generate_launch_description():
    pkg_share = ament_index_python.get_package_share_directory(
        'diffbot2_description')
    xacro_file = pkg_share + '/urdf/diffbot2.xacro'
    robot_description = xacro.process(xacro_file)
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}]
        ),
        launch_ros.actions.Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}]
        )
    ])
