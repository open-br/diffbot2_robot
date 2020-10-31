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


from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Get rviz config
    rviz_config = PathJoinSubstitution(
        [FindPackageShare("diffbot2_description"), "rviz/view_robot.rviz"]
    )

    # OBS: do not add rviz2 as dependency because it is heavy
    return LaunchDescription(
        [
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz",
                output="screen",
                arguments=["-d", rviz_config],
            )
        ]
    )
