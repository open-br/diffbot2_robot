#!/usr/bin/env python3

import launch
import launch_ros
import xacro
import ament_index_python


def generate_launch_description():
  pkg_share = ament_index_python.get_package_share_directory('diffbot2_description')
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
