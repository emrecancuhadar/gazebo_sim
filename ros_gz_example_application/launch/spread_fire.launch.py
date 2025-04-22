#!/usr/bin/env python3

# Copyright 2022 Open Source Robotics Foundation, Inc.
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

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node


def generate_launch_description():
    # Configure ROS nodes for launch

    # Setup project paths
    pkg_project_bringup = get_package_share_directory('ros_gz_example_bringup')
    pkg_project_gazebo = get_package_share_directory('ros_gz_example_gazebo')
    pkg_project_application = get_package_share_directory('ros_gz_example_application')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    # Option to only run the bridge
    use_sim = LaunchConfiguration('use_sim')
    
    # Setup to launch the simulator and Gazebo world
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={
            'gz_sim_launch_file': os.path.join(
                pkg_project_gazebo, 'launch', 'diff_drive.launch.py'),
        }.items(),
        condition=IfCondition(use_sim)
    )

    # Load ROS-GZ bridge config file
    bridge_config = os.path.join(pkg_project_bringup, 'config', 'ros_gz_example_bridge.yaml')

    # Bridge ROS topics and Gazebo messages for establishing communication
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[
            {'config_file': bridge_config},
            # Required service bridges for spawning/deleting entities
            {'/world/demo/create': {'ros_type_name': 'ros_gz_interfaces/srv/SpawnEntity', 
                                    'gz_type_name': 'gz.msgs.EntityFactory', 
                                    'direction': 'ROS_TO_GZ'}},
            {'/world/demo/remove': {'ros_type_name': 'ros_gz_interfaces/srv/DeleteEntity', 
                                    'gz_type_name': 'gz.msgs.Entity',
                                    'direction': 'ROS_TO_GZ'}}
        ],
        output='screen'
    )

    # Run the spread fire node
    spread_fire = Node(
        package='ros_gz_example_application',
        executable='spreadFire.py',
        name='spread_fire',
        output='screen'
    )
    
    return LaunchDescription([
        DeclareLaunchArgument('use_sim', default_value='true',
                              description='Run the Gazebo simulation'),
        gz_sim,
        bridge,
        spread_fire
    ]) 