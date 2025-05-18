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
    # ─── Project package directories ────────────────────────────────────
    pkg_bringup     = get_package_share_directory('ros_gz_example_bringup')
    pkg_gazebo      = get_package_share_directory('ros_gz_example_gazebo')
    pkg_description = get_package_share_directory('ros_gz_example_description')
    pkg_ros_gz_sim  = get_package_share_directory('ros_gz_sim')

    # ─── Load robot description (SDF) ───────────────────────────────────
    sdf_file = os.path.join(pkg_description, 'models', 'diff_drive', 'model.sdf')
    with open(sdf_file, 'r') as infp:
        robot_desc = infp.read()

    # 2) red‐tinted diff_drive2 (you should have created this folder & SDF)
    red_sdf_file = os.path.join(pkg_description, 'models', 'diff_drive_red', 'model.sdf')
    with open(red_sdf_file, 'r') as infp:
        red_desc = infp.read()

    # ─── Launch Gazebo-sim with the world ───────────────────────────────
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': PathJoinSubstitution([
                pkg_gazebo,
                'worlds',
                'diff_drive.sdf'
            ])
        }.items(),
    )

    # ─── ROS-Gazebo bridge ──────────────────────────────────────────────
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        output='screen',
        parameters=[{
            'config_file': os.path.join(
                pkg_bringup, 'config', 'ros_gz_example_bridge.yaml'
            ),
            'qos_overrides./tf_static.publisher.durability': 'transient_local',
        }],
    )

    # ─── State publishers for each robot ───────────────────────────────
    robot_state_publisher1 = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        namespace='diff_drive',
        output='both',
        parameters=[
            {'use_sim_time': True},
            {'robot_description': robot_desc},
        ],
    )

    robot_state_publisher2 = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        namespace='diff_drive2',
        output='both',
        parameters=[
            {'use_sim_time': True},
            {'robot_description': red_desc},
        ],
    )

    # ─── Static TFs: world → each robot’s odom ──────────────────────────
    static_tf1 = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_world_to_diff_drive',
        arguments=[
            '0', '0', '0',        # x y z
            '0', '0', '0', '1',   # qx qy qz qw
            'world',              # parent frame
            'diff_drive/odom'     # child frame
        ]
    )
    static_tf2 = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_world_to_diff_drive2',
        arguments=[
            '0', '0', '0',        # x y z  (Gazebo already placed diff_drive2 at x=1 in the world file)
            '0', '0', '0', '1',   # qx qy qz qw
            'world',
            'diff_drive2/odom'
        ]
    )

    # ─── RViz for visualization ────────────────────────────────────────
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=[
            '-d',
            os.path.join(pkg_bringup, 'config', 'diff_drive.rviz')
        ],
        condition=IfCondition(LaunchConfiguration('rviz')),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'rviz', default_value='true',
            description='Whether to launch RViz'
        ),

        gz_sim,
        bridge,

        robot_state_publisher1,
        robot_state_publisher2,

        static_tf1,
        static_tf2,

        rviz,
    ])
