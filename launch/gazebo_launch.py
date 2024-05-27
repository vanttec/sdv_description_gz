# Copyright 2019 Open Source Robotics Foundation, Inc.
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
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node


def generate_launch_description():

    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    pkg_sdv_description = get_package_share_directory('sdv_description_gz')
    rviz_config = os.path.join(get_package_share_directory('sdv_description_gz'),'rviz/','rviz.rviz')

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={
            'gz_args': PathJoinSubstitution([
            pkg_sdv_description,
            'worlds',
            'parking.sdf'
        ])}.items(),
    )

    # Bridge
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
                   '/delta_l@std_msgs/msg/Float64@gz.msgs.Double',
                   '/delta_r@std_msgs/msg/Float64@gz.msgs.Double',
                   '/wheel_speed@std_msgs/msg/Float64@gz.msgs.Double',
                   '/gz_sim/odometry@nav_msgs/msg/Odometry@gz.msgs.OdometryWithCovariance',
                   '/lidar@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
                   '/lidar/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked',
                   '/zed_rgbd/image@sensor_msgs/msg/Image@gz.msgs.Image',
                   ],
        # parameters=[{
        #     'qos_overrides./model/my_roboboat.subscriber.reliability': 'reliable',
        #             }],
        output='screen'
    )

    ros_ackermann_bridge = Node(
        package='sdv_description_gz',
        executable='ros_ackermann_bridge',
        name='ros_ackermann_bridge',
        output='screen',
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
    )

    return LaunchDescription([
        gz_sim,
        DeclareLaunchArgument('rviz', default_value='true',
                              description='Open RViz.'),
        bridge,
        rviz,
        ros_ackermann_bridge,
    ])

'''
# Lidar Frame
/vtec_sdv/chassis/gpu_lidar
'''