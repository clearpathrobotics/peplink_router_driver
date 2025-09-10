#!/usr/bin/env python3

# Copyright 2025 Clearpath Robotics Inc.
# All rights reserved.
#
# Software License Agreement (BSD License 2.0)
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Clearpath Robotics Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    OpaqueFunction
)
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace


ARGUMENTS = [
    DeclareLaunchArgument('namespace', default_value='',
                          description='Robot namespace'),
    DeclareLaunchArgument('ip_address', default_value='192.168.131.51',
                          description='IP address of the router to manage'),
    DeclareLaunchArgument('username', default_value='admin',
                          description='Login ID for the router'),
    DeclareLaunchArgument('password', default_value='admin',
                          description='Login password for the router'),
    DeclareLaunchArgument('enable_gps', default_value='false',
                          description='Enable publishing GPS data from the router',
                          choices=['false', 'true']),
    DeclareLaunchArgument('publish_passwords', default_value='false',
                          description='Publish modem and SIM card passwords in ROS topics',
                          choices=['false', 'true']),
]


def launch_setup(context, *args, **kwargs):
    namespace = LaunchConfiguration('namespace')
    ip_address = LaunchConfiguration('ip_address')
    username = LaunchConfiguration('username')
    password = LaunchConfiguration('password')
    enable_gps = LaunchConfiguration('enable_gps')
    publish_passwords = LaunchConfiguration('publish_passwords')

    peplink_node = Node(
          name='peplink_router_node',
          package='peplink_router_driver',
          executable='peplink_router_node',
          output='screen',
          parameters=[{
              'ip_address': ip_address,
              'username': username,
              'password': password,
              'enable_gps': enable_gps,
              'publish_passwords': publish_passwords,
          }],
      )

    actions = GroupAction([
        PushRosNamespace(namespace),
        peplink_node,
    ])

    return [actions]


def generate_launch_description():
    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(OpaqueFunction(function=launch_setup))
    return ld
