#!/usr/bin/env python
############################################################################
#
#   Copyright (C) 2022 PX4 Development Team. All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in
#    the documentation and/or other materials provided with the
#    distribution.
# 3. Neither the name PX4 nor the names of its contributors may be
#    used to endorse or promote products derived from this software
#    without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
# OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
# AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
############################################################################

__author__ = "Braden Wagstaff"
__contact__ = "braden@arkelectron.com"

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
import math


LINE = 'l'
SQUARE = 's'


def get_spawn_position(spawn_configuration, uav_instance, uav_number):
    if spawn_configuration == LINE: # UAVs in a line fomation
        coord_x = uav_instance
        coord_y = 0
    
    if spawn_configuration == SQUARE: # UAVs in a square formation
        width = math.ceil(math.sqrt(uav_number))
        coord_x = uav_instance % width
        coord_y = uav_instance // width

    return f"{coord_y},{coord_x}"


def generate_uav_nodes(context):
    uav_number = int(LaunchConfiguration('uav_number').perform(context))
    spawn_configuration = LaunchConfiguration('spawn_configuration').perform(context)

    nodes = []

    nodes.append(
        Node(
            package='px4_offboard',
            namespace='px4_offboard',
            executable='processes',
            name='processes',
            arguments=['0']
        )
    )

    for i in range(uav_number): # create nodes for each UAV instance
        spawn_position = get_spawn_position(spawn_configuration, i, uav_number)

        nodes.extend([
            Node(
                package='px4_offboard',
                namespace=f'px4_{i+1}',
                executable='processes',
                name='processes',
                arguments=[f'{i+1}', spawn_position]
            ),
            Node(
                package='px4_offboard',
                namespace=f'px4_{i+1}',
                executable='visualizer',
                name='visualizer',
                arguments=[f'px4_{i+1}']
            ),
            Node(
                package='px4_offboard',
                namespace=f'px4_{i+1}',
                executable='control',
                name='control',
                arguments=[f'px4_{i+1}'],
                prefix='gnome-terminal --'
            ),
            Node(
                package='px4_offboard',
                namespace=f'px4_{i+1}',
                executable='velocity_control',
                name='velocity',
                arguments=[f'px4_{i+1}'],
                # prefix='gnome-terminal --'
            )
        ])

    return nodes


def generate_launch_description():
    uav_number_argument = DeclareLaunchArgument(
        'uav_number',
        default_value='1',
    )

    spawn_configuration_argument = DeclareLaunchArgument(
        'spawn_configuration',
        default_value=LINE,
    )

    return LaunchDescription([
        uav_number_argument,
        spawn_configuration_argument,
        OpaqueFunction(function=generate_uav_nodes)
    ])
