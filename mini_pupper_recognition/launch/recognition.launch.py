#!/usr/bin/env python3

# SPDX-License-Identifier: Apache-2.0
#
# Copyright (c) 2024 MangDang
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
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition


def generate_launch_description():

    mark_arg = DeclareLaunchArgument(
        name='mark',
        default_value='False',
        description='Enable mark(picture detection) if true'
    )

    ai_line_recognition_node = Node(
            package="mini_pupper_recognition",
            namespace="",
            executable="ai_line_recognition_node",
            name="ai_line_recognition_node",
    )
    movement_node = Node(
            package="mini_pupper_recognition",
            namespace="",
            executable="movement_node",
            name="movement_node",
    )

    ai_face_recognition_node = Node(
            package="mini_pupper_recognition",
            namespace="",
            executable="ai_face_recognition_node",
            name="ai_face_recognition_node",
            condition=IfCondition(LaunchConfiguration('mark'))
    )
    music_dance_node = Node(
            package="mini_pupper_recognition",
            namespace="",
            executable="music_dance_node",
            name="music_dance_node",
            condition=IfCondition(LaunchConfiguration('mark'))
    )

    return LaunchDescription([
        mark_arg,
        ai_line_recognition_node,
        movement_node,
        ai_face_recognition_node,
        music_dance_node
    ])
