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

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from ../../mini_pupper_music/mini_pupper_music.music_server import MusicServiceNode
from ../../mini_pupper_dance/mini_pupper_dance.dance_client import MiniPupperDanceClientAsync
from ../../mini_pupper_dance/mini_pupper_dance.episode import dance_commands
from ../../mini_pupper_dance/mini_pupper_dance.dance_server import MiniPupperDanceService


class MusicDanceNode(Node):
    def __init__(self):
        super().__init__('music_dance_node')
        self.dancer = ''
        self.song = ''
        self.dance_client = MiniPupperDanceClientAsync()

        self.singer_sub = self.create_subscription(
            String,
            'is_singer',
            self._singer_callback,
            10
        )

        self.dancer_sub = self.create_subscription(
            String,
            'is_dancer',
            self._dancer_callback,
            10
        )

        self.song_sub = self.create_subscription(
            String,
            'song',
            self._song_callback,
            10
        )

    def _dancer_callback(self, dancer):
        if dancer.data == 'dancer':
            for command in self.dance_client.dance_commands:
                self.dance_client.send_dance_request(command)

    def _song_callback(self, msg):
        self.song = msg.data

    def _singer_callback(self, singer):
        if singer.data == 'singer':
            self.dance_client.send_play_music_request(f"{self.song}.wav", 1)


def main(args=None):
    rclpy.init(args=args)
    music_dance_node = MusicDanceNode()
    music_service_node = MusicServiceNode()
    dance_service_node = DanceServiceNode()
    rclpy.spin(music_dance_node)
    rclpy.spin(music_service_node)
    rclpy.spin(dance_service_node)    
    rclpy.shutdown()


if __name__ == '__main__':
    main()