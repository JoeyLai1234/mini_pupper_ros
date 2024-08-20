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
from geometry_msgs.msg import Twist
import time


class MovementNode(Node):
    def __init__(self):
        super().__init__('movement_node')
        self.interval = 8.0
        self.angle = 0.0

        self.pointer_sub = self.create_subscription(
            String,
            'direction_of_pointer',
            self._direction_callback,
            10
        )

        if direction.data == 'left':
            self.angle = 10.0
        elif direction.data == 'right':
            self.angle = -10.0
        else:
            self.angle = 0.0

        velocity_cmd = Twist()
        velocity_cmd.angular.z = self.angle
        self.vel_publisher_.publish(velocity_cmd)
        time.sleep(self.interval)


def main(args=None):
    rclpy.init(args=args)
    node = MovementNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()