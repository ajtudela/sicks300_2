#!/usr/bin/env python3
# Copyright (c) 2022 Manolo Fernandez Carmona
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http:#www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


class FindMinimus(Node):

    def __init__(self, min_val_):
        super().__init__('minimus_finder')
        self.subscription = self.create_subscription(
            LaserScan, 'scan_filtered', self.listener_callback, qos_profile_sensor_data)
        self.min_val = min_val_

    def listener_callback(self, msg):
        self.ranges = msg.ranges
        for i in range(len(self.ranges)):
            if (self.ranges[i] < self.min_val):
                print('ranges[' + str(i) + ' ] = ' + str(self.ranges[i]))
        print(' . . . . . . .  . . . .  \n')


def main(args=None):
    rclpy.init(args=args)

    minimal_finder = FindMinimus(0.6)

    rclpy.spin(minimal_finder)

    minimal_finder.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
