# Code provided by Manolo Fernandez Carmona

import rclpy
from rclpy.node import Node

from  sensor_msgs.msg import LaserScan


class FindMinimus(Node):

    def __init__(self, min_val_):
        super().__init__('minimus_finder')
        self.subscription = self.create_subscription( LaserScan, 'scan_filtered', self.listener_callback, 10)
        self.min_val = min_val_



    def listener_callback(self, msg):
        self.ranges = msg.ranges
        for i in range(len(self.ranges)):
            if (self.ranges[i]< self.min_val):
                print("ranges[" + str(i) + " ] = " + str(self.ranges[i]) )
        print(" . . . . . . .  . . . .  \n")



def main(args=None):
    rclpy.init(args=args)

    minimal_finder = FindMinimus(0.6)

    rclpy.spin(minimal_finder)

    minimal_finder.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()