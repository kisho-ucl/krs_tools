#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class KeyboardCmdRot(Node):

    def __init__(self):
        super().__init__('keyboard_cmd_rot')
        self.pub = self.create_publisher(String, '/cmd_rot', 10)
        self.create_timer(0.1, self.timer_callback)
        self.get_logger().info('Node has been started.')

    def timer_callback(self):
        try:
            key = input('r: turn right, l: turn left, i: set init > ')
            msg = String()

            if 'r' in key:
                msg.data = 'right'
            elif 'l' in key:
                msg.data = 'left'
            elif 'i' in key:
                msg.data = 'init'
            else:
                msg.data = 'other'

            self.pub.publish(msg)
        except KeyboardInterrupt:
            self.shutdown()

    def shutdown(self):
        self.get_logger().info('Shutting down node.')

def main(args=None):
    rclpy.init(args=args)
    node = KeyboardCmdRot()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.shutdown()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

