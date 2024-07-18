#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool

class KeyboardCmdRot(Node):

    def __init__(self):
        super().__init__('keyboard_cmd_rot')
        self.pub = self.create_publisher(String, '/cmd_rot', 10)
        self.pub2 = self.create_publisher(Bool, '/servo_state_move', 10)
        self.pub3 = self.create_publisher(Bool, '/op_create_mesh', 10)
        self.pub4 = self.create_publisher(Bool, '/op_save_mesh', 10)
        self.create_timer(0.1, self.timer_callback)
        self.get_logger().info('Node has been started.')

    def timer_callback(self):
        try:
            key = input('r: turn right, l: turn left, i: set init , o: create mesh , p: stop > ')
            msg = String()
            msg2 = Bool()
            msg3 = Bool()
            msg4 = Bool()

            if 'r' in key:
                msg.data = 'right'
            elif 'l' in key:
                msg.data = 'left'
            elif 'i' in key:
                msg.data = 'init'
            elif 'o' in key:
                msg3.data = True
            elif 'p' in key:
                msg3.data = False
            elif 'q' in key:
                msg4.data = True
                self.pub4.publish(msg4)
            else:
                msg.data = 'other'

            msg2.data = True

            self.pub.publish(msg)
            self.pub2.publish(msg2)
            self.pub3.publish(msg3)
            
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

