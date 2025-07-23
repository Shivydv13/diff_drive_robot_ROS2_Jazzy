#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
import sys
import tty
import termios
import select
import time


class KeyboardTeleop(Node):
    def __init__(self):
        super().__init__('keyboard_teleop_node')
        self.publisher_ = self.create_publisher(TwistStamped, '/input_key/cmd_vel_stamped', 10)
        self.get_logger().info("Use W/A/S/D to move, Q to stop.")
        self.timer = self.create_timer(0.1, self.publish_command)
        self.key = None
        self.old_settings = termios.tcgetattr(sys.stdin)
        tty.setcbreak(sys.stdin.fileno())

    def destroy_node(self):
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_settings)
        super().destroy_node()

    def publish_command(self):
        msg = TwistStamped()
        if self.kbhit():
            ch = sys.stdin.read(1)
            self.key = ch.lower()

        if self.key == 'w':
            msg.twist.linear.x = 0.5
            msg.twist.angular.z = 0.0
        elif self.key == 's':
            msg.twist.linear.x = -0.5
            msg.twist.angular.z = 0.0
        elif self.key == 'a':
            msg.twist.linear.x = 0.0
            msg.twist.angular.z = 1.0
        elif self.key == 'd':
            msg.twist.linear.x = 0.0
            msg.twist.angular.z = -1.0
        elif self.key == 'q':
            msg.twist.linear.x = 0.0
            msg.twist.angular.z = 0.0
        else:
            return

        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        self.publisher_.publish(msg)

    def kbhit(self):
        """Check if keyboard was hit (non-blocking)"""
        dr, _, _ = select.select([sys.stdin], [], [], 0)
        return dr != []


def main(args=None):
    rclpy.init(args=args)
    node = KeyboardTeleop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
