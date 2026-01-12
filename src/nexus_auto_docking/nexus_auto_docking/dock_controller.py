#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class DockController(Node):
    def __init__(self):
        super().__init__('dock_controller')
        self.sub = self.create_subscription(Twist, '/dock/vision', self.cb, 10)
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.get_logger().info("Dock controller started")

    def cb(self, msg):
        error = msg.angular.z
        area = msg.linear.x

        out = Twist()

        TARGET = 120000  # you will tune this

        if area > TARGET:
            out.linear.x = 0.0
            out.angular.z = 0.0
            self.get_logger().info("DOCKED")
        elif abs(error) > 5:
            out.angular.z = -0.003 * error
            out.linear.x = 0.0
        else:
            out.angular.z = 0.0
            out.linear.x = 0.05

        self.pub.publish(out)

def main():
    rclpy.init()
    node = DockController()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
