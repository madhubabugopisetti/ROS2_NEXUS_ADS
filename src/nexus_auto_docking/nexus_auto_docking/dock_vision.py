#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from geometry_msgs.msg import Twist

class DockVision(Node):
    def __init__(self):
        super().__init__('dock_vision')
        self.sub = self.create_subscription(Image, '/camera/image', self.cb, 10)
        self.pub = self.create_publisher(Twist, '/dock/vision', 10)
        self.bridge = CvBridge()
        self.get_logger().info("Dock vision node started")

    def cb(self, msg):
        img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        lower1 = (0, 120, 70)
        upper1 = (10, 255, 255)
        lower2 = (170, 120, 70)
        upper2 = (180, 255, 255)

        mask1 = cv2.inRange(hsv, lower1, upper1)
        mask2 = cv2.inRange(hsv, lower2, upper2)
        mask = mask1 | mask2

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if not contours:
            return

        c = max(contours, key=cv2.contourArea)
        x, y, w, h = cv2.boundingRect(c)

        cx = x + w // 2
        img_center = img.shape[1] // 2
        error = cx - img_center
        area = w * h

        msg = Twist()
        msg.angular.z = float(error)
        msg.linear.x = float(area)
        self.pub.publish(msg)

        # Debug draw
        cv2.rectangle(img, (x,y), (x+w, y+h), (0,255,0), 2)
        cv2.line(img, (cx, 0), (cx, img.shape[0]), (255,0,0), 2)
        cv2.line(img, (img_center, 0), (img_center, img.shape[0]), (0,255,255), 2)
        cv2.imshow("dock_vision", img)
        cv2.waitKey(1)

def main():
    rclpy.init()
    node = DockVision()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
