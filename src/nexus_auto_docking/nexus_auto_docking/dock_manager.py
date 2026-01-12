#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool

class DockManager(Node):
    def __init__(self):
        super().__init__('dock_manager')

        self.client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        self.get_logger().info("Waiting for Nav2...")
        self.client.wait_for_server()

        self.get_logger().info("Sending pre-dock goal...")
        self.send_goal()

    def send_goal(self):
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()

        # 🔴 CHANGE THIS to your pre-dock pose
        goal.pose.pose.position.x = 1.5
        goal.pose.pose.position.y = 0.0
        goal.pose.pose.orientation.w = 1.0

        self._send_goal_future = self.client.send_goal_async(goal)
        self._send_goal_future.add_done_callback(self.goal_response_cb)

    def goal_response_cb(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Pre-dock goal rejected")
            return

        self.get_logger().info("Pre-dock goal accepted")
        self._result_future = goal_handle.get_result_async()
        self._result_future.add_done_callback(self.result_cb)

    def result_cb(self, future):
        self.get_logger().info("Reached pre-dock pose!")
        self.get_logger().info("Now run dock_controller + dock_vision manually.")

def main():
    rclpy.init()
    node = DockManager()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
