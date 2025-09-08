#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import time
from frequency_config import get_node_frequency

class PositionNode9(Node):
    def __init__(self):
        super().__init__("position_node_9")
        self.publisher = self.create_publisher(PoseStamped, "/mavros/setpoint_position/local", 10)
        frequency = get_node_frequency("position_node_9")
        self.timer = self.create_timer(frequency, self.timer_callback)
        self.get_logger().info(f"Position Node 9 started at {int(1/frequency):,}Hz")
        
    def timer_callback(self):
        node_num = 9
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.pose.position.x = -3.5
        msg.pose.position.y = -1.3000000000000003
        msg.pose.position.z = 0.3
        msg.pose.orientation.x = 0.0
        msg.pose.orientation.y = 0.0
        msg.pose.orientation.z = 0.0
        msg.pose.orientation.w = 0.707
        
        self.publisher.publish(msg)

def main():
    rclpy.init()
    node = PositionNode9()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
