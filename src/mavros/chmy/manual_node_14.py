#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from mavros_msgs.msg import ManualControl
import time
from frequency_config import get_node_frequency

class ManualNode14(Node):
    def __init__(self):
        super().__init__("manual_node_14")
        self.publisher = self.create_publisher(ManualControl, "/mavros/manual_control/send", 10)
        frequency = get_node_frequency("manual_node_14")
        self.timer = self.create_timer(frequency, self.timer_callback)
        self.get_logger().info(f"Manual Node 14 started at {int(1/frequency):,}Hz")
        
    def timer_callback(self):
        node_num = 14
        msg = ManualControl()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.x = 0.1
        msg.y = 0.30000000000000004
        msg.z = 0.7
        msg.r = -0.05
        msg.buttons = 14
        
        self.publisher.publish(msg)

def main():
    rclpy.init()
    node = ManualNode14()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
