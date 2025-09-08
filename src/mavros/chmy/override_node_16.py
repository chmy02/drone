#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from mavros_msgs.msg import OverrideRCIn
import time
from frequency_config import get_node_frequency

class OverrideNode16(Node):
    def __init__(self):
        super().__init__("override_node_16")
        self.publisher = self.create_publisher(OverrideRCIn, "/mavros/rc/override", 10)
        frequency = get_node_frequency("override_node_16")
        self.timer = self.create_timer(frequency, self.timer_callback)
        self.get_logger().info(f"Override Node 16 started at {int(1/frequency):,}Hz")
        
    def timer_callback(self):
        node_num = 16
        msg = OverrideRCIn()
        base_value = 1500 + (node_num % 5 - 2) * 80
        msg.channels = [
            base_value, 
            base_value + 40, 
            base_value - 40, 
            base_value + 20,
            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
        ]
        
        self.publisher.publish(msg)

def main():
    rclpy.init()
    node = OverrideNode16()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
