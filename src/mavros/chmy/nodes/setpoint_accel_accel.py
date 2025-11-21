#!/usr/bin/env python3
"""
Topic: /mavros/setpoint_accel/accel (geometry_msgs/Vector3Stamped)
발행 주기: 10Hz
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3Stamped
import time


class SetpointAccelAccelNode(Node):
    def __init__(self):
        super().__init__('setpoint_accel_accel_node')
        
        self.node_id = 5
        self.msg_counter = 0
        
        self.publisher = self.create_publisher(
            Vector3Stamped,
            '/mavros/setpoint_accel/accel',
            10
        )
        
        self.timer = self.create_timer(0.1, self.publish_message)
        self.get_logger().info(f'Node {self.node_id} started: /mavros/setpoint_accel/accel @ 10Hz')
    
    def publish_message(self):
        msg = Vector3Stamped()
        publish_time_ns = time.time_ns()
        self.msg_counter += 1
        
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = f"node_{self.node_id}_msg_{self.msg_counter}_time_{publish_time_ns}"
        
        msg.vector.x = 0.5
        msg.vector.y = 0.0
        msg.vector.z = 0.0
        
        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = SetpointAccelAccelNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

