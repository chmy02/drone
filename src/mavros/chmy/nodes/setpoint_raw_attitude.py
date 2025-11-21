#!/usr/bin/env python3
"""
Topic: /mavros/setpoint_raw/attitude (mavros_msgs/AttitudeTarget)
발행 주기: 10Hz
"""

import rclpy
from rclpy.node import Node
from mavros_msgs.msg import AttitudeTarget
import time


class SetpointRawAttitudeNode(Node):
    def __init__(self):
        super().__init__('setpoint_raw_attitude_node')
        
        self.node_id = 4
        self.msg_counter = 0
        
        self.publisher = self.create_publisher(
            AttitudeTarget,
            '/mavros/setpoint_raw/attitude',
            10
        )
        
        self.timer = self.create_timer(0.1, self.publish_message)
        self.get_logger().info(f'Node {self.node_id} started: /mavros/setpoint_raw/attitude @ 10Hz')
    
    def publish_message(self):
        msg = AttitudeTarget()
        publish_time_ns = time.time_ns()
        self.msg_counter += 1
        
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = f"node_{self.node_id}_msg_{self.msg_counter}_time_{publish_time_ns}"
        
        msg.type_mask = 0b00000111
        msg.orientation.w = 1.0
        msg.thrust = 0.0  # thrust_scaling 미설정 시 early return 방지
        
        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = SetpointRawAttitudeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

