#!/usr/bin/env python3
"""
Topic: /mavros/setpoint_raw/local (mavros_msgs/PositionTarget)
발행 주기: 10Hz
"""

import rclpy
from rclpy.node import Node
from mavros_msgs.msg import PositionTarget
import time


class SetpointRawLocalNode(Node):
    def __init__(self):
        super().__init__('setpoint_raw_local_node')
        
        self.node_id = 1
        self.msg_counter = 0
        
        self.publisher = self.create_publisher(
            PositionTarget,
            '/mavros/setpoint_raw/local',
            10
        )
        
        self.timer = self.create_timer(0.1, self.publish_message)
        self.get_logger().info(f'Node {self.node_id} started: /mavros/setpoint_raw/local @ 10Hz')
    
    def publish_message(self):
        msg = PositionTarget()
        publish_time_ns = time.time_ns()
        self.msg_counter += 1
        
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = f"node_{self.node_id}_msg_{self.msg_counter}_time_{publish_time_ns}"
        
        msg.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
        msg.type_mask = 0b0000111111111000
        msg.position.x = 1.0
        msg.position.y = 0.0
        msg.position.z = -1.0
        
        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = SetpointRawLocalNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

