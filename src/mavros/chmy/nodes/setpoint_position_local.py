#!/usr/bin/env python3
"""
Topic: /mavros/setpoint_position/local (geometry_msgs/PoseStamped)
발행 주기: 10Hz
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import time


class SetpointPositionLocalNode(Node):
    def __init__(self):
        super().__init__('setpoint_position_local_node')
        
        self.node_id = 3
        self.msg_counter = 0
        
        self.publisher = self.create_publisher(
            PoseStamped,
            '/mavros/setpoint_position/local',
            10
        )
        
        self.timer = self.create_timer(0.1, self.publish_message)
        self.get_logger().info(f'Node {self.node_id} started: /mavros/setpoint_position/local @ 10Hz')
    
    def publish_message(self):
        msg = PoseStamped()
        publish_time_ns = time.time_ns()
        self.msg_counter += 1
        
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = f"node_{self.node_id}_msg_{self.msg_counter}_time_{publish_time_ns}"
        
        msg.pose.position.x = 2.0
        msg.pose.position.y = 0.0
        msg.pose.position.z = 1.5
        msg.pose.orientation.w = 1.0
        
        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = SetpointPositionLocalNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

