#!/usr/bin/env python3
"""
Topic: /mavros/setpoint_velocity/cmd_vel (geometry_msgs/TwistStamped)
발행 주기: 10Hz
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
import time


class SetpointVelocityCmdVelNode(Node):
    def __init__(self):
        super().__init__('setpoint_velocity_cmd_vel_node')
        
        self.node_id = 2
        self.msg_counter = 0
        
        self.publisher = self.create_publisher(
            TwistStamped,
            '/mavros/setpoint_velocity/cmd_vel',
            10
        )
        
        self.timer = self.create_timer(0.1, self.publish_message)
        self.get_logger().info(f'Node {self.node_id} started: /mavros/setpoint_velocity/cmd_vel @ 10Hz')
    
    def publish_message(self):
        msg = TwistStamped()
        publish_time_ns = time.time_ns()
        self.msg_counter += 1
        
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = f"node_{self.node_id}_msg_{self.msg_counter}_time_{publish_time_ns}"
        
        msg.twist.linear.x = 0.5
        msg.twist.linear.y = 0.0
        msg.twist.linear.z = 0.0
        msg.twist.angular.z = 0.1
        
        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = SetpointVelocityCmdVelNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

