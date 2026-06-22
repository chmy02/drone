#!/usr/bin/env python3
"""
Topic 7: /mavros/setpoint_trajectory/local (trajectory_msgs/MultiDOFJointTrajectory)
발행 주기: 10Hz
"""

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import MultiDOFJointTrajectory, MultiDOFJointTrajectoryPoint
from geometry_msgs.msg import Transform, Twist
from builtin_interfaces.msg import Duration
import time
import psutil
import gc
gc.enable()  # 실험: GC 활성화 상태에서 레이턴시 측정


class SetpointTrajectoryLocalNode(Node):
    def __init__(self):
        super().__init__('setpoint_trajectory_local_node')
        
        self.node_id = 7
        self.msg_counter = 0
        
        self.gz_proc = None
        self.px4_proc = None
        self.mav_proc = None
        self._find_processes()
        psutil.cpu_percent()
        
        self.publisher = self.create_publisher(
            MultiDOFJointTrajectory,
            '/mavros/setpoint_trajectory/local',
            10
        )
        
        self.timer = self.create_timer(0.1, self.publish_message)
        self.get_logger().info(f'Node {self.node_id} started: /mavros/setpoint_trajectory/local @ 10Hz')
    
    def _find_processes(self):
        for proc in psutil.process_iter(['pid', 'name', 'cmdline']):
            try:
                name = proc.info['name'].lower()
                cmdline = ' '.join(proc.info['cmdline'] or []).lower()
                if 'gz' in name or 'gazebo' in name or 'gzserver' in name:
                    self.gz_proc = psutil.Process(proc.info['pid'])
                elif 'px4' in name or 'px4' in cmdline:
                    self.px4_proc = psutil.Process(proc.info['pid'])
                elif 'mavros' in cmdline or name == 'mavros_node':
                    self.mav_proc = psutil.Process(proc.info['pid'])
            except (psutil.NoSuchProcess, psutil.AccessDenied):
                pass
    
    def _get_cpu_info(self):
        cpu_total = psutil.cpu_percent()
        cpu_gz, cpu_px4, cpu_mav = 0.0, 0.0, 0.0
        try:
            if self.gz_proc: cpu_gz = self.gz_proc.cpu_percent()
        except: self.gz_proc = None
        try:
            if self.px4_proc: cpu_px4 = self.px4_proc.cpu_percent()
        except: self.px4_proc = None
        try:
            if self.mav_proc: cpu_mav = self.mav_proc.cpu_percent()
        except: self.mav_proc = None
        return cpu_total, cpu_gz, cpu_px4, cpu_mav
    
    def publish_message(self):
        msg = MultiDOFJointTrajectory()
        publish_time_ns = time.time_ns()
        self.msg_counter += 1
        cpu_total, cpu_gz, cpu_px4, cpu_mav = self._get_cpu_info()
        
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = (f"node_{self.node_id}_msg_{self.msg_counter}_time_{publish_time_ns}"
                               f"_cpu_{cpu_total:.1f}_gz_{cpu_gz:.1f}_px4_{cpu_px4:.1f}_mav_{cpu_mav:.1f}")
        
        # 단일 trajectory point 생성
        point = MultiDOFJointTrajectoryPoint()
        
        # Transform (position + orientation)
        transform = Transform()
        transform.translation.x = 1.0
        transform.translation.y = 0.0
        transform.translation.z = 1.0
        transform.rotation.w = 1.0
        transform.rotation.x = 0.0
        transform.rotation.y = 0.0
        transform.rotation.z = 0.0
        point.transforms.append(transform)
        
        # Velocity
        velocity = Twist()
        velocity.linear.x = 0.0
        velocity.linear.y = 0.0
        velocity.linear.z = 0.0
        point.velocities.append(velocity)
        
        # Acceleration
        accel = Twist()
        accel.linear.x = 0.0
        accel.linear.y = 0.0
        accel.linear.z = 0.0
        point.accelerations.append(accel)
        
        # Time from start
        point.time_from_start = Duration(sec=0, nanosec=100000000)  # 0.1초
        
        msg.points.append(point)
        
        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = SetpointTrajectoryLocalNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

