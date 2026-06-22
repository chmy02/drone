#!/usr/bin/env python3
"""
Topic: /mavros/setpoint_raw/local (mavros_msgs/PositionTarget)
발행 주기: 10Hz
"""

import rclpy
from rclpy.node import Node
from mavros_msgs.msg import PositionTarget
import time
import psutil
import gc
gc.enable()  # 실험: GC 활성화 상태에서 레이턴시 측정


class SetpointRawLocalNode(Node):
    def __init__(self):
        super().__init__('setpoint_raw_local_node')
        
        self.node_id = 1
        self.msg_counter = 0
        
        # CPU 모니터링용 프로세스 캐싱
        self.gz_proc = None
        self.px4_proc = None
        self.mav_proc = None
        self._find_processes()
        
        # 초기 CPU 측정 (baseline)
        psutil.cpu_percent()
        
        self.publisher = self.create_publisher(
            PositionTarget,
            '/mavros/setpoint_raw/local',
            10
        )
        
        self.timer = self.create_timer(0.1, self.publish_message)
        self.get_logger().info(f'Node {self.node_id} started: /mavros/setpoint_raw/local @ 10Hz')
    
    def _find_processes(self):
        """Gazebo, PX4, MAVROS 프로세스 찾기"""
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
        """CPU 사용률 측정"""
        cpu_total = psutil.cpu_percent()
        
        cpu_gz = 0.0
        cpu_px4 = 0.0
        cpu_mav = 0.0
        
        try:
            if self.gz_proc:
                cpu_gz = self.gz_proc.cpu_percent()
        except (psutil.NoSuchProcess, psutil.AccessDenied):
            self.gz_proc = None
            
        try:
            if self.px4_proc:
                cpu_px4 = self.px4_proc.cpu_percent()
        except (psutil.NoSuchProcess, psutil.AccessDenied):
            self.px4_proc = None
            
        try:
            if self.mav_proc:
                cpu_mav = self.mav_proc.cpu_percent()
        except (psutil.NoSuchProcess, psutil.AccessDenied):
            self.mav_proc = None
        
        return cpu_total, cpu_gz, cpu_px4, cpu_mav
    
    def publish_message(self):
        msg = PositionTarget()
        publish_time_ns = time.time_ns()
        self.msg_counter += 1
        
        # CPU 측정
        cpu_total, cpu_gz, cpu_px4, cpu_mav = self._get_cpu_info()
        
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = (f"node_{self.node_id}_msg_{self.msg_counter}_time_{publish_time_ns}"
                               f"_cpu_{cpu_total:.1f}_gz_{cpu_gz:.1f}_px4_{cpu_px4:.1f}_mav_{cpu_mav:.1f}")
        
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

