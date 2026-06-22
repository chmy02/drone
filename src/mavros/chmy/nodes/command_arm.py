#!/usr/bin/env python3
"""
Service: /mavros/cmd/arming (mavros_msgs/srv/CommandBool)
발행 주기: 10Hz (Arm/Disarm 토글)
토픽 번호: 0 (서비스 콜)

레이턴시 측정:
  t1: Python 노드에서 service call 시작 (call_async 호출 시점)
  t3: MAVROS 플러그인(command.cpp)에서 service callback 시작
  t4: MAVROS 플러그인에서 send_message() 완료
  request.timestamp_ns, request.cpu_meta 로 t1·CPU 전달
"""

import rclpy
from rclpy.node import Node
from mavros_msgs.srv import CommandBool
import time
import os
from datetime import datetime
import psutil
import gc
gc.enable()  # 실험: GC 활성화 상태에서 레이턴시 측정


class CommandArmNode(Node):
    def __init__(self):
        super().__init__('command_arm_node')
        
        self.node_id = 0  # Topic 0 = Service Call
        self.msg_counter = 0
        self.arm_state = False  # False=Disarm, True=Arm 토글
        self.skip_first_call = True  # 첫 번째 호출 건너뛰기 (초기화 안정화)
        
        # CPU 모니터링용 프로세스 캐싱
        self.gz_proc = None
        self.px4_proc = None
        self.mav_proc = None
        self._find_processes()
        psutil.cpu_percent()
        
        # 서비스 클라이언트 생성
        self.client = self.create_client(
            CommandBool,
            '/mavros/cmd/arming'
        )
        
        # 로그 파일 설정 (t1 측정용)
        self.log_dir = "/home/rtcl-chmy/mavros_ws/src/mavros/chmy/logs"
        os.makedirs(self.log_dir, exist_ok=True)
        
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.log_file = os.path.join(
            self.log_dir, 
            f"{timestamp}_topic0_command_arm_t1.log"
        )
        
        # 로그 파일 초기화
        with open(self.log_file, 'w') as f:
            pass  # 빈 파일 생성
        
        # 서비스 대기
        self.get_logger().info('Waiting for /mavros/cmd/arming service...')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting...')
        
        self.get_logger().info(f'Node {self.node_id} started: /mavros/cmd/arming @ 10Hz')
        self.get_logger().info(f'T1 log file: {self.log_file}')
        
        # 10Hz 타이머 (첫 번째 호출은 건너뛰고, 0.1초 후부터 시작)
        self.timer = self.create_timer(0.1, self.call_service)
    
    def _find_processes(self):
        """Gazebo, PX4, MAVROS 프로세스 찾기"""
        for proc in psutil.process_iter(['pid', 'name', 'cmdline']):
            try:
                name = (proc.info.get('name') or '').lower()
                cmdline = ' '.join(proc.info.get('cmdline') or []).lower()
                if 'gz' in name or 'gazebo' in name or 'gzserver' in name:
                    self.gz_proc = psutil.Process(proc.info['pid'])
                elif 'px4' in name or 'px4' in cmdline:
                    self.px4_proc = psutil.Process(proc.info['pid'])
                elif 'mavros' in cmdline or name == 'mavros_node':
                    self.mav_proc = psutil.Process(proc.info['pid'])
            except (psutil.NoSuchProcess, psutil.AccessDenied):
                pass
    
    def _get_cpu_info(self):
        """CPU 사용률 측정 (total, gz, px4, mav)"""
        cpu_total = psutil.cpu_percent()
        cpu_gz = cpu_px4 = cpu_mav = 0.0
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
    
    def call_service(self):
        """서비스 비동기 호출"""
        if self.skip_first_call:
            self.skip_first_call = False
            self.get_logger().info('Skipping first call for initialization...')
            return
        
        self.msg_counter += 1
        self.arm_state = not self.arm_state  # 토글 (arm ↔ disarm)
        
        # t1 및 CPU 측정 (call_async 직전)
        t1_ns = time.time_ns()
        cpu_total, cpu_gz, cpu_px4, cpu_mav = self._get_cpu_info()
        cpu_meta = f"{cpu_total:.1f}_{cpu_gz:.1f}_{cpu_px4:.1f}_{cpu_mav:.1f}"
        
        request = CommandBool.Request()
        request.value = self.arm_state
        request.timestamp_ns = int(t1_ns)
        request.cpu_meta = cpu_meta
        
        # t1 로그 기록 (msg_counter, t1)
        with open(self.log_file, 'a') as f:
            f.write(f"{self.msg_counter},{t1_ns}\n")
        
        # 비동기 호출 (블로킹 방지)
        future = self.client.call_async(request)
        future.add_done_callback(
            lambda f: self.service_callback(f, self.msg_counter, self.arm_state)
        )
    
    def service_callback(self, future, msg_id, arm_value):
        """서비스 응답 콜백"""
        try:
            response = future.result()
            success = response.success
            result = response.result
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')
            success = False
            result = 255
        
        # 100번째마다 상태 출력
        if msg_id % 100 == 0:
            state_str = "ARM" if arm_value else "DISARM"
            self.get_logger().info(
                f'[{msg_id}] {state_str}: success={success}, result={result}'
            )


def main(args=None):
    rclpy.init(args=args)
    node = CommandArmNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info(f'T1 log file: {node.log_file}')
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
