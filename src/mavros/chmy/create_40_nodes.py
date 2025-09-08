#!/usr/bin/env python3
"""
40개 노드를 생성하는 스크립트
"""

import os

def create_position_node(node_num):
    """위치 명령 노드 생성"""
    filename = f"position_node_{node_num}.py"
    
    content = f'''#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import time
from frequency_config import get_node_frequency

class PositionNode{node_num}(Node):
    def __init__(self):
        super().__init__("position_node_{node_num}")
        self.publisher = self.create_publisher(PoseStamped, "/mavros/setpoint_position/local", 10)
        frequency = get_node_frequency("position_node_{node_num}")
        self.timer = self.create_timer(frequency, self.timer_callback)
        self.get_logger().info(f"Position Node {node_num} started at {{int(1/frequency):,}}Hz")
        
    def timer_callback(self):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.pose.position.x = {node_num * 0.5 - 8.0}
        msg.pose.position.y = {node_num * 0.3 - 4.0}
        msg.pose.position.z = 0.3
        msg.pose.orientation.x = 0.0
        msg.pose.orientation.y = 0.0
        msg.pose.orientation.z = 0.0
        msg.pose.orientation.w = 0.707
        
        self.publisher.publish(msg)

def main():
    rclpy.init()
    node = PositionNode{node_num}()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
'''
    
    with open(filename, 'w') as f:
        f.write(content)
    
    print(f"✅ {filename} 생성 완료")

def create_manual_node(node_num):
    """수동 조종 노드 생성"""
    filename = f"manual_node_{node_num}.py"
    
    content = f'''#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from mavros_msgs.msg import ManualControl
import time
from frequency_config import get_node_frequency

class ManualNode{node_num}(Node):
    def __init__(self):
        super().__init__("manual_node_{node_num}")
        self.publisher = self.create_publisher(ManualControl, "/mavros/manual_control/send", 10)
        frequency = get_node_frequency("manual_node_{node_num}")
        self.timer = self.create_timer(frequency, self.timer_callback)
        self.get_logger().info(f"Manual Node {node_num} started at {{int(1/frequency):,}}Hz")
        
    def timer_callback(self):
        msg = ManualControl()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.x = {(node_num % 4 - 1.5) * 0.2}
        msg.y = {((node_num + 1) % 4 - 1.5) * 0.2}
        msg.z = {0.5 + (node_num % 3) * 0.1}
        msg.r = {(node_num % 2 - 0.5) * 0.1}
        msg.buttons = {node_num}
        
        self.publisher.publish(msg)

def main():
    rclpy.init()
    node = ManualNode{node_num}()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
'''
    
    with open(filename, 'w') as f:
        f.write(content)
    
    print(f"✅ {filename} 생성 완료")

def create_override_node(node_num):
    """RC 오버라이드 노드 생성"""
    filename = f"override_node_{node_num}.py"
    
    content = f'''#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from mavros_msgs.msg import OverrideRCIn
import time
from frequency_config import get_node_frequency

class OverrideNode{node_num}(Node):
    def __init__(self):
        super().__init__("override_node_{node_num}")
        self.publisher = self.create_publisher(OverrideRCIn, "/mavros/rc/override", 10)
        frequency = get_node_frequency("override_node_{node_num}")
        self.timer = self.create_timer(frequency, self.timer_callback)
        self.get_logger().info(f"Override Node {node_num} started at {{int(1/frequency):,}}Hz")
        
    def timer_callback(self):
        msg = OverrideRCIn()
        base_value = 1500 + (node_num % 3 - 1) * 100
        msg.channels = [
            base_value, 
            base_value + 50, 
            base_value - 50, 
            base_value + 25,
            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
        ]
        
        self.publisher.publish(msg)

def main():
    rclpy.init()
    node = OverrideNode{node_num}()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
'''
    
    with open(filename, 'w') as f:
        f.write(content)
    
    print(f"✅ {filename} 생성 완료")

def main():
    """메인 함수"""
    print("🚀 40개 노드를 생성합니다...")
    
    # 위치 명령 노드 8개 추가 (9~16)
    print("\n📍 위치 명령 노드 8개 추가 중...")
    for i in range(9, 17):
        create_position_node(i)
    
    # 수동 조종 노드 8개 추가 (9~16)
    print("\n🎮 수동 조종 노드 8개 추가 중...")
    for i in range(9, 17):
        create_manual_node(i)
    
    # RC 오버라이드 노드 4개 추가 (5~8)
    print("\n🎛️ RC 오버라이드 노드 4개 추가 중...")
    for i in range(5, 9):
        create_override_node(i)
    
    print("\n🎯 총 40개 노드 생성이 완료되었습니다!")
    print("이제 frequency_config.py를 업데이트하고 실행 스크립트를 수정해야 합니다.")

if __name__ == '__main__':
    main()
