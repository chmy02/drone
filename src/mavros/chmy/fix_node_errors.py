#!/usr/bin/env python3
"""
노드 파일들의 오류를 수정하는 스크립트
"""

import os
import re

def fix_override_node(filename, node_num):
    """RC 오버라이드 노드 파일 수정"""
    print(f"수정 중: {filename}")
    
    with open(filename, 'r') as f:
        content = f.read()
    
    # node_num 변수 정의 추가
    content = content.replace(
        '    def timer_callback(self):',
        f'    def timer_callback(self):\n        node_num = {node_num}'
    )
    
    with open(filename, 'w') as f:
        f.write(content)
    
    print(f"✅ {filename} 수정 완료")

def fix_manual_node(filename, node_num):
    """수동 조종 노드 파일 수정"""
    print(f"수정 중: {filename}")
    
    with open(filename, 'r') as f:
        content = f.read()
    
    # node_num 변수 정의 추가
    content = content.replace(
        '    def timer_callback(self):',
        f'    def timer_callback(self):\n        node_num = {node_num}'
    )
    
    with open(filename, 'w') as f:
        f.write(content)
    
    print(f"✅ {filename} 수정 완료")

def fix_position_node(filename, node_num):
    """위치 명령 노드 파일 수정"""
    print(f"수정 중: {filename}")
    
    with open(filename, 'r') as f:
        content = f.read()
    
    # node_num 변수 정의 추가
    content = content.replace(
        '    def timer_callback(self):',
        f'    def timer_callback(self):\n        node_num = {node_num}'
    )
    
    with open(filename, 'w') as f:
        f.write(content)
    
    print(f"✅ {filename} 수정 완료")

def main():
    """메인 함수"""
    print("🔧 노드 파일들의 오류를 수정합니다...")
    
    # RC 오버라이드 노드들 수정
    print("\n🎛️ RC 오버라이드 노드들 수정 중...")
    for i in range(1, 17):
        filename = f"override_node_{i}.py"
        if os.path.exists(filename):
            fix_override_node(filename, i)
    
    # 수동 조종 노드들 수정
    print("\n🎮 수동 조종 노드들 수정 중...")
    for i in range(1, 33):
        filename = f"manual_node_{i}.py"
        if os.path.exists(filename):
            fix_manual_node(filename, i)
    
    # 위치 명령 노드들 수정
    print("\n📍 위치 명령 노드들 수정 중...")
    for i in range(1, 33):
        filename = f"position_node_{i}.py"
        if os.path.exists(filename):
            fix_position_node(filename, i)
    
    print("\n🎯 모든 노드 파일 수정이 완료되었습니다!")

if __name__ == '__main__':
    main()
