#!/usr/bin/env python3
"""
모든 노드 파일을 주기 설정 파일을 사용하도록 업데이트하는 스크립트
"""

import os
import re

def update_node_file(filename, node_name):
    """노드 파일을 업데이트"""
    print(f"업데이트 중: {filename}")
    
    with open(filename, 'r') as f:
        content = f.read()
    
    # import 추가
    if 'from frequency_config import get_node_frequency' not in content:
        content = content.replace(
            'import time',
            'import time\nfrom frequency_config import get_node_frequency'
        )
    
    # timer 설정 부분 찾기
    timer_pattern = r'self\.timer = self\.create_timer\([^)]+\).*?self\.get_logger\(\)\.info\([^)]+\)'
    
    # 새로운 timer 설정
    new_timer = f'''        frequency = get_node_frequency("{node_name}")
        self.timer = self.create_timer(frequency, self.timer_callback)
        self.get_logger().info(f"{{node_name}} started at {{int(1/frequency):,}}Hz")'''
    
    # 기존 timer 설정을 새로운 것으로 교체
    content = re.sub(timer_pattern, new_timer, content, flags=re.DOTALL)
    
    with open(filename, 'w') as f:
        f.write(content)
    
    print(f"✅ {filename} 업데이트 완료")

def main():
    """메인 함수"""
    print("🚀 모든 노드 파일을 주기 설정 파일을 사용하도록 업데이트합니다...")
    
    # 업데이트할 노드 파일들
    nodes_to_update = [
        ("position_node_1.py", "position_node_1"),
        ("position_node_2.py", "position_node_2"),
        ("position_node_3.py", "position_node_3"),
        ("position_node_4.py", "position_node_4"),
        ("position_node_5.py", "position_node_5"),
        ("position_node_6.py", "position_node_6"),
        ("position_node_7.py", "position_node_7"),
        ("position_node_8.py", "position_node_8"),
        ("manual_node_1.py", "manual_node_1"),
        ("manual_node_2.py", "manual_node_2"),
        ("manual_node_3.py", "manual_node_3"),
        ("manual_node_4.py", "manual_node_4"),
        ("manual_node_5.py", "manual_node_5"),
        ("manual_node_6.py", "manual_node_6"),
        ("manual_node_7.py", "manual_node_7"),
        ("manual_node_8.py", "manual_node_8"),
        ("override_node_1.py", "override_node_1"),
        ("override_node_2.py", "override_node_2"),
        ("override_node_3.py", "override_node_3"),
        ("override_node_4.py", "override_node_4"),
    ]
    
    # 각 노드 파일 업데이트
    for filename, node_name in nodes_to_update:
        if os.path.exists(filename):
            update_node_file(filename, node_name)
        else:
            print(f"⚠️  {filename} 파일을 찾을 수 없습니다.")
    
    print("\n🎯 모든 노드 파일 업데이트가 완료되었습니다!")
    print("이제 frequency_config.py에서 주기를 변경하면 모든 노드에 적용됩니다.")

if __name__ == '__main__':
    main() 