#!/usr/bin/env python3
"""
frequency_config.py를 80개 노드로 업데이트
"""

def update_frequency_config():
    """frequency_config.py 업데이트"""
    
    content = '''#!/usr/bin/env python3
"""
주기 설정 파일
모든 노드의 주기를 이 파일에서 중앙 관리합니다.
"""

# ============================================================================
# 주기 설정 (초 단위)
# ============================================================================

# 기본 주기 (1000Hz)
DEFAULT_FREQUENCY = 0.001

# 특별 주기 (극한 테스트용)
EXTREME_FREQUENCY = 0.00001  # 100,000Hz

# ============================================================================
# 노드별 주기 설정
# ============================================================================

# 위치 명령 노드들 (32개)
POSITION_NODE_FREQUENCIES = {
'''
    
    # 위치 명령 노드 32개
    for i in range(1, 33):
        content += f'    "position_node_{i}": DEFAULT_FREQUENCY,   # 1000Hz\n'
    
    content += '''}

# 수동 조종 노드들 (32개)
MANUAL_NODE_FREQUENCIES = {
'''
    
    # 수동 조종 노드 32개
    for i in range(1, 33):
        content += f'    "manual_node_{i}": DEFAULT_FREQUENCY,    # 1000Hz\n'
    
    content += '''}

# RC 오버라이드 노드들 (16개)
OVERRIDE_NODE_FREQUENCIES = {
'''
    
    # RC 오버라이드 노드 16개
    for i in range(1, 17):
        content += f'    "override_node_{i}": DEFAULT_FREQUENCY,  # 1000Hz\n'
    
    content += '''}

# ============================================================================
# 주기 변경 함수들
# ============================================================================

def set_all_frequencies_to_1000hz():
    """모든 노드를 1000Hz로 설정"""
    global DEFAULT_FREQUENCY
    DEFAULT_FREQUENCY = 0.001
    
    # 모든 노드를 기본 주기로 설정
    for key in POSITION_NODE_FREQUENCIES:
        POSITION_NODE_FREQUENCIES[key] = DEFAULT_FREQUENCY
    
    for key in MANUAL_NODE_FREQUENCIES:
        MANUAL_NODE_FREQUENCIES[key] = DEFAULT_FREQUENCY
    
    for key in OVERRIDE_NODE_FREQUENCIES:
        OVERRIDE_NODE_FREQUENCIES[key] = DEFAULT_FREQUENCY

def set_all_frequencies_to_10000hz():
    """모든 노드를 10,000Hz로 설정"""
    global DEFAULT_FREQUENCY
    DEFAULT_FREQUENCY = 0.0001
    
    # 모든 노드를 고주기로 설정
    for key in POSITION_NODE_FREQUENCIES:
        POSITION_NODE_FREQUENCIES[key] = DEFAULT_FREQUENCY
    
    for key in MANUAL_NODE_FREQUENCIES:
        MANUAL_NODE_FREQUENCIES[key] = DEFAULT_FREQUENCY
    
    for key in OVERRIDE_NODE_FREQUENCIES:
        OVERRIDE_NODE_FREQUENCIES[key] = DEFAULT_FREQUENCY

def set_extreme_frequency_for_node(node_name, frequency):
    """특정 노드만 극한 주기로 설정"""
    if node_name in POSITION_NODE_FREQUENCIES:
        POSITION_NODE_FREQUENCIES[node_name] = frequency
    elif node_name in MANUAL_NODE_FREQUENCIES:
        MANUAL_NODE_FREQUENCIES[node_name] = frequency
    elif node_name in OVERRIDE_NODE_FREQUENCIES:
        OVERRIDE_NODE_FREQUENCIES[node_name] = frequency

def get_node_frequency(node_name):
    """노드의 주기 반환"""
    if node_name in POSITION_NODE_FREQUENCIES:
        return POSITION_NODE_FREQUENCIES[node_name]
    elif node_name in MANUAL_NODE_FREQUENCIES:
        return MANUAL_NODE_FREQUENCIES[node_name]
    elif node_name in OVERRIDE_NODE_FREQUENCIES:
        return OVERRIDE_NODE_FREQUENCIES[node_name]
    else:
        return DEFAULT_FREQUENCY

def print_current_frequencies():
    """현재 모든 노드의 주기 출력"""
    print("\\n=== 현재 노드별 주기 설정 ===")
    
    print("\\n📍 위치 명령 노드들:")
    for node, freq in POSITION_NODE_FREQUENCIES.items():
        hz = int(1 / freq)
        print(f"  {node}: {freq:.6f}초 ({hz:,}Hz)")
    
    print("\\n🎮 수동 조종 노드들:")
    for node, freq in MANUAL_NODE_FREQUENCIES.items():
        hz = int(1 / freq)
        print(f"  {node}: {freq:.6f}초 ({hz:,}Hz)")
    
    print("\\n🎛️ RC 오버라이드 노드들:")
    for node, freq in OVERRIDE_NODE_FREQUENCIES.items():
        hz = int(1 / freq)
        print(f"  {node}: {freq:.6f}초 ({hz:,}Hz)")
    
    print("\\n===============================")
'''
    
    with open('frequency_config.py', 'w') as f:
        f.write(content)
    
    print("✅ frequency_config.py 업데이트 완료 (80개 노드)")

if __name__ == '__main__':
    update_frequency_config()
