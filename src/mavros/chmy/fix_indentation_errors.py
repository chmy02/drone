#!/usr/bin/env python3
"""
초기 노드 파일들의 들여쓰기 오류를 수정하는 스크립트
"""

import os
import re

def fix_indentation_error(filename):
    """들여쓰기 오류 수정"""
    print(f"수정 중: {filename}")
    
    with open(filename, 'r') as f:
        content = f.read()
    
    # 잘못된 들여쓰기 수정
    # "        frequency = get_node_frequency" 라인을 찾아서 수정
    lines = content.split('\n')
    fixed_lines = []
    
    for line in lines:
        # 잘못된 들여쓰기 라인 찾기
        if line.strip().startswith('frequency = get_node_frequency') and not line.startswith('        '):
            # 올바른 들여쓰기로 수정
            fixed_lines.append('        ' + line.strip())
        else:
            fixed_lines.append(line)
    
    content = '\n'.join(fixed_lines)
    
    with open(filename, 'w') as f:
        f.write(content)
    
    print(f"✅ {filename} 수정 완료")

def main():
    """메인 함수"""
    print("🔧 초기 노드 파일들의 들여쓰기 오류를 수정합니다...")
    
    # 수정할 파일들 (초기 생성된 노드들)
    files_to_fix = []
    
    # 위치 명령 노드 1~8
    for i in range(1, 9):
        files_to_fix.append(f"position_node_{i}.py")
    
    # 수동 조종 노드 1~8
    for i in range(1, 9):
        files_to_fix.append(f"manual_node_{i}.py")
    
    # RC 오버라이드 노드 1~4
    for i in range(1, 5):
        files_to_fix.append(f"override_node_{i}.py")
    
    # 각 파일 수정
    for filename in files_to_fix:
        if os.path.exists(filename):
            fix_indentation_error(filename)
        else:
            print(f"⚠️  {filename} 파일을 찾을 수 없습니다.")
    
    print("\n🎯 모든 초기 노드 파일 수정이 완료되었습니다!")

if __name__ == '__main__':
    main()
