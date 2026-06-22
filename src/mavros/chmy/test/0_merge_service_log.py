#!/usr/bin/env python3
"""
서비스 로그 병합 스크립트
Python 노드의 t1 로그와 MAVROS 플러그인의 t3, t4 로그를 병합합니다.

사용법:
  python3 0_merge_service_log.py [타임스탬프]
  
예시:
  python3 0_merge_service_log.py                    # 최신 로그 자동 병합
  python3 0_merge_service_log.py 20260111_223749    # 특정 타임스탬프 로그 병합
"""

import os
import sys
import glob
from datetime import datetime


LOG_DIR = "/home/rtcl-chmy/mavros_ws/src/mavros/chmy/logs"


def find_latest_logs():
    """가장 최근 로그 파일들을 찾습니다 (타임스탬프가 달라도 가장 최근 파일 매칭)."""
    t1_pattern = os.path.join(LOG_DIR, "*_topic0_command_arm_t1.log")
    latency_pattern = os.path.join(LOG_DIR, "*_topic0_command_arm_latency.log")
    
    t1_files = glob.glob(t1_pattern)
    latency_files = glob.glob(latency_pattern)
    
    if not t1_files or not latency_files:
        print("❌ 로그 파일을 찾을 수 없습니다!")
        return None, None, None
    
    # 수정 시간 기준으로 정렬 (가장 최근 파일)
    t1_files_sorted = sorted(t1_files, key=os.path.getmtime, reverse=True)
    latency_files_sorted = sorted(latency_files, key=os.path.getmtime, reverse=True)
    
    # 가장 최근 파일 선택
    latest_t1_file = t1_files_sorted[0]
    latest_latency_file = latency_files_sorted[0]
    
    # 타임스탬프 추출 (출력용)
    t1_basename = os.path.basename(latest_t1_file)
    latency_basename = os.path.basename(latest_latency_file)
    t1_timestamp = "_".join(t1_basename.split("_")[:2])
    latency_timestamp = "_".join(latency_basename.split("_")[:2])
    
    if t1_timestamp != latency_timestamp:
        print(f"⚠️  타임스탬프가 다릅니다:")
        print(f"   t1 로그: {t1_timestamp}")
        print(f"   latency 로그: {latency_timestamp}")
        print(f"   → 가장 최근 파일들을 매칭합니다.")
    
    # latency 파일의 타임스탬프를 기준으로 사용 (출력 파일명용)
    return latency_timestamp, latest_t1_file, latest_latency_file


def merge_logs(t1_file, latency_file, output_file):
    """t1 로그와 latency 로그를 병합합니다."""
    
    # t1 로그 파일 읽기 (msg_counter -> t1 매핑)
    t1_map = {}
    try:
        with open(t1_file, 'r') as f:
            for line in f:
                line = line.strip()
                if not line:
                    continue
                parts = line.split(',')
                if len(parts) >= 2:
                    msg_counter = int(parts[0])
                    t1_ns = int(parts[1])
                    t1_map[msg_counter] = t1_ns
    except Exception as e:
        print(f"❌ t1 로그 파일 읽기 실패: {e}")
        return False
    
    print(f"📖 t1 로그 파일 읽기 완료: {len(t1_map)}개 항목")
    
    # latency 로그 파일 읽기 및 병합
    merged_lines = []
    missing_t1_count = 0
    offset_calculated = False
    clock_offset_ns = 0
    
    try:
        with open(latency_file, 'r') as f:
            for line in f:
                line = line.strip()
                if not line:
                    continue
                parts = line.split(',')
                if len(parts) >= 9:
                    node_id = parts[0]
                    msg_counter = int(parts[1])
                    t1_old = parts[2]  # 기존 t1 (무시)
                    t3_ns = int(parts[3])
                    t4_ns = int(parts[4])
                    proc_ns = int(parts[5])
                    proc_us = float(parts[6])
                    total_ns = int(parts[7])
                    total_us = float(parts[8])
                    
                    # t1 찾기
                    if msg_counter in t1_map:
                        t1_ns = t1_map[msg_counter]
                        
                        # 첫 번째 매칭 항목에서 시계 오프셋 계산
                        if not offset_calculated:
                            # t1과 t3의 차이를 오프셋으로 사용
                            # t1이 t3보다 크면 Python 노드 시계가 앞서 있음
                            clock_offset_ns = t3_ns - t1_ns
                            print(f"📐 시계 오프셋 계산: {clock_offset_ns / 1e6:.2f} ms")
                            offset_calculated = True
                        
                        # 시계 오프셋 적용
                        t1_adjusted = t1_ns + clock_offset_ns
                        
                        # t1_adjusted가 t3보다 크면, t1 = t3로 설정 (최소값 보장)
                        if t1_adjusted > t3_ns:
                            t1_adjusted = t3_ns
                        
                        # 통신 레이턴시 (t1→t3) 재계산
                        communication_latency_ns = t3_ns - t1_adjusted
                        communication_latency_us = communication_latency_ns / 1000.0
                        
                        # 전체 레이턴시 (t1→t4) 재계산
                        total_latency_ns = t4_ns - t1_adjusted
                        total_latency_us = total_latency_ns / 1000.0
                        
                        # 병합된 라인 생성
                        merged_line = f"{node_id},{msg_counter},{t1_adjusted},{t3_ns},{t4_ns}," \
                                     f"{proc_ns},{proc_us:.3f}," \
                                     f"{total_latency_ns},{total_latency_us:.3f}\n"
                        merged_lines.append(merged_line)
                    else:
                        missing_t1_count += 1
                        # t1을 찾을 수 없으면 기존 데이터 사용 (t1 = t3)
                        merged_line = f"{node_id},{msg_counter},{t3_ns},{t3_ns},{t4_ns}," \
                                     f"{proc_ns},{proc_us:.3f}," \
                                     f"{total_latency_ns},{total_latency_us:.3f}\n"
                        merged_lines.append(merged_line)
    except Exception as e:
        print(f"❌ latency 로그 파일 읽기 실패: {e}")
        return False
    
    if missing_t1_count > 0:
        print(f"⚠️  t1을 찾을 수 없는 항목: {missing_t1_count}개")
    
    # 병합된 로그 파일 저장
    try:
        # 백업 파일 생성
        backup_file = latency_file + ".backup"
        if os.path.exists(latency_file):
            os.rename(latency_file, backup_file)
            print(f"📋 백업 파일 생성: {backup_file}")
        
        # 병합된 로그 파일 저장
        with open(output_file, 'w') as f:
            f.writelines(merged_lines)
        
        print(f"✅ 병합 완료: {len(merged_lines)}개 항목 → {output_file}")
        return True
    except Exception as e:
        print(f"❌ 병합된 로그 파일 저장 실패: {e}")
        # 백업에서 복원
        if os.path.exists(backup_file):
            os.rename(backup_file, latency_file)
        return False


def main():
    """메인 함수"""
    timestamp = None
    
    if len(sys.argv) > 1:
        timestamp = sys.argv[1]
        print(f"📅 지정된 타임스탬프: {timestamp}")
        
        t1_file = os.path.join(LOG_DIR, f"{timestamp}_topic0_command_arm_t1.log")
        latency_file = os.path.join(LOG_DIR, f"{timestamp}_topic0_command_arm_latency.log")
        
        if not os.path.exists(t1_file):
            print(f"❌ t1 로그 파일을 찾을 수 없습니다: {t1_file}")
            return
        if not os.path.exists(latency_file):
            print(f"❌ latency 로그 파일을 찾을 수 없습니다: {latency_file}")
            return
    else:
        print("🔍 최신 로그 파일 검색 중...")
        timestamp, t1_file, latency_file = find_latest_logs()
        
        if not timestamp:
            return
        
        print(f"📅 발견된 타임스탬프: {timestamp}")
    
    print(f"📁 t1 로그 파일: {os.path.basename(t1_file)}")
    print(f"📁 latency 로그 파일: {os.path.basename(latency_file)}")
    print()
    
    # 출력 파일 (latency 파일을 덮어쓰기)
    output_file = latency_file
    
    # 병합 수행
    success = merge_logs(t1_file, latency_file, output_file)
    
    if success:
        print(f"\n✅ 병합 완료!")
        print(f"📁 병합된 파일: {output_file}")
    else:
        print(f"\n❌ 병합 실패!")
        sys.exit(1)


if __name__ == "__main__":
    main()

