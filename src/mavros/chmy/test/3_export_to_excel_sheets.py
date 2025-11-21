#!/usr/bin/env python3
"""
7개 토픽 로그 파일을 하나의 엑셀 파일로 변환
각 토픽별로 별도 시트 생성 + 분석 요약 시트 포함 (총 8개 시트)
"""

import pandas as pd
import os
import glob
import numpy as np

# 로그 디렉토리
LOG_DIR = "/home/rtcl-chmy/mavros_ws/src/mavros/chmy/logs"

# 7개 토픽 정보 (번호 재정렬 완료)
TOPICS = [
    (1, "topic1_setpoint_raw_local", "Topic1_setpoint_raw_local"),
    (2, "topic2_setpoint_velocity_cmd_vel", "Topic2_setpoint_velocity"),
    (3, "topic3_setpoint_position_local", "Topic3_setpoint_position"),
    (4, "topic4_setpoint_raw_attitude", "Topic4_setpoint_raw_attitude"),
    (5, "topic5_setpoint_accel_accel", "Topic5_setpoint_accel"),
    (6, "topic6_vision_pose_pose", "Topic6_vision_pose"),
    (7, "topic7_mocap_pose", "Topic7_mocap_pose"),
]

def find_latest_logs():
    """가장 최근 날짜의 모든 로그 파일들을 찾습니다 (sequential test 대응)."""
    all_logs = glob.glob(os.path.join(LOG_DIR, "*_topic*_latency.log"))
    
    if not all_logs:
        print("❌ 로그 파일을 찾을 수 없습니다!")
        return None
    
    # 날짜별로 그룹화 (YYYYMMDD)
    dates = {}
    for log in all_logs:
        basename = os.path.basename(log)
        date = basename.split("_")[0]  # YYYYMMDD 추출
        if date not in dates:
            dates[date] = []
        dates[date].append(log)
    
    # 가장 최근 날짜 선택
    latest_date = sorted(dates.keys())[-1]
    log_files = dates[latest_date]
    
    # Topic 1의 타임스탬프를 찾아서 반환 (파일명 생성용)
    topic1_ts = None
    for log in log_files:
        if "_topic1_" in os.path.basename(log):
            topic1_ts = "_".join(os.path.basename(log).split("_")[:2])
            break
    
    # Topic 1이 없으면 가장 이른 타임스탬프 사용
    if not topic1_ts:
        timestamps = [os.path.basename(log).split("_topic")[0] for log in log_files]
        topic1_ts = sorted(timestamps)[0]
    
    return topic1_ts, log_files

def create_summary_sheet(topic_logs):
    """각 토픽의 통계 분석 결과를 DataFrame으로 생성"""
    summary_data = []
    
    topic_names = {
        1: "setpoint_raw/local",
        2: "setpoint_velocity/cmd_vel",
        3: "setpoint_position/local",
        4: "setpoint_raw/attitude",
        5: "setpoint_accel/accel",
        6: "vision_pose/pose",
        7: "mocap/pose",
    }
    
    for topic_id in sorted(topic_logs.keys()):
        log_path, _ = topic_logs[topic_id]
        
        try:
            # CSV 읽기
            df = pd.read_csv(
                log_path,
                names=['node_id', 'msg_counter', 'publish_time_ns', 
                       'callback_start_ns', 'send_complete_ns',
                       'processing_latency_ns', 'processing_latency_us',
                       'total_latency_ns', 'total_latency_us']
            )
            
            # 용어 재정의:
            # - communication_latency (통신): t1→t3 (기존 total_latency)
            # - processing_latency (처리): t3→t4
            # - total_latency (전체): t1→t4 = communication + processing
            df['communication_latency_us'] = df['total_latency_us']  # t1→t3
            df['total_latency_real_us'] = df['total_latency_us'] + df['processing_latency_us']  # t1→t4
            
            # 통계 계산
            summary_data.append({
                'Topic': f"Topic {topic_id}",
                'Topic Name': topic_names.get(topic_id, f"Unknown {topic_id}"),
                'Message Count': len(df),
                # 통신 레이턴시 (t1→t3)
                'Communication Mean (μs)': df['communication_latency_us'].mean(),
                'Communication Median (μs)': df['communication_latency_us'].median(),
                'Communication Min (μs)': df['communication_latency_us'].min(),
                'Communication Max (μs)': df['communication_latency_us'].max(),
                # 처리 레이턴시 (t3→t4)
                'Processing Mean (μs)': df['processing_latency_us'].mean(),
                'Processing Median (μs)': df['processing_latency_us'].median(),
                'Processing Max (μs)': df['processing_latency_us'].max(),
                # 전체 레이턴시 (t1→t4)
                'Total Mean (μs)': df['total_latency_real_us'].mean(),
                'Total Median (μs)': df['total_latency_real_us'].median(),
                'Total Std (μs)': df['total_latency_real_us'].std(),
                'Total Min (μs)': df['total_latency_real_us'].min(),
                'Total Max (μs)': df['total_latency_real_us'].max(),
                'Total P95 (μs)': df['total_latency_real_us'].quantile(0.95),
                'Total P99 (μs)': df['total_latency_real_us'].quantile(0.99),
            })
        except Exception as e:
            print(f"   ⚠️  Topic {topic_id} 통계 생성 실패: {e}")
            continue
    
    return pd.DataFrame(summary_data)

def main():
    result = find_latest_logs()
    if not result:
        return
    
    timestamp, log_files = result
    output_file = os.path.join(LOG_DIR, f"{timestamp}_latency_7topics.xlsx")
    
    print(f"📊 7개 토픽을 하나의 엑셀 파일(7개 시트)로 변환 중...")
    print(f"📅 타임스탬프: {timestamp}\n")
    
    # 토픽별로 로그 파일 매칭
    topic_logs = {}
    for log_file in log_files:
        for topic_id, pattern, sheet_name in TOPICS:
            if pattern in os.path.basename(log_file):
                topic_logs[topic_id] = (log_file, sheet_name)
                break
    
    # ExcelWriter 생성
    with pd.ExcelWriter(output_file, engine='openpyxl') as writer:
        # 1. Summary 시트 먼저 생성
        print(f"📊 분석 요약 시트 생성 중...")
        summary_df = create_summary_sheet(topic_logs)
        summary_df.to_excel(writer, sheet_name='Summary', index=False)
        print(f"   ✅ Summary 시트 생성 완료\n")
        
        # 2. 각 토픽별 Raw 데이터 시트 생성
        for topic_id in sorted(topic_logs.keys()):
            log_path, sheet_name = topic_logs[topic_id]
            
            print(f"📄 처리 중: {sheet_name}")
            
            try:
                # CSV 읽기 (헤더 없음, 9개 컬럼)
                df = pd.read_csv(
                    log_path,
                    names=['node_id', 'msg_counter', 'publish_time_ns', 
                           'callback_start_ns', 'send_complete_ns',
                           'processing_latency_ns', 'processing_latency_us',
                           'total_latency_ns', 'total_latency_us']
                )
                
                # 시트에 쓰기
                df.to_excel(writer, sheet_name=sheet_name, index=False)
                
                print(f"   ✅ {len(df):,}개 샘플 → 시트: {sheet_name}")
                
            except Exception as e:
                print(f"   ❌ 오류: {e}")
                continue
    
    print(f"\n✅ 엑셀 파일 생성 완료!")
    print(f"📁 파일: {output_file}")
    print(f"\n📊 생성된 시트 (총 {len(topic_logs) + 1}개):")
    print(f"   - Summary (분석 요약)")
    for topic_id in sorted(topic_logs.keys()):
        _, sheet_name = topic_logs[topic_id]
        print(f"   - {sheet_name}")
    print(f"\n💡 엑셀에서 열면 하단에 시트 탭이 보입니다!")

if __name__ == "__main__":
    main()

