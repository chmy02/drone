#!/usr/bin/env python3
"""
13개 토픽 로그 파일을 하나의 엑셀 파일로 변환
각 토픽별로 별도 시트 생성 + 분석 요약 시트 포함

출력 컬럼 (마이크로초 + CPU 사용량):
  - Node ID
  - Msg Counter
  - Comm Latency (t1→t3) μs: 통신 레이턴시
  - Proc Latency (t3→t4) μs: 처리 레이턴시
  - Total Latency (t1→t4) μs: 전체 레이턴시
  - CPU Total (%): 시스템 전체 CPU 사용량
  - CPU Gazebo (%): Gazebo 프로세스 CPU 사용량
  - CPU PX4 (%): PX4 프로세스 CPU 사용량
  - CPU MAVROS (%): MAVROS 프로세스 CPU 사용량
"""

import pandas as pd
import os
import glob
import numpy as np
import re
import shutil
from datetime import datetime

# 로그 디렉토리
LOG_DIR = "/home/rtcl-chmy/mavros_ws/src/mavros/chmy/logs"

# 13개 토픽 정보 (0: 서비스, 1~12: 토픽)
TOPICS = [
    # 서비스 콜
    (0, "topic0_command_arm", "Topic0_service_arm"),
    # 기본 플러그인 (mavros)
    (1, "topic1_setpoint_raw_local", "Topic1_raw_local"),
    (2, "topic2_setpoint_velocity_cmd_vel", "Topic2_velocity"),
    (3, "topic3_setpoint_position_local", "Topic3_position_local"),
    (4, "topic4_setpoint_raw_attitude", "Topic4_raw_attitude"),
    (5, "topic5_setpoint_accel_accel", "Topic5_accel"),
    (6, "topic6_setpoint_raw_global", "Topic6_raw_global"),
    (7, "topic7_setpoint_trajectory_local", "Topic7_trajectory"),
    (8, "topic8_manual_control_send", "Topic8_manual_control"),
    (9, "topic9_setpoint_position_global", "Topic9_position_global"),
    (10, "topic10_actuator_control", "Topic10_actuator"),
    # Extras 플러그인 (mavros_extras)
    (11, "topic11_vision_pose_pose", "Topic11_vision_pose"),
    (12, "topic12_mocap_pose", "Topic12_mocap_pose"),
]

def find_latest_logs():
    """가장 최근 실행 1회분의 토픽 로그/시스템 모니터 로그를 찾습니다."""
    all_logs = glob.glob(os.path.join(LOG_DIR, "*_topic*_latency.log"))

    if not all_logs:
        print("❌ 로그 파일을 찾을 수 없습니다!")
        return None, None, None

    # 토픽 로그 파일명 prefix(YYYYMMDD_HHMMSS) 기준으로 최근 실행 찾기
    run_timestamps = sorted(set(os.path.basename(log).split("_topic")[0] for log in all_logs))
    latest_ts = run_timestamps[-1]
    latest_date = latest_ts.split("_")[0]

    # 해당 실행과 같은 prefix를 가진 토픽 로그만 선택
    log_files = [
        log for log in all_logs
        if os.path.basename(log).startswith(f"{latest_ts}_topic")
    ]

    def topic_order(path):
        m = re.search(r"_topic(\d+)_", os.path.basename(path))
        return int(m.group(1)) if m else 999

    log_files = sorted(log_files, key=topic_order)

    # 같은 날짜 시스템 모니터 중, latest_ts와 시간이 가장 가까운 파일 선택
    sys_logs = sorted(
        glob.glob(os.path.join(LOG_DIR, f"{latest_date}_*_system_monitor.log")),
        reverse=True
    )
    system_monitor_log = None
    if sys_logs:
        base_dt = datetime.strptime(latest_ts, "%Y%m%d_%H%M%S")
        best_diff = None
        for path in sys_logs:
            sys_ts = os.path.basename(path).replace("_system_monitor.log", "")
            try:
                sys_dt = datetime.strptime(sys_ts, "%Y%m%d_%H%M%S")
                diff = abs((sys_dt - base_dt).total_seconds())
                if best_diff is None or diff < best_diff:
                    best_diff = diff
                    system_monitor_log = path
            except ValueError:
                continue

    return latest_ts, log_files, system_monitor_log


def move_files_to_run_folder(timestamp, files_to_move):
    """첫 로그 timestamp 기준 폴더(YYMMDD_HHMMSS)를 만들고 파일 이동"""
    folder_name = timestamp[2:]  # YYYYMMDD_HHMMSS -> YYMMDD_HHMMSS
    run_dir = os.path.join(LOG_DIR, folder_name)
    os.makedirs(run_dir, exist_ok=True)

    moved = []
    for src in files_to_move:
        if not src or not os.path.exists(src):
            continue
        dst = os.path.join(run_dir, os.path.basename(src))
        if os.path.abspath(src) == os.path.abspath(dst):
            continue
        if os.path.exists(dst):
            os.remove(dst)
        shutil.move(src, dst)
        moved.append(dst)
    return run_dir, moved

def create_summary_sheet(topic_logs):
    """각 토픽의 통계 분석 결과를 DataFrame으로 생성"""
    summary_data = []
    
    topic_names = {
        # 서비스 콜
        0: "cmd/arming (service)",
        # 기본 플러그인 (mavros)
        1: "setpoint_raw/local",
        2: "setpoint_velocity/cmd_vel",
        3: "setpoint_position/local",
        4: "setpoint_raw/attitude",
        5: "setpoint_accel/accel",
        6: "setpoint_raw/global",
        7: "setpoint_trajectory/local",
        8: "manual_control/send",
        9: "setpoint_position/global",
        10: "actuator_control",
        # Extras 플러그인 (mavros_extras)
        11: "vision_pose/pose",
        12: "mocap/pose",
    }
    
    for topic_id in sorted(topic_logs.keys()):
        log_path, _ = topic_logs[topic_id]
        
        try:
            # 새 CSV 형식 (9개 컬럼): node_id, msg_counter, t1_t3_us, t3_t4_us, t1_t4_us, cpu_total, cpu_gz, cpu_px4, cpu_mav
            df = pd.read_csv(
                log_path,
                names=['node_id', 'msg_counter', 't1_t3_us', 't3_t4_us', 't1_t4_us',
                       'cpu_total', 'cpu_gz', 'cpu_px4', 'cpu_mav']
            )
            
            # 통계 계산
            summary_data.append({
                'Topic': f"Topic {topic_id}",
                'Topic Name': topic_names.get(topic_id, f"Unknown {topic_id}"),
                'Message Count': len(df),
                # 통신 레이턴시 (t1→t3)
                'Communication Mean (μs)': df['t1_t3_us'].mean(),
                'Communication Median (μs)': df['t1_t3_us'].median(),
                'Communication Min (μs)': df['t1_t3_us'].min(),
                'Communication Max (μs)': df['t1_t3_us'].max(),
                # 처리 레이턴시 (t3→t4)
                'Processing Mean (μs)': df['t3_t4_us'].mean(),
                'Processing Median (μs)': df['t3_t4_us'].median(),
                'Processing Max (μs)': df['t3_t4_us'].max(),
                # 전체 레이턴시 (t1→t4)
                'Total Mean (μs)': df['t1_t4_us'].mean(),
                'Total Median (μs)': df['t1_t4_us'].median(),
                'Total Std (μs)': df['t1_t4_us'].std(),
                'Total Min (μs)': df['t1_t4_us'].min(),
                'Total Max (μs)': df['t1_t4_us'].max(),
                'Total P95 (μs)': df['t1_t4_us'].quantile(0.95),
                'Total P99 (μs)': df['t1_t4_us'].quantile(0.99),
                # CPU 사용량
                'CPU Total Mean (%)': df['cpu_total'].mean(),
                'CPU Gazebo Mean (%)': df['cpu_gz'].mean(),
                'CPU PX4 Mean (%)': df['cpu_px4'].mean(),
                'CPU MAVROS Mean (%)': df['cpu_mav'].mean(),
            })
        except Exception as e:
            print(f"   ⚠️  Topic {topic_id} 통계 생성 실패: {e}")
            continue
    
    return pd.DataFrame(summary_data)

def parse_system_monitor_log(log_path, top_n=20):
    """시스템 모니터 로그 파싱 → DataFrame (timestamp_ns, cpu_total, Top1..TopN)"""
    rows = []
    with open(log_path) as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            parts = line.split(",", 2)  # max 3 parts: ts, cpu_total, procs
            if len(parts) < 2:
                continue
            ts_ns = parts[0]
            cpu_total = float(parts[1]) if parts[1] else 0.0
            procs = []
            if len(parts) >= 3 and parts[2]:
                procs = [p.strip() for p in parts[2].split(";") if p.strip()]
            row = {"Timestamp (ns)": ts_ns, "CPU Total (%)": cpu_total}
            for i in range(top_n):
                row[f"Top{i+1}"] = procs[i] if i < len(procs) else ""
            rows.append(row)
    return pd.DataFrame(rows)


def main():
    result = find_latest_logs()
    if not result or result[0] is None:
        return

    timestamp, log_files, system_monitor_log = result
    output_file = os.path.join(LOG_DIR, f"{timestamp}_latency_13topics.xlsx")
    
    print(f"📊 13개 토픽(서비스 1개 + 토픽 12개)을 하나의 엑셀 파일로 변환 중...")
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
        
        # 2. 각 토픽별 Raw 데이터 시트 생성 (마이크로초 + CPU)
        for topic_id in sorted(topic_logs.keys()):
            log_path, sheet_name = topic_logs[topic_id]
            
            print(f"📄 처리 중: {sheet_name}")
            
            try:
                # 새 CSV 형식 (9개 컬럼): node_id, msg_counter, t1_t3_us, t3_t4_us, t1_t4_us, cpu_total, cpu_gz, cpu_px4, cpu_mav
                df = pd.read_csv(
                    log_path,
                    names=['node_id', 'msg_counter', 't1_t3_us', 't3_t4_us', 't1_t4_us',
                           'cpu_total', 'cpu_gz', 'cpu_px4', 'cpu_mav']
                )
                
                # 컬럼명 변경 (더 명확하게)
                df.columns = ['Node ID', 'Msg Counter', 
                             'Comm Latency (t1→t3) μs',
                             'Proc Latency (t3→t4) μs', 
                             'Total Latency (t1→t4) μs',
                             'CPU Total (%)', 'CPU Gazebo (%)', 'CPU PX4 (%)', 'CPU MAVROS (%)']
                
                # 시트에 쓰기
                df.to_excel(writer, sheet_name=sheet_name, index=False)
                
                print(f"   ✅ {len(df):,}개 샘플 → 시트: {sheet_name}")
                
            except Exception as e:
                print(f"   ❌ 오류: {e}")
                continue

        # 3. 시스템 모니터 시트 (있을 경우)
        if system_monitor_log and os.path.exists(system_monitor_log):
            print(f"\n📄 처리 중: System_Monitor")
            try:
                sys_df = parse_system_monitor_log(system_monitor_log, top_n=20)
                sys_df.to_excel(writer, sheet_name="System_Monitor", index=False)
                print(f"   ✅ {len(sys_df):,}개 샘플 → 시트: System_Monitor")
            except Exception as e:
                print(f"   ❌ 시스템 모니터 오류: {e}")

    print(f"\n✅ 엑셀 파일 생성 완료!")
    print(f"📁 파일: {output_file}")
    n_sheets = len(topic_logs) + 1 + (1 if system_monitor_log and os.path.exists(system_monitor_log) else 0)
    print(f"\n📊 생성된 시트 (총 {n_sheets}개):")
    print(f"   - Summary (분석 요약)")
    for topic_id in sorted(topic_logs.keys()):
        _, sheet_name = topic_logs[topic_id]
        print(f"   - {sheet_name}")
    if system_monitor_log and os.path.exists(system_monitor_log):
        print(f"   - System_Monitor")
    print(f"\n💡 엑셀에서 열면 하단에 시트 탭이 보입니다!")

    # 4. 결과 파일 정리: 첫 로그 timestamp 폴더로 이동
    related_files = list(topic_logs[tid][0] for tid in topic_logs.keys())
    related_files.append(output_file)
    if system_monitor_log:
        related_files.append(system_monitor_log)
    analysis_file = os.path.join(LOG_DIR, f"{timestamp}_analysis.txt")
    if os.path.exists(analysis_file):
        related_files.append(analysis_file)

    run_dir, moved_files = move_files_to_run_folder(timestamp, related_files)
    print(f"\n📦 결과 정리 완료: {run_dir}")
    print(f"   - 이동 파일 수: {len(moved_files)}")

if __name__ == "__main__":
    main()

