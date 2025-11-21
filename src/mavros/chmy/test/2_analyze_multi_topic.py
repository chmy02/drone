#!/usr/bin/env python3
"""
다중 토픽 latency 로그 파일을 분석하는 스크립트
각 토픽별로 latency를 분석하고 통계를 출력합니다.
"""

import os
import sys
import glob
import pandas as pd
import numpy as np


def analyze_log_file(log_file):
    """단일 로그 파일 분석"""
    try:
        # CSV 형식 (9개 컬럼): node_id, msg_counter, publish_time_ns, callback_start_ns, send_complete_ns,
        #                       processing_latency_ns, processing_latency_us, total_latency_ns, total_latency_us
        # 주의: total_latency_us는 실제로 communication latency (t1→t3)
        df = pd.read_csv(log_file, names=['node_id', 'msg_counter', 'publish_time_ns', 
                                            'callback_start_ns', 'send_complete_ns',
                                            'processing_latency_ns', 'processing_latency_us',
                                            'total_latency_ns', 'total_latency_us'])
        
        if df.empty:
            return None
        
        # 초반 워밍업 메시지 제외 (각 노드별로)
        if len(df) > WARMUP_SKIP_MESSAGES:
            df_filtered = []
            for node_id in df['node_id'].unique():
                node_df = df[df['node_id'] == node_id]
                if len(node_df) > WARMUP_SKIP_MESSAGES:
                    df_filtered.append(node_df.iloc[WARMUP_SKIP_MESSAGES:])
                else:
                    df_filtered.append(node_df)  # 데이터가 적으면 전체 사용
            
            if df_filtered:
                df = pd.concat(df_filtered, ignore_index=True)
            
            if df.empty:
                return None
        
        # 용어 재정의:
        # - communication_latency (통신): t1→t3 (기존 total_latency)
        # - processing_latency (처리): t3→t4
        # - total_latency (전체): t1→t4 = communication + processing
        df['communication_latency_us'] = df['total_latency_us']  # t1→t3
        df['total_latency_real_us'] = df['total_latency_us'] + df['processing_latency_us']  # t1→t4
        
        # 전체 통계 (마이크로초 기준)
        stats = {
            'file': os.path.basename(log_file),
            'count': len(df),
            # 통신 레이턴시 (t1→t3)
            'comm_mean_us': df['communication_latency_us'].mean(),
            'comm_median_us': df['communication_latency_us'].median(),
            'comm_std_us': df['communication_latency_us'].std(),
            'comm_min_us': df['communication_latency_us'].min(),
            'comm_max_us': df['communication_latency_us'].max(),
            # 처리 레이턴시 (t3→t4)
            'proc_mean_us': df['processing_latency_us'].mean(),
            'proc_median_us': df['processing_latency_us'].median(),
            'proc_max_us': df['processing_latency_us'].max(),
            # 전체 레이턴시 (t1→t4)
            'total_mean_us': df['total_latency_real_us'].mean(),
            'total_median_us': df['total_latency_real_us'].median(),
            'total_std_us': df['total_latency_real_us'].std(),
            'total_min_us': df['total_latency_real_us'].min(),
            'total_max_us': df['total_latency_real_us'].max(),
            'total_p95_us': df['total_latency_real_us'].quantile(0.95),
            'total_p99_us': df['total_latency_real_us'].quantile(0.99),
        }
        
        # 토픽별 통계 (node_id 기준)
        by_topic = {}
        for node_id in df['node_id'].unique():
            topic_df = df[df['node_id'] == node_id]
            by_topic[node_id] = {
                'count': len(topic_df),
                # 통신 레이턴시
                'comm_mean_us': topic_df['communication_latency_us'].mean(),
                'comm_median_us': topic_df['communication_latency_us'].median(),
                # 처리 레이턴시
                'proc_mean_us': topic_df['processing_latency_us'].mean(),
                'proc_median_us': topic_df['processing_latency_us'].median(),
                # 전체 레이턴시
                'total_mean_us': topic_df['total_latency_real_us'].mean(),
                'total_median_us': topic_df['total_latency_real_us'].median(),
                'total_min_us': topic_df['total_latency_real_us'].min(),
                'total_max_us': topic_df['total_latency_real_us'].max(),
            }
        
        stats['by_topic'] = by_topic
        
        return stats
    
    except Exception as e:
        print(f"❌ 로그 파일 분석 실패: {e}")
        return None


def print_statistics(stats_list):
    """통계 출력"""
    print("\n" + "=" * 100)
    print("📊 다중 토픽 Latency 분석 결과")
    print("=" * 100)
    
    # 토픽 이름 매핑 (7개 - 재정렬 완료)
    topic_names = {
        1: "setpoint_raw/local (PositionTarget)",
        2: "setpoint_velocity/cmd_vel (TwistStamped)",
        3: "setpoint_position/local (PoseStamped)",
        4: "setpoint_raw/attitude (AttitudeTarget)",
        5: "setpoint_accel/accel (Vector3Stamped)",
        6: "vision_pose/pose (PoseStamped)",
        7: "mocap/pose (PoseStamped)",
    }
    
    for stats in stats_list:
        print(f"\n📄 파일: {stats['file']}")
        print(f"   총 메시지 수: {stats['count']}")
        print(f"\n   📊 레이턴시 분석:")
        print(f"      통신 레이턴시 (t1→t3): {stats['comm_mean_us']:7.1f} μs (평균), {stats['comm_median_us']:7.1f} μs (중앙값)")
        print(f"      처리 레이턴시 (t3→t4): {stats['proc_mean_us']:7.1f} μs (평균), {stats['proc_median_us']:7.1f} μs (중앙값)")
        print(f"      전체 레이턴시 (t1→t4): {stats['total_mean_us']:7.1f} μs (평균), {stats['total_median_us']:7.1f} μs (중앙값)")
        print(f"\n   📈 전체 레이턴시 상세:")
        print(f"      표준편차: {stats['total_std_us']:.1f} μs")
        print(f"      범위: {stats['total_min_us']:.1f} ~ {stats['total_max_us']:.1f} μs")
        print(f"      P95/P99: {stats['total_p95_us']:.1f} / {stats['total_p99_us']:.1f} μs")
        
        if stats['by_topic']:
            print(f"\n   📌 토픽별 통계:")
            for node_id in sorted(stats['by_topic'].keys()):
                topic_stats = stats['by_topic'][node_id]
                topic_name = topic_names.get(node_id, f"Unknown Topic {node_id}")
                print(f"      Topic {node_id} - {topic_name}")
                print(f"         메시지 수: {topic_stats['count']:4d}")
                print(f"         통신 (t1→t3): {topic_stats['comm_mean_us']:7.1f} μs (평균)")
                print(f"         처리 (t3→t4): {topic_stats['proc_mean_us']:7.1f} μs (평균)")
                print(f"         전체 (t1→t4): {topic_stats['total_mean_us']:7.1f} μs (평균)")
                print(f"         범위: {topic_stats['total_min_us']:7.1f} ~ {topic_stats['total_max_us']:7.1f} μs")
    
    print("\n" + "=" * 100)


def save_analysis(stats_list, output_file):
    """분석 결과 저장"""
    with open(output_file, 'w') as f:
        f.write("=" * 100 + "\n")
        f.write("다중 토픽 Latency 분석 결과\n")
        f.write("=" * 100 + "\n\n")
        
        topic_names = {
            1: "setpoint_raw/local",
            2: "setpoint_velocity/cmd_vel",
            3: "setpoint_position/local",
            4: "setpoint_raw/attitude",
            5: "setpoint_accel/accel",
            6: "vision_pose/pose",
            7: "mocap/pose",
        }
        
        for stats in stats_list:
            f.write(f"\n파일: {stats['file']}\n")
            f.write(f"총 메시지 수: {stats['count']}\n\n")
            f.write(f"레이턴시 분석:\n")
            f.write(f"  통신 레이턴시 (t1→t3): {stats['comm_mean_us']:.3f} μs (평균), {stats['comm_median_us']:.3f} μs (중앙값)\n")
            f.write(f"  처리 레이턴시 (t3→t4): {stats['proc_mean_us']:.3f} μs (평균), {stats['proc_median_us']:.3f} μs (중앙값)\n")
            f.write(f"  전체 레이턴시 (t1→t4): {stats['total_mean_us']:.3f} μs (평균), {stats['total_median_us']:.3f} μs (중앙값)\n\n")
            f.write(f"전체 레이턴시 상세:\n")
            f.write(f"  표준편차: {stats['total_std_us']:.3f} μs\n")
            f.write(f"  범위: {stats['total_min_us']:.3f} ~ {stats['total_max_us']:.3f} μs\n")
            f.write(f"  P95/P99: {stats['total_p95_us']:.3f} / {stats['total_p99_us']:.3f} μs\n")
            
            if stats['by_topic']:
                f.write(f"\n토픽별 통계:\n")
                for node_id in sorted(stats['by_topic'].keys()):
                    topic_stats = stats['by_topic'][node_id]
                    topic_name = topic_names.get(node_id, f"Unknown Topic {node_id}")
                    f.write(f"  Topic {node_id} - {topic_name}:\n")
                    f.write(f"    메시지 수: {topic_stats['count']}\n")
                    f.write(f"    통신 (t1→t3): {topic_stats['comm_mean_us']:.3f} μs (평균), {topic_stats['comm_median_us']:.3f} μs (중앙값)\n")
                    f.write(f"    처리 (t3→t4): {topic_stats['proc_mean_us']:.3f} μs (평균), {topic_stats['proc_median_us']:.3f} μs (중앙값)\n")
                    f.write(f"    전체 (t1→t4): {topic_stats['total_mean_us']:.3f} μs (평균), {topic_stats['total_median_us']:.3f} μs (중앙값)\n")
                    f.write(f"    범위: {topic_stats['total_min_us']:.3f} ~ {topic_stats['total_max_us']:.3f} μs\n")
            f.write("\n")
        
        f.write("=" * 100 + "\n")


def main():
    log_dir = "/home/rtcl-chmy/mavros_ws/src/mavros/chmy/logs"
    
    # 로그 파일 찾기 (새로운 파일명 규칙: YYYYMMDD_HHMMSS_topicN_*_latency.log)
    log_pattern = f"{log_dir}/*_topic*_*_latency.log"
    log_files = sorted(glob.glob(log_pattern), key=os.path.getmtime)
    
    if not log_files:
        print(f"❌ 로그 파일을 찾을 수 없습니다: {log_pattern}")
        return
    
    print(f"📁 {len(log_files)}개의 로그 파일을 찾았습니다.")
    
    # 모든 로그 파일 분석
    stats_list = []
    for log_file in log_files:
        print(f"📊 분석 중: {os.path.basename(log_file)}")
        stats = analyze_log_file(log_file)
        if stats:
            stats_list.append(stats)
    
    if not stats_list:
        print("❌ 분석할 데이터가 없습니다.")
        return
    
    # 결과 출력
    print_statistics(stats_list)
    
    # 결과 저장 - topic 1의 타임스탬프 추출
    topic1_file = None
    for log_file in log_files:
        if "_topic1_" in log_file:
            topic1_file = os.path.basename(log_file)
            break
    
    if topic1_file:
        # 파일명에서 타임스탬프 추출 (YYYYMMDD_HHMMSS)
        timestamp_str = "_".join(topic1_file.split("_")[:2])
        output_file = f"{log_dir}/{timestamp_str}_analysis.txt"
    else:
        # topic1 파일이 없으면 첫 번째 파일의 타임스탬프 사용
        first_file = os.path.basename(log_files[0])
        timestamp_str = "_".join(first_file.split("_")[:2])
        output_file = f"{log_dir}/{timestamp_str}_analysis.txt"
    
    save_analysis(stats_list, output_file)
    print(f"\n💾 분석 결과 저장: {output_file}")


if __name__ == "__main__":
    main()

