#!/usr/bin/env python3
"""
원하는 토픽들을 동시에 실행하는 스크립트
커맨드 라인 인자로 토픽 번호를 지정할 수 있습니다.

사용법:
  python3 0_run_multi_topic.py 1 3 5        # Topic 1, 3, 5를 동시 실행
  python3 0_run_multi_topic.py 1-3          # Topic 1, 2, 3을 동시 실행
  python3 0_run_multi_topic.py 1 2 4-7      # Topic 1, 2, 4, 5, 6, 7을 동시 실행
  python3 0_run_multi_topic.py              # 전체 7개 토픽 동시 실행
"""

import subprocess
import time
import sys
import os
import signal

# 설정
NODE_SCRIPT_DIR = "/home/rtcl-chmy/mavros_ws/src/mavros/chmy/nodes"
MAVROS_WS = "/home/rtcl-chmy/mavros_ws"
TEST_DURATION = 60  # 초 (1분)

# 7개 노드 스크립트 목록 (번호 재정렬 완료)
NODE_SCRIPTS = {
    1: "setpoint_raw_local.py",           # Topic 1
    2: "setpoint_velocity_cmd_vel.py",    # Topic 2
    3: "setpoint_position_local.py",      # Topic 3
    4: "setpoint_raw_attitude.py",        # Topic 4
    5: "setpoint_accel_accel.py",         # Topic 5
    6: "vision_pose_pose.py",             # Topic 6
    7: "mocap_pose.py",                   # Topic 7
}

TOPIC_NAMES = {
    1: "setpoint_raw/local",
    2: "setpoint_velocity/cmd_vel",
    3: "setpoint_position/local",
    4: "setpoint_raw/attitude",
    5: "setpoint_accel/accel",
    6: "vision_pose/pose",
    7: "mocap/pose",
}

# 실행된 프로세스들을 저장할 리스트
processes = []
mavros_process = None


def signal_handler(sig, frame):
    """Ctrl+C 처리"""
    print("\n\n🛑 실험 중단 중...")
    cleanup()
    sys.exit(0)


def start_mavros():
    """MAVROS 시작"""
    global mavros_process
    
    print("\n🚀 MAVROS 시작 중...")
    
    # MAVROS 실행 명령
    mavros_cmd = f"cd {MAVROS_WS} && source install/setup.bash && ros2 run mavros mavros_node --ros-args -p fcu_url:=udp://:14540@127.0.0.1:14557"
    
    try:
        mavros_process = subprocess.Popen(
            mavros_cmd,
            shell=True,
            executable='/bin/bash',
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            preexec_fn=os.setsid  # 새 프로세스 그룹 생성
        )
        
        print("   ⏳ MAVROS 초기화 대기 중 (5초)...")
        time.sleep(5)
        
        # MAVROS가 정상 실행 중인지 확인
        if mavros_process.poll() is None:
            print("   ✅ MAVROS 실행 완료\n")
            return True
        else:
            print("   ❌ MAVROS 실행 실패")
            return False
    except Exception as e:
        print(f"   ❌ MAVROS 시작 오류: {e}")
        return False


def stop_mavros():
    """MAVROS 종료"""
    global mavros_process
    
    if mavros_process is None:
        return
    
    print("\n🛑 MAVROS 종료 중...")
    
    try:
        # 프로세스 그룹 전체 종료 (자식 프로세스도 함께)
        os.killpg(os.getpgid(mavros_process.pid), signal.SIGTERM)
        mavros_process.wait(timeout=5)
        print("   ✅ MAVROS 종료 완료")
    except Exception as e:
        try:
            # 강제 종료
            os.killpg(os.getpgid(mavros_process.pid), signal.SIGKILL)
            print("   ✅ MAVROS 강제 종료 완료")
        except:
            print(f"   ⚠️  MAVROS 종료 중 오류: {e}")
    
    mavros_process = None


def cleanup():
    """모든 프로세스 종료"""
    print("🧹 모든 노드 종료 중...")
    for proc in processes:
        try:
            proc.terminate()
            proc.wait(timeout=2)
        except:
            proc.kill()
    print("✅ 모든 노드 종료 완료")
    
    # MAVROS 종료
    stop_mavros()


def parse_topic_selection(args):
    """명령줄 인자에서 토픽 번호 파싱 (입력 순서 유지)"""
    if not args:
        # 인자 없으면 전체 토픽 (1~7)
        return list(range(1, 8))
    
    selected = []
    seen = set()  # 중복 제거용
    
    for arg in args:
        if '-' in arg:
            # 범위 지정 (예: 1-5)
            try:
                start, end = map(int, arg.split('-'))
                for t in range(start, end + 1):
                    if 1 <= t <= 7 and t not in seen:
                        selected.append(t)
                        seen.add(t)
            except:
                print(f"⚠️  잘못된 범위 형식: {arg}")
        else:
            # 개별 번호 (예: 1, 2, 3)
            try:
                t = int(arg)
                if 1 <= t <= 7 and t not in seen:
                    selected.append(t)
                    seen.add(t)
            except:
                print(f"⚠️  잘못된 토픽 번호: {arg}")
    
    return selected


def run_experiment(selected_topics):
    """실험 실행"""
    print("=" * 80)
    print("🚀 다중 토픽 동시 실행 Latency 측정")
    print("=" * 80)
    print(f"📊 실험 설정:")
    print(f"   - 선택된 토픽: {selected_topics}")
    print(f"   - 노드 개수: {len(selected_topics)}개 (동시 실행)")
    print(f"   - 발행 주기: 10Hz (각 노드)")
    print(f"   - 실험 시간: {TEST_DURATION}초")
    print(f"\n📋 실행할 토픽:")
    for topic_num in selected_topics:
        print(f"   Topic {topic_num}: {TOPIC_NAMES[topic_num]}")
    print("=" * 80)
    
    # 1. MAVROS 시작
    if not start_mavros():
        print("❌ MAVROS 시작 실패. 실험을 중단합니다.")
        return
    
    # 2. 선택된 노드 동시 실행
    print(f"\n📝 {len(selected_topics)}개 노드 동시 실행 중...")
    for topic_num in selected_topics:
        script = NODE_SCRIPTS[topic_num]
        script_path = os.path.join(NODE_SCRIPT_DIR, script)
        
        if not os.path.exists(script_path):
            print(f"❌ 파일을 찾을 수 없습니다: {script_path}")
            continue
        
        try:
            proc = subprocess.Popen(
                ['python3', script_path],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE
            )
            processes.append(proc)
            print(f"   ✅ Topic {topic_num} 시작: {TOPIC_NAMES[topic_num]}")
            time.sleep(0.1)  # 약간의 지연
        except Exception as e:
            print(f"   ❌ Topic {topic_num} 실행 실패: {e}")
    
    print(f"\n✅ 총 {len(processes)}개 노드 동시 실행 완료\n")
    
    # 3. 실험 진행
    print(f"⏱️  {TEST_DURATION}초 동안 latency 측정 중...\n")
    for remaining in range(TEST_DURATION, 0, -10):
        print(f"   ⏳ 남은 시간: {remaining}초...")
        time.sleep(10)
    
    print("\n✅ 실험 완료!\n")
    
    # 4. 노드 및 MAVROS 종료
    cleanup()
    
    # 5. 로그 파일 확인
    print("\n📁 로그 파일 확인 중...")
    log_dir = "/home/rtcl-chmy/mavros_ws/src/mavros/chmy/logs"
    
    if os.path.exists(log_dir):
        import glob
        log_files = sorted(glob.glob(os.path.join(log_dir, "*_topic*_latency.log")), 
                          key=os.path.getmtime, reverse=True)
        if log_files:
            print(f"✅ 로그 파일 생성됨 (최근 {min(len(log_files), 10)}개):")
            for log_file in log_files[:10]:
                size = os.path.getsize(log_file)
                basename = os.path.basename(log_file)
                print(f"   - {basename} ({size:,} bytes)")
            print(f"\n💡 다음 단계:")
            print(f"   1. 분석: python3 /home/rtcl-chmy/mavros_ws/src/mavros/chmy/test/2_analyze_multi_topic.py")
            print(f"   2. 엑셀: python3 /home/rtcl-chmy/mavros_ws/src/mavros/chmy/test/3_export_to_excel_sheets.py")
        else:
            print("❌ 로그 파일이 생성되지 않았습니다.")
    else:
        print(f"❌ 로그 디렉토리가 없습니다: {log_dir}")


def main():
    # Ctrl+C 핸들러 등록
    signal.signal(signal.SIGINT, signal_handler)
    
    # 명령줄 인자 파싱
    selected_topics = parse_topic_selection(sys.argv[1:])
    
    if not selected_topics:
        print("❌ 유효한 토픽을 선택하세요 (1~7)")
        print("\n사용법:")
        print("  python3 0_run_multi_topic.py              # 전체 7개 토픽 동시 실행")
        print("  python3 0_run_multi_topic.py 1            # 토픽 1만 실행")
        print("  python3 0_run_multi_topic.py 1 3 5        # 토픽 1, 3, 5 동시 실행")
        print("  python3 0_run_multi_topic.py 1-3          # 토픽 1~3 동시 실행")
        print("  python3 0_run_multi_topic.py 1 2 4-7      # 토픽 1, 2, 4~7 동시 실행")
        print("\n📋 사용 가능한 토픽 (1~7):")
        for topic_num, topic_name in TOPIC_NAMES.items():
            print(f"  {topic_num}. {topic_name}")
        sys.exit(1)
    
    try:
        run_experiment(selected_topics)
    except Exception as e:
        print(f"\n❌ 오류 발생: {e}")
        cleanup()
        sys.exit(1)


if __name__ == "__main__":
    main()

