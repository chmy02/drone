#!/usr/bin/env python3
"""
13개의 서로 다른 토픽을 사용하는 노드를 선택적으로 실행하는 스크립트
각 노드는 10Hz로 메시지를 발행하며, latency를 측정합니다.
측정 시간: 2분 (120초)

사용법:
  python3 0_run_topic.py              # 전체 13개 토픽 실행 (0~12)
  python3 0_run_topic.py 0            # 토픽 0만 실행 (arming)
  python3 0_run_topic.py 1 2 3        # 토픽 1, 2, 3 실행
  python3 0_run_topic.py 0-5          # 토픽 0~5 실행
"""

import subprocess
import time
import sys
import os
import signal

# 설정
NODE_SCRIPT_DIR = "/home/rtcl-chmy/mavros_ws/src/mavros/chmy/nodes"
MAVROS_WS = "/home/rtcl-chmy/mavros_ws"
TEST_DURATION = 120  # 초 (2분)

# 13개 노드 스크립트 목록 (Topic 0 ~ 12)
NODE_SCRIPTS = [
    "command_arm.py",                  # Topic 0 (Service: /mavros/cmd/arming)
    "setpoint_raw_local.py",           # Topic 1
    "setpoint_velocity_cmd_vel.py",    # Topic 2
    "setpoint_position_local.py",      # Topic 3
    "setpoint_raw_attitude.py",        # Topic 4
    "setpoint_accel_accel.py",         # Topic 5
    "setpoint_raw_global.py",          # Topic 6
    "setpoint_trajectory_local.py",    # Topic 7
    "manual_control_send.py",          # Topic 8
    "setpoint_position_global.py",     # Topic 9
    "actuator_control.py",             # Topic 10
    "vision_pose_pose.py",             # Topic 11
    "mocap_pose.py",                   # Topic 12
]

MAX_TOPIC = 12  # Topic 0 ~ 12 (총 13개)

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
    """명령줄 인자에서 토픽 번호 파싱"""
    if not args:
        # 인자 없으면 전체 토픽 (0~12)
        return list(range(0, MAX_TOPIC + 1))
    
    selected = set()
    for arg in args:
        if '-' in arg:
            # 범위 지정 (예: 0-5)
            try:
                start, end = map(int, arg.split('-'))
                selected.update(range(start, end + 1))
            except:
                print(f"⚠️  잘못된 범위 형식: {arg}")
        else:
            # 개별 번호 (예: 0, 1, 2)
            try:
                selected.add(int(arg))
            except:
                print(f"⚠️  잘못된 토픽 번호: {arg}")
    
    # 0~12 범위로 제한
    selected = [t for t in sorted(selected) if 0 <= t <= MAX_TOPIC]
    return selected


def run_experiment(selected_topics):
    """실험 실행"""
    # 선택된 토픽에 해당하는 스크립트만 필터링 (Topic 0부터 시작)
    scripts_to_run = [(i, NODE_SCRIPTS[i]) for i in selected_topics]
    
    print("=" * 80)
    print("🚀 다중 토픽 Latency 측정 실험 시작")
    print("=" * 80)
    print(f"📊 실험 설정:")
    print(f"   - 선택된 토픽: {selected_topics}")
    print(f"   - 노드 개수: {len(scripts_to_run)}개")
    print(f"   - 발행 주기: 10Hz (각 노드)")
    print(f"   - 실험 시간: {TEST_DURATION}초")
    print(f"   - 스크립트 위치: {NODE_SCRIPT_DIR}")
    print("=" * 80)
    
    # 1. MAVROS 시작
    if not start_mavros():
        print("❌ MAVROS 시작 실패. 실험을 중단합니다.")
        return
    
    # 2. 선택된 노드 실행
    print(f"\n📝 {len(scripts_to_run)}개 노드 실행 중...")
    for topic_num, script in scripts_to_run:
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
            print(f"   ✅ Topic {topic_num:2d} 시작: {script}")
            time.sleep(0.2)  # 약간의 지연
        except Exception as e:
            print(f"   ❌ Topic {topic_num} 실행 실패: {e}")
    
    print(f"\n✅ 총 {len(processes)}개 노드 실행 완료\n")
    
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
        log_files = sorted([f for f in os.listdir(log_dir) if f.endswith('.log')])
        if log_files:
            print(f"✅ 로그 파일 생성됨:")
            for log_file in log_files[-15:]:  # 최근 15개만 표시
                log_path = os.path.join(log_dir, log_file)
                size = os.path.getsize(log_path)
                print(f"   - {log_file} ({size} bytes)")
            print(f"\n💡 분석 스크립트를 실행하세요:")
            print(f"   python3 /home/rtcl-chmy/mavros_ws/src/mavros/chmy/test/2_analyze_multi_topic.py")
        else:
            print("❌ 로그 파일이 생성되지 않았습니다.")
    else:
        print(f"❌ 로그 디렉토리가 없습니다: {log_dir}")


def main():
    # Ctrl+C 핸들러 등록
    signal.signal(signal.SIGINT, signal_handler)
    
    # 명령줄 인자 파싱
    selected_topics = parse_topic_selection(sys.argv[1:])
    
    if not selected_topics and selected_topics != [0]:
        print(f"❌ 유효한 토픽을 선택하세요 (0~{MAX_TOPIC})")
        print("\n사용법:")
        print(f"  python3 0_run_topic.py              # 전체 {MAX_TOPIC + 1}개 토픽 (0~12)")
        print("  python3 0_run_topic.py 0            # 토픽 0만 (arming)")
        print("  python3 0_run_topic.py 1 2 3        # 토픽 1, 2, 3")
        print("  python3 0_run_topic.py 0-5          # 토픽 0~5")
        print("\n📋 토픽 목록:")
        for i, script in enumerate(NODE_SCRIPTS, 0):
            print(f"  {i:2d}. {script}")
        sys.exit(1)
    
    try:
        run_experiment(selected_topics)
    except Exception as e:
        print(f"\n❌ 오류 발생: {e}")
        cleanup()
        sys.exit(1)


if __name__ == "__main__":
    main()
