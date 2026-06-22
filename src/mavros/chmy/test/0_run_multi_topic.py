#!/usr/bin/env python3
"""
원하는 토픽들을 동시에 실행하는 스크립트
커맨드 라인 인자로 토픽 번호를 지정할 수 있습니다.

사용법:
  python3 0_run_multi_topic.py 0            # 서비스 콜 (cmd/arming) 단독 실행
  python3 0_run_multi_topic.py 0 1 2        # 서비스 + Topic 1, 2 동시 실행
  python3 0_run_multi_topic.py 1 3 5        # Topic 1, 3, 5를 동시 실행 (입력 순서대로)
  python3 0_run_multi_topic.py 0-5          # 서비스 + Topic 1~5를 동시 실행
  python3 0_run_multi_topic.py 11 12 1-5    # Topic 11, 12 먼저, 그 다음 1~5 (순서 지정!)
  python3 0_run_multi_topic.py              # 전체 13개 (서비스 0 + 토픽 1~12) 동시 실행

옵션:
  -i, --interval <초>   노드 시작 간격 (기본: 0.5초)
  -w, --warmup <초>     모든 노드 시작 후 안정화 시간 (기본: 3초)
  -d, --duration <초>   실험 시간 (기본: 120초 = 2분)

예시:
  python3 0_run_multi_topic.py 11 12 1-10 -i 1.0 -w 5   # 11,12 먼저, 간격 1초, 안정화 5초
  python3 0_run_multi_topic.py 1-12 -i 0.3              # 전체 토픽, 간격 0.3초

코어 고정(선택): chmy/scripts/cpu_affinity.env.example 을 cpu_affinity.env 로 복사·편집 후
  source .../chmy/scripts/cpu_affinity.env && python3 0_run_multi_topic.py ...
  또는 ./chmy/scripts/run_multi_topic_affinity.sh ...

토픽 목록:
  [서비스 콜] 0 (command/arming)
  [기본 플러그인] 1~10
  [Extras 플러그인] 11~12
"""

import subprocess
import time
import sys
import os
import signal
import argparse

_CHMY_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
if _CHMY_DIR not in sys.path:
    sys.path.insert(0, _CHMY_DIR)
import affinity_env  # noqa: E402

# 설정
NODE_SCRIPT_DIR = "/home/rtcl-chmy/mavros_ws/src/mavros/chmy/nodes"
MAVROS_WS = "/home/rtcl-chmy/mavros_ws"

# 기본값
DEFAULT_INTERVAL = 2.0    # 노드 시작 간격 (초)
DEFAULT_WARMUP = 5.0      # 안정화 시간 (초)
DEFAULT_DURATION = 60     # 실험 시간 (초) - 1분

# 13개 노드 스크립트 목록 (0: 서비스, 1~12: 토픽)
NODE_SCRIPTS = {
    # 서비스 콜 - Topic 0
    0: "command_arm.py",                  # Topic 0 (Service)
    # 기본 플러그인 (mavros) - Topic 1~10
    1: "setpoint_raw_local.py",           # Topic 1
    2: "setpoint_velocity_cmd_vel.py",    # Topic 2
    3: "setpoint_position_local.py",      # Topic 3
    4: "setpoint_raw_attitude.py",        # Topic 4
    5: "setpoint_accel_accel.py",         # Topic 5
    6: "setpoint_raw_global.py",          # Topic 6 (NEW)
    7: "setpoint_trajectory_local.py",    # Topic 7 (NEW)
    8: "manual_control_send.py",          # Topic 8 (NEW)
    9: "setpoint_position_global.py",     # Topic 9 (NEW)
    10: "actuator_control.py",            # Topic 10 (NEW)
    # Extras 플러그인 (mavros_extras) - Topic 11~12
    11: "vision_pose_pose.py",            # Topic 11 (was 6)
    12: "mocap_pose.py",                  # Topic 12 (was 7)
}

TOPIC_NAMES = {
    # 서비스 콜 - Topic 0
    0: "cmd/arming (service)",
    # 기본 플러그인 (mavros) - Topic 1~10
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
    # Extras 플러그인 (mavros_extras) - Topic 11~12
    11: "vision_pose/pose",
    12: "mocap/pose",
}

MIN_TOPIC = 0   # 최소 토픽 번호 (서비스 콜)
MAX_TOPIC = 12  # 최대 토픽 번호

# 실행된 프로세스들을 저장할 리스트
processes = []
mavros_process = None
system_monitor_proc = None


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
    mavros_cmd = (
        f"cd {MAVROS_WS} && source install/setup.bash && "
        "ros2 run mavros mavros_node --ros-args -p fcu_url:=udp://:14540@127.0.0.1:14557"
    )
    
    try:
        mavros_process = subprocess.Popen(
            affinity_env.bash_lc_mavros(mavros_cmd),
            shell=False,
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
    
    # 1. 실행된 노드 프로세스 종료
    for proc in processes:
        try:
            proc.terminate()
            proc.wait(timeout=2)
        except:
            try:
                proc.kill()
            except:
                pass
    
    # 2. 기존에 실행 중인 command_arm.py 프로세스 종료 (혹시 모를 경우 대비)
    try:
        result = subprocess.run(
            ['pkill', '-f', 'command_arm.py'],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            timeout=2
        )
        if result.returncode == 0:
            print("   ✅ 기존 command_arm.py 프로세스 종료")
    except:
        pass
    
    print("✅ 모든 노드 종료 완료")

    # 2.5. 시스템 모니터 대기 (실행 중이었다면)
    global system_monitor_proc
    if system_monitor_proc is not None:
        try:
            system_monitor_proc.wait(timeout=5)
            print("   ✅ 시스템 모니터 종료 완료")
        except subprocess.TimeoutExpired:
            system_monitor_proc.kill()
            print("   ⚠️  시스템 모니터 타임아웃 후 종료")
        system_monitor_proc = None

    # 3. MAVROS 종료
    stop_mavros()


def parse_topic_selection(args):
    """명령줄 인자에서 토픽 번호 파싱 (입력 순서 유지)"""
    if not args:
        # 인자 없으면 전체 (0번 서비스 + 1~12 토픽)
        return list(range(0, MAX_TOPIC + 1))
    
    selected = []
    seen = set()  # 중복 제거용
    
    for arg in args:
        if '-' in arg and not arg.startswith('-'):
            # 범위 지정 (예: 1-5, 0-5)
            try:
                start, end = map(int, arg.split('-'))
                for t in range(start, end + 1):
                    if MIN_TOPIC <= t <= MAX_TOPIC and t not in seen:
                        selected.append(t)
                        seen.add(t)
            except:
                print(f"⚠️  잘못된 범위 형식: {arg}")
        else:
            # 개별 번호 (예: 0, 1, 2, 3)
            try:
                t = int(arg)
                if MIN_TOPIC <= t <= MAX_TOPIC and t not in seen:
                    selected.append(t)
                    seen.add(t)
            except:
                print(f"⚠️  잘못된 토픽 번호: {arg}")
    
    return selected


def run_experiment(selected_topics, interval, warmup, duration, use_system_monitor=False):
    """실험 실행"""
    print("=" * 80)
    print("🚀 다중 토픽 동시 실행 Latency 측정")
    print("=" * 80)
    print(f"📊 실험 설정:")
    print(f"   - 선택된 토픽: {selected_topics}")
    print(f"   - 노드 개수: {len(selected_topics)}개")
    print(f"   - 실행 순서: 입력 순서 유지 ⭐")
    print(f"   - 노드 시작 간격: {interval}초")
    print(f"   - 안정화 시간: {warmup}초")
    print(f"   - 발행 주기: 10Hz (각 노드)")
    print(f"   - 실험 시간: {duration}초")
    if use_system_monitor:
        print(f"   - 시스템 모니터: 0.1초 간격 CPU 기록 활성화")
    for line in affinity_env.summary_lines():
        print(f"   - {line}")
    print(f"\n📋 실행 순서:")
    for idx, topic_num in enumerate(selected_topics, 1):
        if topic_num == 0:
            plugin_type = "🔴 서비스"
        elif topic_num >= 11:
            plugin_type = "🔵 extras"
        else:
            plugin_type = "🟢 기본"
        print(f"   {idx:2d}번째 → Topic {topic_num:2d}: {TOPIC_NAMES[topic_num]} ({plugin_type})")
    print("=" * 80)
    
    # 0. 기존 command_arm.py 프로세스 종료 (혹시 실행 중인 경우)
    if 0 in selected_topics:
        print("\n🧹 기존 command_arm.py 프로세스 확인 중...")
        try:
            result = subprocess.run(
                ['pgrep', '-f', 'command_arm.py'],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE
            )
            if result.returncode == 0:
                print("   ⚠️  기존 command_arm.py 프로세스 발견, 종료 중...")
                subprocess.run(['pkill', '-f', 'command_arm.py'], timeout=2)
                time.sleep(0.5)
                print("   ✅ 기존 프로세스 종료 완료")
        except:
            pass
    
    # 1. MAVROS 시작
    if not start_mavros():
        print("❌ MAVROS 시작 실패. 실험을 중단합니다.")
        return
    
    # 2. 선택된 노드 순서대로 실행 (토픽 측정 시작 = 첫 노드 기동 시점)
    extra_warmup = 1.0 if 0 in selected_topics else 0
    total_warmup = warmup + extra_warmup
    node_launch_duration = max(0, (len(selected_topics) - 1)) * interval

    if use_system_monitor:
        global system_monitor_proc
        monitor_script = os.path.join(os.path.dirname(os.path.abspath(__file__)), "system_monitor.py")
        monitor_duration = node_launch_duration + total_warmup + duration + 2
        system_monitor_proc = subprocess.Popen(
            affinity_env.taskset_prefix(affinity_env.cpus_monitor())
            + ["python3", monitor_script, "-d", str(monitor_duration), "-i", "0.1"],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.PIPE,
        )
        print(f"📊 시스템 모니터 시작 (토픽측정 시작~종료, 총 {monitor_duration:.0f}초)\n")

    print(f"\n📝 {len(selected_topics)}개 노드 순차 시작 중 (간격: {interval}초)...")
    for idx, topic_num in enumerate(selected_topics, 1):
        script = NODE_SCRIPTS[topic_num]
        script_path = os.path.join(NODE_SCRIPT_DIR, script)
        
        if not os.path.exists(script_path):
            print(f"❌ 파일을 찾을 수 없습니다: {script_path}")
            continue
        
        try:
            proc = subprocess.Popen(
                affinity_env.popen_argv_topic_python(script_path),
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE
            )
            processes.append(proc)
            print(f"   ✅ [{idx:2d}/{len(selected_topics)}] Topic {topic_num:2d} 시작: {TOPIC_NAMES[topic_num]}")
            
            # 마지막 노드가 아니면 간격만큼 대기
            if idx < len(selected_topics):
                time.sleep(interval)
        except Exception as e:
            print(f"   ❌ Topic {topic_num} 실행 실패: {e}")
    
    print(f"\n✅ 총 {len(processes)}개 노드 실행 완료")

    # 3. 안정화 대기
    if warmup > 0:
        if 0 in selected_topics:
            print(f"\n⏳ 시스템 안정화 대기 중 ({warmup}초 + {extra_warmup}초 추가)...")
        else:
            print(f"\n⏳ 시스템 안정화 대기 중 ({warmup}초)...")
        time.sleep(total_warmup)
        print("✅ 안정화 완료!\n")

    # 4. 실험 진행
    print(f"⏱️  {duration}초 동안 latency 측정 중...\n")
    for remaining in range(int(duration), 0, -10):
        print(f"   ⏳ 남은 시간: {remaining}초...")
        time.sleep(10)
    
    print("\n✅ 실험 완료!\n")
    
    # 5. 노드 및 MAVROS 종료
    cleanup()
    
    # 5. 로그 파일 확인
    print("\n📁 로그 파일 확인 중...")
    log_dir = "/home/rtcl-chmy/mavros_ws/src/mavros/chmy/logs"
    
    if os.path.exists(log_dir):
        import glob
        log_files = sorted(glob.glob(os.path.join(log_dir, "*_topic*_latency.log")),
                          key=os.path.getmtime, reverse=True)
        sys_logs = sorted(glob.glob(os.path.join(log_dir, "*_system_monitor.log")),
                          key=os.path.getmtime, reverse=True)
        if log_files:
            print(f"✅ 로그 파일 생성됨 (최근 {min(len(log_files), 15)}개):")
            for log_file in log_files[:15]:
                size = os.path.getsize(log_file)
                basename = os.path.basename(log_file)
                print(f"   - {basename} ({size:,} bytes)")
            if sys_logs:
                for s in sys_logs[:3]:
                    print(f"   - {os.path.basename(s)} (시스템 모니터)")
            print(f"\n💡 다음 단계:")
            print(f"   1. 분석: python3 /home/rtcl-chmy/mavros_ws/src/mavros/chmy/test/2_analyze_multi_topic.py")
            print(f"   2. 엑셀: python3 /home/rtcl-chmy/mavros_ws/src/mavros/chmy/test/3_export_to_excel_sheets.py")
        else:
            print("❌ 로그 파일이 생성되지 않았습니다.")
    else:
        print(f"❌ 로그 디렉토리가 없습니다: {log_dir}")


def print_help():
    """도움말 출력"""
    print(f"\n📋 사용 가능한 토픽 ({MIN_TOPIC}~{MAX_TOPIC}):")
    print("\n  🔴 서비스 콜:")
    print(f"     {0:2d}. {TOPIC_NAMES[0]}")
    print("\n  🟢 기본 플러그인 (mavros):")
    for topic_num in range(1, 11):
        print(f"    {topic_num:2d}. {TOPIC_NAMES[topic_num]}")
    print("\n  🔵 Extras 플러그인 (mavros_extras):")
    for topic_num in range(11, MAX_TOPIC + 1):
        print(f"    {topic_num:2d}. {TOPIC_NAMES[topic_num]}")
    
    print("\n📖 사용법:")
    print("  python3 0_run_multi_topic.py [토픽...] [옵션]")
    print("\n토픽 지정 (순서대로 실행됨!):")
    print("  0             → 서비스 콜 (cmd/arming) 단독 실행")
    print("  0 1 2         → 서비스 + Topic 1, 2 순서대로")
    print("  1 3 5         → Topic 1, 3, 5 순서대로")
    print("  0-5           → 서비스 + Topic 1~5 순서대로")
    print("  11 12 1-5     → Topic 11, 12 먼저, 그 다음 1~5 ⭐")
    print("  (생략)        → 전체 1~12 순서대로 (0번 제외)")
    print("\n옵션:")
    print(f"  -i, --interval <초>   노드 시작 간격 (기본: {DEFAULT_INTERVAL}초)")
    print(f"  -w, --warmup <초>     안정화 시간 (기본: {DEFAULT_WARMUP}초)")
    print(f"  -d, --duration <초>   실험 시간 (기본: {DEFAULT_DURATION}초 = 1분)")
    print(f"  -m, --system-monitor  0.1초 간격 CPU 기록 (레이턴시 분석용)")
    print("\n예시:")
    print("  python3 0_run_multi_topic.py 0 1 2 -d 30")
    print("  → 서비스(0) + Topic 1, 2 실행, 30초 측정")
    print("  python3 0_run_multi_topic.py 11 12 1-10 -i 1.0 -w 5")
    print("  → Topic 11, 12를 먼저 시작 (간격 1초), 그 다음 1~10, 안정화 5초")


def main():
    # Ctrl+C 핸들러 등록
    signal.signal(signal.SIGINT, signal_handler)
    
    # 옵션과 토픽 번호 분리
    topic_args = []
    interval = DEFAULT_INTERVAL
    warmup = DEFAULT_WARMUP
    duration = DEFAULT_DURATION
    use_system_monitor = False

    i = 1
    while i < len(sys.argv):
        arg = sys.argv[i]
        
        if arg in ['-i', '--interval']:
            if i + 1 < len(sys.argv):
                try:
                    interval = float(sys.argv[i + 1])
                    i += 2
                    continue
                except ValueError:
                    print(f"❌ 잘못된 interval 값: {sys.argv[i + 1]}")
                    sys.exit(1)
        elif arg in ['-w', '--warmup']:
            if i + 1 < len(sys.argv):
                try:
                    warmup = float(sys.argv[i + 1])
                    i += 2
                    continue
                except ValueError:
                    print(f"❌ 잘못된 warmup 값: {sys.argv[i + 1]}")
                    sys.exit(1)
        elif arg in ['-d', '--duration']:
            if i + 1 < len(sys.argv):
                try:
                    duration = float(sys.argv[i + 1])
                    i += 2
                    continue
                except ValueError:
                    print(f"❌ 잘못된 duration 값: {sys.argv[i + 1]}")
                    sys.exit(1)
        elif arg in ['-m', '--system-monitor']:
            use_system_monitor = True
            i += 1
            continue
        elif arg in ['-h', '--help']:
            print_help()
            sys.exit(0)
        else:
            topic_args.append(arg)
        
        i += 1
    
    # 토픽 파싱
    selected_topics = parse_topic_selection(topic_args)
    
    if not selected_topics:
        print(f"❌ 유효한 토픽을 선택하세요 ({MIN_TOPIC}~{MAX_TOPIC})")
        print_help()
        sys.exit(1)
    
    try:
        run_experiment(selected_topics, interval, warmup, duration, use_system_monitor)
    except Exception as e:
        print(f"\n❌ 오류 발생: {e}")
        cleanup()
        sys.exit(1)


if __name__ == "__main__":
    main()
