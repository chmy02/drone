#!/usr/bin/env python3
"""
장애물 정지 E2E 측정 러너.

사용법:
  python3 5_end_to_end_measurement.py 1
  python3 5_end_to_end_measurement.py 10 1 2 4

인자:
  1번째: 시행 횟수(iterations)
  2번째 이후: 함께 실행할 토픽 번호(0~12). 생략하면 main flow만 실행.

동작:
  - MAVROS/부하 토픽 노드는 1회만 시작해 유지
  - main flow 노드가 내부에서 N회 반복
    (정지 → 후퇴 복귀 → 5초 대기 → 재전진)
  - 완료 후 한 번에 모두 종료

코어 고정(선택): CHMY_CPU_* 환경변수 또는
  chmy/scripts/cpu_affinity.env + run_e2e_affinity.sh
"""

import os
import signal
import subprocess
import sys
import time

_CHMY_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
if _CHMY_DIR not in sys.path:
    sys.path.insert(0, _CHMY_DIR)
import affinity_env  # noqa: E402

MAVROS_WS = "/home/rtcl-chmy/mavros_ws"
NODE_SCRIPT_DIR = "/home/rtcl-chmy/mavros_ws/src/mavros/chmy/nodes"
OBSTACLE_SCRIPT = "/home/rtcl-chmy/mavros_ws/src/mavros/chmy/auto_drive_test/obstacle_stop_test.py"
NODE_START_INTERVAL = 0.5
ROS_LOG_DIR = "/home/rtcl-chmy/mavros_ws/src/mavros/chmy/logs/ros_runtime"

NODE_SCRIPTS = {
    0: "command_arm.py",
    1: "setpoint_raw_local.py",
    2: "setpoint_velocity_cmd_vel.py",
    3: "setpoint_position_local.py",
    4: "setpoint_raw_attitude.py",
    5: "setpoint_accel_accel.py",
    6: "setpoint_raw_global.py",
    7: "setpoint_trajectory_local.py",
    8: "manual_control_send.py",
    9: "setpoint_position_global.py",
    10: "actuator_control.py",
    11: "vision_pose_pose.py",
    12: "mocap_pose.py",
}

TOPIC_NAMES = {
    0: "cmd/arming (service)",
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
    11: "vision_pose/pose",
    12: "mocap/pose",
}

running_topic_procs = []
mavros_process = None
main_flow_process = None
stop_requested = False


def ros_env():
    env = os.environ.copy()
    env["ROS_LOG_DIR"] = ROS_LOG_DIR
    os.makedirs(ROS_LOG_DIR, exist_ok=True)
    return env


def parse_topics(args):
    selected = []
    for raw in args:
        try:
            n = int(raw)
        except ValueError:
            print(f"⚠️ 잘못된 토픽 번호 무시: {raw}")
            continue
        if n < 0 or n > 12:
            print(f"⚠️ 범위를 벗어난 토픽 번호 무시: {n}")
            continue
        if n not in selected:
            selected.append(n)
    return selected


def signal_handler(sig, frame):
    del sig, frame
    global stop_requested
    stop_requested = True
    print("\n🛑 중단 요청 수신. 정리 후 종료합니다...")
    cleanup_once()
    sys.exit(130)


def start_mavros():
    global mavros_process
    print("🚀 MAVROS 시작...")
    inner = (
        f"cd {MAVROS_WS} && source install/setup.bash && "
        "ros2 run mavros mavros_node --ros-args "
        "-p fcu_url:=udp://:14540@127.0.0.1:14557"
    )
    mavros_process = subprocess.Popen(
        affinity_env.bash_lc_mavros(inner),
        shell=False,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        preexec_fn=os.setsid,
    )
    time.sleep(5.0)
    if mavros_process.poll() is not None:
        print("❌ MAVROS 시작 실패")
        return False
    print("✅ MAVROS 준비 완료")
    return True


def stop_mavros():
    global mavros_process
    if mavros_process is None:
        return
    try:
        os.killpg(os.getpgid(mavros_process.pid), signal.SIGTERM)
        mavros_process.wait(timeout=5)
    except Exception:
        try:
            os.killpg(os.getpgid(mavros_process.pid), signal.SIGKILL)
        except Exception:
            pass
    mavros_process = None


def start_topic_nodes(selected_topics):
    print(f"📡 부하 토픽 노드 시작 ({len(selected_topics)}개)")
    for topic_id in selected_topics:
        script = NODE_SCRIPTS[topic_id]
        path = os.path.join(NODE_SCRIPT_DIR, script)
        if not os.path.exists(path):
            print(f"   ❌ 파일 없음: {path}")
            continue
        proc = subprocess.Popen(
            affinity_env.popen_argv_topic_python(path),
            env=ros_env(),
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
        )
        running_topic_procs.append(proc)
        print(f"   ✅ Topic {topic_id}: {TOPIC_NAMES[topic_id]}")
        time.sleep(NODE_START_INTERVAL)


def stop_topic_nodes():
    global running_topic_procs
    for proc in running_topic_procs:
        try:
            proc.terminate()
            proc.wait(timeout=2)
        except Exception:
            try:
                proc.kill()
            except Exception:
                pass
    running_topic_procs = []


def run_main_flow(iterations):
    global main_flow_process
    cmd = [
        "python3",
        OBSTACLE_SCRIPT,
        "--max-e2e-iterations",
        str(iterations),
        "--retreat-distance",
        "5.0",
        "--post-wait-sec",
        "5.0",
        "--auto-exit-on-complete",
        "--log-all-lidar",
    ]
    print(f"🧭 Main flow 시작 (장애물 감지→정지 반복 {iterations}회)")
    main_flow_process = subprocess.Popen(
        affinity_env.popen_argv_main_python(cmd),
        env=ros_env(),
    )
    rc = main_flow_process.wait()
    main_flow_process = None
    return rc


def stop_main_flow():
    global main_flow_process
    if main_flow_process is None:
        return
    try:
        main_flow_process.terminate()
        main_flow_process.wait(timeout=5)
    except Exception:
        try:
            main_flow_process.kill()
        except Exception:
            pass
    main_flow_process = None


def cleanup_once():
    stop_main_flow()
    stop_topic_nodes()
    stop_mavros()


def print_usage():
    print("사용법:")
    print("  python3 5_end_to_end_measurement.py <시행횟수> [토픽번호 ...]")
    print("예시:")
    print("  python3 5_end_to_end_measurement.py 1")
    print("  python3 5_end_to_end_measurement.py 10 1 2 4")
    print("비고:")
    print("  - 토픽번호를 생략하면 main flow만 실행")
    print("  - 시행 간 5초 대기는 main flow 내부에서 처리")


def main():
    signal.signal(signal.SIGINT, signal_handler)

    if len(sys.argv) < 2:
        print_usage()
        return 1

    try:
        iterations = int(sys.argv[1])
    except ValueError:
        print("❌ 시행 횟수는 정수여야 합니다.")
        print_usage()
        return 1

    if iterations <= 0:
        print("❌ 시행 횟수는 1 이상이어야 합니다.")
        return 1

    selected_topics = parse_topics(sys.argv[2:])
    print("📋 실행 설정")
    print(f"   - 시행 횟수: {iterations}")
    print(f"   - 부하 토픽: {selected_topics if selected_topics else '없음(main flow만)'}")
    print("   - MAVROS/부하 토픽은 1회 시작 후 유지")
    for line in affinity_env.summary_lines():
        print(f"   - {line}")

    try:
        if not start_mavros():
            return 1
        start_topic_nodes(selected_topics)
        time.sleep(2.0)

        rc = run_main_flow(iterations)
        if rc != 0:
            print(f"❌ Main flow 비정상 종료 (code={rc})")
            return 1

        print("\n" + "=" * 80)
        print(f"✅ 완료: 반복 {iterations}회 수행")
        print("=" * 80)
        return 0
    finally:
        cleanup_once()
        print("🧹 원복 완료 (노드/토픽/MAVROS 종료)")


if __name__ == "__main__":
    raise SystemExit(main())
