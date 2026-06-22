#!/usr/bin/env python3
"""
이륙만 테스트 — ARM → /mavros/cmd/takeoff_local → 목표 고도 호버.

OFFBOARD·vz 수동 상승 없음. 7번 이륙 확인용.

사용 (chmy/test):
  source ~/mavros_ws/install/setup.bash
  python3 8_takeoff_only.py
"""

import argparse
import os
import signal
import subprocess
import sys
import time

import rclpy
from geometry_msgs.msg import PoseStamped, Vector3
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandTOLLocal
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

_TEST_DIR = os.path.dirname(os.path.abspath(__file__))
_CHMY_DIR = os.path.abspath(os.path.join(_TEST_DIR, '..'))
if _CHMY_DIR not in sys.path:
    sys.path.insert(0, _CHMY_DIR)
import affinity_env  # noqa: E402

DEFAULT_FCU_URL = 'udp://:14540@127.0.0.1:14557'

mavros_process = None
_started_mavros_here = False


def _find_mavros_ws() -> str:
    env = (os.environ.get('MAVROS_WS') or '').strip()
    if env and os.path.isfile(os.path.join(env, 'install', 'setup.bash')):
        return env
    d = os.path.abspath(_CHMY_DIR)
    for _ in range(8):
        setup = os.path.join(d, 'install', 'setup.bash')
        if os.path.isfile(setup):
            return d
        parent = os.path.dirname(d)
        if parent == d:
            break
        d = parent
    return '/home/rtcl-chmy/mavros_ws'


MAVROS_WS = _find_mavros_ws()
ROS_LOG_DIR = os.path.join(_CHMY_DIR, 'logs', 'ros_runtime')


def start_mavros(fcu_url: str, wait_sec: float) -> bool:
    global mavros_process, _started_mavros_here
    print('🚀 MAVROS 시작...')
    inner = (
        f'cd {MAVROS_WS} && source install/setup.bash && '
        f'ros2 run mavros mavros_node --ros-args -p fcu_url:={fcu_url}'
    )
    mavros_process = subprocess.Popen(
        affinity_env.bash_lc_mavros(inner),
        shell=False,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        preexec_fn=os.setsid,
    )
    _started_mavros_here = True
    time.sleep(wait_sec)
    if mavros_process.poll() is not None:
        print('❌ MAVROS 시작 실패')
        mavros_process = None
        _started_mavros_here = False
        return False
    print('✅ MAVROS 준비 완료')
    return True


def stop_mavros():
    global mavros_process, _started_mavros_here
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
    _started_mavros_here = False


def cleanup():
    stop_mavros()


def _signal_handler(sig, frame):
    del sig, frame
    print('\n🛑 중단 — MAVROS 정리...')
    cleanup()
    sys.exit(130)


class TakeoffOnlyNode(Node):
    def __init__(self, config):
        super().__init__('takeoff_only_node')

        self.target_alt = config.flight_altitude
        self.alt_tolerance = config.alt_tolerance
        self.takeoff_rate = config.takeoff_rate
        self.auto_arm = config.auto_arm

        self.fcu_connected = False
        self.is_armed = False
        self.current_pose = None
        self._arm_requested = False
        self._takeoff_requested = False
        self._reached_alt = False

        self.create_subscription(State, '/mavros/state', self._state_cb, 10)
        self.create_subscription(
            PoseStamped,
            '/mavros/local_position/pose',
            self._pose_cb,
            qos_profile_sensor_data,
        )

        self.arming_client = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.takeoff_client = self.create_client(
            CommandTOLLocal, '/mavros/cmd/takeoff_local')

        self.create_timer(0.05, self._status_loop)

        if self.auto_arm:
            self.create_timer(config.arm_delay_sec, self._auto_arm_once)

        self.get_logger().info('=== takeoff_local 이륙 테스트 ===')
        self.get_logger().info(
            f'z={self.target_alt:.1f}m rate={self.takeoff_rate:.1f}m/s')

    def _state_cb(self, msg: State):
        prev = (self.fcu_connected, self.is_armed)
        self.fcu_connected = msg.connected
        self.is_armed = msg.armed
        if (self.fcu_connected, self.is_armed) != prev:
            self.get_logger().info(
                f'FCU connected={self.fcu_connected} armed={self.is_armed} '
                f'mode={msg.mode}')

    def _pose_cb(self, msg: PoseStamped):
        self.current_pose = msg

    def _status_loop(self):
        if not self.fcu_connected:
            self.get_logger().warn('FCU 미연결', throttle_duration_sec=2.0)
            return
        if self.current_pose is None:
            self.get_logger().info('pose 대기', throttle_duration_sec=2.0)
            return
        alt = self.current_pose.pose.position.z
        if not self.is_armed:
            self.get_logger().info(
                f'ARM 대기 z={alt:.2f}m', throttle_duration_sec=2.0)
            return
        if not self._takeoff_requested:
            self.get_logger().info('takeoff 대기', throttle_duration_sec=2.0)
            return
        if self._reached_alt:
            self.get_logger().info(
                f'호버 z={alt:.2f}m', throttle_duration_sec=3.0)
            return
        if alt >= self.target_alt - self.alt_tolerance:
            self._reached_alt = True
            self.get_logger().warn(
                f'✅ 목표 고도 z={alt:.2f}m (Ctrl+C 종료)')
            return
        self.get_logger().info(
            f'takeoff z={alt:.2f}/{self.target_alt:.1f}m',
            throttle_duration_sec=1.0)

    def _auto_arm_once(self):
        if self._arm_requested:
            return
        self._arm_requested = True
        if not self.auto_arm:
            return
        if not self.arming_client.wait_for_service(timeout_sec=15.0):
            self.get_logger().error('/mavros/cmd/arming 없음')
            return
        req = CommandBool.Request()
        req.value = True
        self.arming_client.call_async(req).add_done_callback(self._on_arm_done)

    def _on_arm_done(self, future):
        try:
            resp = future.result()
            if not resp.success:
                self.get_logger().error(f'ARM 실패: {resp}')
                return
        except Exception as e:
            self.get_logger().error(f'ARM 예외: {e}')
            return
        self._request_takeoff_local()

    def _request_takeoff_local(self):
        if self._takeoff_requested:
            return
        if not self.takeoff_client.wait_for_service(timeout_sec=10.0):
            self.get_logger().error('/mavros/cmd/takeoff_local 없음')
            return
        self._takeoff_requested = True
        req = CommandTOLLocal.Request()
        req.min_pitch = 0.0
        req.offset = 0.0
        req.rate = float(self.takeoff_rate)
        req.yaw = 0.0
        req.position = Vector3(x=0.0, y=0.0, z=float(self.target_alt))
        self.get_logger().info(
            f'takeoff_local z={self.target_alt:.1f}m')
        self.takeoff_client.call_async(req).add_done_callback(self._on_takeoff_done)

    def _on_takeoff_done(self, future):
        try:
            resp = future.result()
            if resp.success:
                self.get_logger().info('takeoff_local OK')
            else:
                self.get_logger().error(f'takeoff_local 실패 result={resp.result}')
        except Exception as e:
            self.get_logger().error(f'takeoff_local 예외: {e}')


def main(args=None):
    parser = argparse.ArgumentParser(description='takeoff_local 이륙만')
    parser.add_argument('--flight-altitude', type=float, default=5.0)
    parser.add_argument('--alt-tolerance', type=float, default=0.5)
    parser.add_argument('--takeoff-rate', type=float, default=1.0)
    parser.add_argument('--arm-delay-sec', type=float, default=3.0)
    parser.add_argument('--no-auto-arm', action='store_true')
    parser.add_argument(
        '--no-start-mavros',
        action='store_true',
        help='MAVROS를 띄우지 않음',
    )
    parser.add_argument('--fcu-url', default=DEFAULT_FCU_URL)
    parser.add_argument('--mavros-wait-sec', type=float, default=5.0)
    config = parser.parse_args(args=args)
    config.auto_arm = not config.no_auto_arm

    signal.signal(signal.SIGINT, _signal_handler)
    os.makedirs(ROS_LOG_DIR, exist_ok=True)
    os.environ.setdefault('ROS_LOG_DIR', ROS_LOG_DIR)

    setup_bash = os.path.join(MAVROS_WS, 'install', 'setup.bash')
    if not os.path.isfile(setup_bash):
        print(f'❌ {setup_bash} 없음')
        return 1

    if not config.no_start_mavros:
        if not start_mavros(config.fcu_url, config.mavros_wait_sec):
            return 1
    else:
        print('ℹ️  --no-start-mavros: 기존 MAVROS 사용')

    rclpy.init(args=None)
    node = TakeoffOnlyNode(config)
    print('\n' + '=' * 50)
    print('  8_takeoff_only — takeoff_local')
    print('=' * 50)
    print(f'  고도 {config.flight_altitude} m, PX4 시뮬 후 실행')
    print('=' * 50 + '\n')
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print('\n종료')
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        if _started_mavros_here:
            cleanup()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
