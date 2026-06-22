#!/usr/bin/env python3
"""
목표 (x, y)까지 자율 주행 테스트 (고도 고정, 라이다 회피).

상태: TAKEOFF(/mavros/cmd/takeoff_local) → OFFBOARD → FORWARD(직진)
      → (벽) TURN_CLEAR → … → 목표 xy 도달 시 DONE

전제: PX4(시뮬) + /lidar. MAVROS는 스크립트가 자동 기동 (--no-start-mavros 로 생략).

사용 예 (chmy/test):
  source ~/mavros_ws/install/setup.bash
  python3 7_goto_xy_nav.py

  ./7_goto_xy_nav.py                      # 실행 권한 있을 때
"""

import argparse
import math
import os
import signal
import subprocess
import sys
import time

import rclpy
from geometry_msgs.msg import PoseStamped, TwistStamped, Vector3
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandTOLLocal, SetMode
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

_TEST_DIR = os.path.dirname(os.path.abspath(__file__))
_CHMY_DIR = os.path.abspath(os.path.join(_TEST_DIR, '..'))
if _CHMY_DIR not in sys.path:
    sys.path.insert(0, _CHMY_DIR)
import affinity_env  # noqa: E402

DEFAULT_FCU_URL = 'udp://:14540@127.0.0.1:14557'


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

mavros_process = None
_started_mavros_here = False


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
    print('\n🛑 중단 요청 — MAVROS 정리 중...')
    cleanup()
    sys.exit(130)


class NavPhase:
    TAKEOFF = 'TAKEOFF'          # MAVROS takeoff_local
    FORWARD = 'FORWARD'          # 전방 비어 있으면 직진
    TURN_CLEAR = 'TURN_CLEAR'    # 벽 만남 → 여유 큰 쪽으로 회전
    DONE = 'DONE'


class GotoXyNavNode(Node):
    def __init__(self, config):
        super().__init__('goto_xy_nav_node')

        self.goal_x = config.goal_x
        self.goal_y = config.goal_y
        self.flight_altitude = config.flight_altitude
        self.alt_tolerance = config.alt_tolerance
        self.takeoff_rate = config.takeoff_rate
        self.cruise_velocity = config.cruise_velocity
        self.obstacle_distance = config.obstacle_distance
        self.obstacle_clear_distance = (
            config.obstacle_distance + config.obstacle_clear_margin)
        self.lidar_range_min = config.lidar_range_min
        self.lidar_range_max = config.lidar_range_max
        self.use_scan_range_limits = config.use_scan_range_limits
        # 전방 시야 총각(°). 내부에서는 ±(fov/2) 만 사용 (기본 90° → ±45°)
        self.front_fov_deg = config.front_fov_deg
        self.front_half_deg = self.front_fov_deg * 0.5
        self.goal_tolerance = config.goal_tolerance
        self.turn_rate = config.turn_rate
        self.auto_arm_offboard = config.auto_arm_offboard
        self.fcu_url = getattr(config, 'fcu_url', DEFAULT_FCU_URL)
        self._auto_sequence_started = False
        self._lidar_debug_count = 0

        self.phase = NavPhase.TAKEOFF
        self.fcu_connected = False
        self.is_armed = False
        self.is_offboard = False
        self._takeoff_requested = False
        self._offboard_requested = False
        self.current_pose = None
        self.min_front_dist = float('inf')
        self.min_left_dist = float('inf')
        self.min_right_dist = float('inf')
        self.obstacle_ahead = False
        self._obstacle_latched = False
        self._clear_turn_sign = 1.0  # +1: 좌회전, -1: 우회전

        lidar_qos = QoSProfile(depth=10)
        lidar_qos.reliability = ReliabilityPolicy.BEST_EFFORT
        # MAVROS local_position 은 SensorDataQoS(BEST_EFFORT) — RELIABLE 이면 수신 안 됨

        self.create_subscription(State, '/mavros/state', self._state_cb, 10)
        self.create_subscription(
            PoseStamped,
            '/mavros/local_position/pose',
            self._pose_cb,
            qos_profile_sensor_data,
        )
        self.create_subscription(LaserScan, '/lidar', self._lidar_cb, lidar_qos)
        self.velocity_pub = self.create_publisher(
            TwistStamped, '/mavros/setpoint_velocity/cmd_vel', 10)

        self.arming_client = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.takeoff_client = self.create_client(
            CommandTOLLocal, '/mavros/cmd/takeoff_local')
        self.set_mode_client = self.create_client(SetMode, '/mavros/set_mode')

        self.create_timer(0.02, self._control_loop)
        self.create_timer(0.1, self._publish_idle_setpoint)

        if self.auto_arm_offboard:
            self.create_timer(1.0, self._try_begin_takeoff_sequence)
        self._auto_arm_timer = None

        self.get_logger().info('=== goto_xy 자율주행 테스트 ===')
        self.get_logger().info(
            f'이륙: /mavros/cmd/takeoff_local z={self.flight_altitude:.1f}m '
            f'rate={self.takeoff_rate:.1f}m/s')
        self.get_logger().info(
            f'목표=({self.goal_x:.2f}, {self.goal_y:.2f}) m, '
            f'고도={self.flight_altitude:.1f} m, '
            f'속도={self.cruise_velocity:.2f} m/s')
        self.get_logger().info(
            f'장애물: {self.obstacle_distance:.1f} m 이하 정지, '
            f'{self.obstacle_clear_distance:.1f} m 초과 시 해제, '
            f'라이다 유효 [{self.lidar_range_min:.2f}, {self.lidar_range_max:.2f}] m')
        self.get_logger().info(
            f'전방 시야 {self.front_fov_deg:.0f}° (±{self.front_half_deg:.0f}°), '
            f'벽 만나면 좌/우 거리 큰 쪽으로 돌아 직진 반복')

    def _state_cb(self, msg: State):
        prev = (self.fcu_connected, self.is_armed, self.is_offboard)
        self.fcu_connected = msg.connected
        self.is_armed = msg.armed
        self.is_offboard = msg.mode == 'OFFBOARD'
        cur = (self.fcu_connected, self.is_armed, self.is_offboard)
        if cur != prev:
            self.get_logger().info(
                f'FCU connected={self.fcu_connected} armed={self.is_armed} '
                f'mode={msg.mode}')

    def _pose_cb(self, msg: PoseStamped):
        self.current_pose = msg

    def _horizontal_dist_to_goal(self) -> float:
        px = self.current_pose.pose.position.x
        py = self.current_pose.pose.position.y
        return math.hypot(self.goal_x - px, self.goal_y - py)

    def _effective_lidar_limits(self, msg: LaserScan):
        """CLI 한도와 LaserScan.range_min/max 교집합."""
        rmin = self.lidar_range_min
        rmax = self.lidar_range_max
        if self.use_scan_range_limits and msg.range_max > 0.0:
            rmax = min(rmax, float(msg.range_max))
        if self.use_scan_range_limits and msg.range_min >= 0.0:
            rmin = max(rmin, float(msg.range_min))
        if rmax <= rmin:
            rmax = rmin + 0.01
        return rmin, rmax

    def _lidar_range_valid(self, dist: float, rmin: float, rmax: float) -> bool:
        if math.isinf(dist) or math.isnan(dist) or dist <= 0.0:
            return False
        if dist <= rmin or dist >= rmax:
            return False
        # 미반사·out-of-range 가 range_max 근처로 오는 경우 제외
        if rmax > 0.0 and dist >= rmax * 0.995:
            return False
        return True

    def _lidar_cb(self, msg: LaserScan):
        if not msg.ranges:
            return

        rmin, rmax = self._effective_lidar_limits(msg)
        half_rad = math.radians(self.front_half_deg)

        min_front = float('inf')
        min_left = float('inf')
        min_right = float('inf')

        for i, dist in enumerate(msg.ranges):
            if not self._lidar_range_valid(dist, rmin, rmax):
                continue
            angle = msg.angle_min + i * msg.angle_increment
            if angle < -half_rad or angle > half_rad:
                continue
            min_front = min(min_front, dist)
            if angle > 0.0:
                min_left = min(min_left, dist)
            elif angle < 0.0:
                min_right = min(min_right, dist)

        self.min_front_dist = min_front
        self.min_left_dist = min_left
        self.min_right_dist = min_right

        if min_front != float('inf') and min_front <= self.obstacle_distance:
            self._obstacle_latched = True
        elif min_front == float('inf') or min_front > self.obstacle_clear_distance:
            self._obstacle_latched = False
        self.obstacle_ahead = self._obstacle_latched

        self._lidar_debug_count += 1
        if self._lidar_debug_count % 40 == 1:
            self.get_logger().info(
                f'라이다 scan=[{msg.range_min:.2f},{msg.range_max:.2f}] '
                f'유효=[{rmin:.2f},{rmax:.2f}] '
                f'전방={self._fmt_dist(min_front)} '
                f'좌={self._fmt_dist(min_left)} '
                f'우={self._fmt_dist(min_right)} '
                f'장애물={self.obstacle_ahead}',
            )

    def _publish_vel(self, vx=0.0, vy=0.0, vz=0.0, wz=0.0):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.twist.linear.x = float(vx)
        msg.twist.linear.y = float(vy)
        msg.twist.linear.z = float(vz)
        msg.twist.angular.z = float(wz)
        self.velocity_pub.publish(msg)

    def _publish_idle_setpoint(self):
        if not self.is_offboard:
            self._publish_vel(0.0, 0.0, 0.0, 0.0)

    def _pick_clear_turn_sign(self):
        """전방 시야 안에서 좌/우 장애물 거리가 더 먼 쪽으로 회전."""
        left = self.min_left_dist if self.min_left_dist != float('inf') else 0.0
        right = self.min_right_dist if self.min_right_dist != float('inf') else 0.0
        if left >= right:
            self._clear_turn_sign = 1.0
            side = '좌'
        else:
            self._clear_turn_sign = -1.0
            side = '우'
        self.get_logger().warn(
            f'벽 감지 → {side}쪽 직진 경로 (좌={left:.2f}m 우={right:.2f}m '
            f'전방={self._fmt_dist(self.min_front_dist)})',
            throttle_duration_sec=0.5,
        )

    @staticmethod
    def _fmt_dist(d):
        return 'inf' if d == float('inf') else f'{d:.2f}m'

    def _log_wait_for_link(self):
        if not self.fcu_connected:
            self.get_logger().error(
                'PX4↔MAVROS 미연결 (FCU connected=false). '
                f'1) PX4/Gazebo 실행  2) fcu_url={self.fcu_url}  '
                '3) 확인: ros2 topic echo /mavros/state --once',
                throttle_duration_sec=5.0,
            )
            return
        if self.current_pose is None:
            self.get_logger().warn(
                'FCU는 연결됐으나 /mavros/local_position/pose 없음. '
                '시뮬·EKF 기동 대기 (라이다만 되는 상태는 정상일 수 있음)',
                throttle_duration_sec=5.0,
            )

    def _try_begin_takeoff_sequence(self):
        if self._auto_sequence_started or not self.auto_arm_offboard:
            return
        if not self.fcu_connected or self.current_pose is None:
            self._log_wait_for_link()
            return
        self._auto_sequence_started = True
        self.get_logger().info('PX4·pose 준비됨 → ARM·takeoff_local 시작')
        self._auto_arm_once()

    def _control_loop(self):
        if not self.fcu_connected or self.current_pose is None:
            self._log_wait_for_link()
            return
        alt = self.current_pose.pose.position.z
        dist_goal = self._horizontal_dist_to_goal()

        if self.phase == NavPhase.TAKEOFF:
            if not self.is_armed:
                self.get_logger().info(
                    f'ARM 대기 (z={alt:.2f} m)', throttle_duration_sec=2.0)
                return
            if not self._takeoff_requested:
                self.get_logger().info('takeoff 대기…', throttle_duration_sec=2.0)
                return
            if alt < self.flight_altitude - self.alt_tolerance:
                self.get_logger().info(
                    f'takeoff 상승 z={alt:.2f}/{self.flight_altitude:.1f} m',
                    throttle_duration_sec=1.0)
                return
            if not self.is_offboard:
                self._request_offboard_once()
                self.get_logger().info(
                    '이륙 완료 → OFFBOARD 전환', throttle_duration_sec=2.0)
                return
            self.phase = NavPhase.FORWARD
            self.get_logger().info('이륙·OFFBOARD 완료 → 직진')
            return

        if not self.is_armed:
            self.get_logger().info(
                f'대기: ARM 필요 (고도 {alt:.2f} m)', throttle_duration_sec=2.0)
            return
        if not self.is_offboard:
            self.get_logger().info(
                f'대기: OFFBOARD 필요 (고도 {alt:.2f} m)', throttle_duration_sec=2.0)
            return

        if self.phase == NavPhase.DONE:
            self._publish_vel(0.0, 0.0, 0.0, 0.0)
            self.get_logger().info(
                f'도착 유지 @ ({self.current_pose.pose.position.x:.2f}, '
                f'{self.current_pose.pose.position.y:.2f})',
                throttle_duration_sec=3.0,
            )
            return

        if dist_goal <= self.goal_tolerance:
            self.phase = NavPhase.DONE
            self._publish_vel(0.0, 0.0, 0.0, 0.0)
            self.get_logger().warn(
                f'목표 도착 (잔여 {dist_goal:.2f} m ≤ {self.goal_tolerance:.2f} m)')
            return

        if self.obstacle_ahead:
            if self.phase != NavPhase.TURN_CLEAR:
                self.phase = NavPhase.TURN_CLEAR
                self._pick_clear_turn_sign()
            self._publish_vel(0.0, 0.0, 0.0, self._clear_turn_sign * self.turn_rate)
            self.get_logger().info(
                f'회전 중(여유 큰 쪽) 전방={self._fmt_dist(self.min_front_dist)}',
                throttle_duration_sec=1.0,
            )
            return

        if self.phase == NavPhase.TURN_CLEAR:
            self.phase = NavPhase.FORWARD
            self.get_logger().info(
                f'전방 확보(>{self.obstacle_clear_distance:.1f}m) → 직진')

        self._publish_vel(self.cruise_velocity, 0.0, 0.0, 0.0)
        if self.phase != NavPhase.FORWARD:
            self.phase = NavPhase.FORWARD
        self.get_logger().info(
            f'직진 {self.cruise_velocity:.2f} m/s, 목표까지 {dist_goal:.2f} m, '
            f'전방 {self._fmt_dist(self.min_front_dist)}',
            throttle_duration_sec=1.0,
        )

    def _auto_arm_once(self):
        if self._auto_arm_timer is not None:
            self._auto_arm_timer.cancel()
            self._auto_arm_timer = None
        if not self.auto_arm_offboard:
            return
        self.get_logger().info('자동 ARM → takeoff_local...')
        if not self.arming_client.wait_for_service(timeout_sec=10.0):
            self.get_logger().error('/mavros/cmd/arming 응답 없음')
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
            self.get_logger().error('/mavros/cmd/takeoff_local 응답 없음')
            return
        self._takeoff_requested = True
        req = CommandTOLLocal.Request()
        req.min_pitch = 0.0
        req.offset = 0.0
        req.rate = float(self.takeoff_rate)
        req.yaw = 0.0
        req.position = Vector3(x=0.0, y=0.0, z=float(self.flight_altitude))
        self.get_logger().info(
            f'takeoff_local z={self.flight_altitude:.1f}m rate={self.takeoff_rate:.1f}m/s')
        self.takeoff_client.call_async(req).add_done_callback(self._on_takeoff_done)

    def _on_takeoff_done(self, future):
        try:
            resp = future.result()
            if resp.success:
                self.get_logger().info('takeoff_local 성공 (PX4 이륙 중)')
            else:
                self.get_logger().error(f'takeoff_local 실패 result={resp.result}')
        except Exception as e:
            self.get_logger().error(f'takeoff_local 예외: {e}')

    def _request_offboard_once(self):
        if self._offboard_requested:
            return
        self._offboard_requested = True
        if not self.set_mode_client.wait_for_service(timeout_sec=10.0):
            self.get_logger().error('/mavros/set_mode 응답 없음')
            return
        req = SetMode.Request()
        req.custom_mode = 'OFFBOARD'
        self.set_mode_client.call_async(req).add_done_callback(self._on_mode_done)

    def _on_mode_done(self, future):
        try:
            resp = future.result()
            if resp.mode_sent:
                self.get_logger().info('OFFBOARD 요청 전송됨')
            else:
                self.get_logger().warn('OFFBOARD mode_sent=False')
        except Exception as e:
            self.get_logger().error(f'OFFBOARD 예외: {e}')


def main(args=None):
    parser = argparse.ArgumentParser(description='목표 xy 자율주행 (고도 고정)')
    parser.add_argument('--goal-x', type=float, default=9.3)
    parser.add_argument('--goal-y', type=float, default=-6.8)
    parser.add_argument('--flight-altitude', type=float, default=5.0)
    parser.add_argument('--alt-tolerance', type=float, default=0.5)
    parser.add_argument(
        '--takeoff-rate',
        type=float,
        default=1.0,
        help='takeoff_local 상승 속도 (m/s)',
    )
    parser.add_argument('--cruise-velocity', type=float, default=0.5)
    parser.add_argument(
        '--obstacle-distance',
        type=float,
        default=2.0,
        help='전방 거리가 이 값 이하이면 정지·회피 (m)',
    )
    parser.add_argument(
        '--obstacle-clear-margin',
        type=float,
        default=0.3,
        help='장애물 해제 여유 (m). 해제 거리 = obstacle-distance + 이 값',
    )
    parser.add_argument(
        '--lidar-range-min',
        type=float,
        default=0.1,
        help='라이다로 인정할 최소 거리 (m)',
    )
    parser.add_argument(
        '--lidar-range-max',
        type=float,
        default=5.0,
        help='라이다로 인정할 최대 거리 (m). 이보다 먼 반사는 무시',
    )
    parser.add_argument(
        '--no-use-scan-range-limits',
        action='store_true',
        help='LaserScan.range_min/max 와 교집합하지 않고 CLI 값만 사용',
    )
    parser.add_argument(
        '--front-fov-deg',
        type=float,
        default=90.0,
        help='전방 라이다 시야 총각(°). 기본 90 → ±45° 만 사용',
    )
    parser.add_argument('--goal-tolerance', type=float, default=0.5)
    parser.add_argument('--turn-rate', type=float, default=0.35, help='벽 회피 회전 각속도 rad/s')
    parser.add_argument('--no-auto-arm-offboard', action='store_true')
    parser.add_argument(
        '--no-start-mavros',
        action='store_true',
        help='MAVROS를 띄우지 않음 (이미 다른 터미널에서 실행 중일 때)',
    )
    parser.add_argument(
        '--fcu-url',
        default=DEFAULT_FCU_URL,
        help=f'MAVROS fcu_url (기본: {DEFAULT_FCU_URL})',
    )
    parser.add_argument(
        '--mavros-wait-sec',
        type=float,
        default=8.0,
        help='MAVROS 기동 후 대기 시간(초)',
    )
    config = parser.parse_args(args=args)
    config.auto_arm_offboard = not config.no_auto_arm_offboard
    config.use_scan_range_limits = not config.no_use_scan_range_limits

    signal.signal(signal.SIGINT, _signal_handler)
    os.makedirs(ROS_LOG_DIR, exist_ok=True)
    os.environ.setdefault('ROS_LOG_DIR', ROS_LOG_DIR)

    setup_bash = os.path.join(MAVROS_WS, 'install', 'setup.bash')
    if not os.path.isfile(setup_bash):
        print(f'❌ 워크스페이스 없음: {setup_bash}')
        print('   source ~/mavros_ws/install/setup.bash 후 다시 실행하세요.')
        return 1

    if not config.no_start_mavros:
        if not start_mavros(config.fcu_url, config.mavros_wait_sec):
            return 1
    else:
        print('ℹ️  --no-start-mavros: 기존 MAVROS 사용')

    rclpy.init(args=None)
    node = GotoXyNavNode(config)
    print('\n' + '=' * 50)
    print('  goto_xy 자율주행  (chmy/test)')
    print('=' * 50)
    print(f'  cwd: {os.getcwd()}')
    print(f'  ws:  {MAVROS_WS}')
    print(f'  목표: ({config.goal_x}, {config.goal_y}) m')
    print(f'  고도: {config.flight_altitude} m, 속도: {config.cruise_velocity} m/s')
    print(f'  도착 허용: {config.goal_tolerance} m')
    print(
        f'  라이다: 최대 {config.lidar_range_max} m, 전방 {config.front_fov_deg}°, '
        f'장애물 판정 {config.obstacle_distance} m'
    )
    if not config.no_start_mavros:
        print(f'  MAVROS: 자동 기동 (fcu_url={config.fcu_url})')
    print('  ⚠ PX4/Gazebo 먼저 → ros2 topic echo /mavros/state --once')
    for line in affinity_env.summary_lines():
        print(f'  {line}')
    end_mavros = '' if config.no_start_mavros else ' (MAVROS도 종료)'
    print(f'  종료: Ctrl+C{end_mavros}')
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
            print('🧹 MAVROS 종료')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
