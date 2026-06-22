#!/usr/bin/env python3
"""
라이다 장애물 감지 시 정지 테스트

로그에 쓰는 시각 (직접 측정만 — stamp 또는 get_clock()):
  파일 로그 (ns / ms):
    ns: t1_bridge t2_ros t3_ros t4_mavros t5_sim t5_ros t6_ros
    ms: dt1_bridge_t5_sim t2t3 t3t4 t4t5_ros t5t6
  내부적으로는 t3_sim·라이다 폴백 등으로 t5_sim 축 유지 (상단 t3_sim/t5_sim 설명 참고)
  발생 순서: t1_bridge → t2_ros → t3_ros → t4_mavros → t5_sim/t5_ros → t6_ros
  t1_bridge : /lidar LaserScan.header.stamp (센서/브릿지가 찍은 시각)
  t2_ros    : 장애물 최초 판정 시각 get_clock()
  t3_ros    : 정지 setpoint publish 시각 get_clock() (노드에 use_sim_time 이 없으면 보통 wall)
  t3_sim    : /clock (rosgraph_msgs/Clock) 우선; 없으면 라이다 header.stamp 최신값( t1 과 동일 축, 스캔 주기만큼 시각이 약간 늦을 수 있음)
  t4_mavros : MAVROS setpoint 플러그인 CSV 3열 (corr=t1_bridge), 송신 직후 시각
  t5_sim    : /clock 우선; 없으면 라이다 최신 헤더; 그다음 브릿지·velocity_local.header
  t5_ros    : 감속 판정 순간 get_clock() (velocity_cb)
  t6_ros    : 정지 판정 순간 get_clock() (파일 ns 줄; t6_sim 은 내부용·파일 미기록)
  /clock 미수신 시 라이다 최신 스탬프로 t3_sim·t5_sim 폴백 (실기는 --no-gz-clock)
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, TwistStamped, Vector3Stamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode
from rosgraph_msgs.msg import Clock
import argparse
import math
import os
from pathlib import Path


class ObstacleStopNode(Node):
    def __init__(self, config):
        super().__init__('obstacle_stop_node')
        
        # 파라미터
        self.OBSTACLE_DISTANCE = config.obstacle_distance  # 장애물 감지 거리 (m)
        self.FRONT_ANGLE_RANGE = 30   # 전방 감지 각도 범위 (±도)
        self.CRUISE_VELOCITY = config.cruise_velocity  # 순항 속도 (m/s)
        self.FLIGHT_ALTITUDE = 5.0    # 비행 고도 (m)
        # True: 노드 시작 후 setpoint 몇 초 유지 뒤 /mavros 로 ARM + OFFBOARD 자동 요청
        self.AUTO_ARM_OFFBOARD = config.auto_arm_offboard
        self._auto_arm_timer = None
        # False: E2E는 t1→t5만 콘솔·파일 (t6·t2→t3 등 생략). True: 전 구간 상세
        self.LOG_FULL_LATENCY_CHAIN = config.log_full
        # E2E 여러 번: t5 기록 후 POST_E2E_WAIT_SEC 호버 → RETREAT_DISTANCE_M 만큼 후퇴 → 재전진
        self.REPEAT_E2E_ENABLED = True
        self.MAX_E2E_ITERATIONS = config.max_e2e_iterations
        self.RETREAT_DISTANCE_M = config.retreat_distance
        self.RETREAT_SPEED = config.retreat_speed
        self.POST_E2E_WAIT_SEC = config.post_wait_sec
        self.AUTO_EXIT_ON_COMPLETE = config.auto_exit_on_complete
        self.LOG_ALL_LIDAR = config.log_all_lidar
        self._batch_phase = None  # None | 'wait_post_e2e' | 'retreat'
        self._wait_post_e2e_start_ns = None
        self._retreat_xy0 = None
        self._t5_done_logged_pulse = False
        self._t5_recorded_at_ns = None  # 간략 모드: t5 후 t6 올 때까지 반복 전환 지연
        self._e2e_batch_complete = False  # MAX 회 도달 후 호버만
        self.lidar_msg_counter = 0
        self.trigger_lidar_msg_idx = None
        
        # 상태 변수
        self.current_state = State()
        self.obstacle_detected = False
        self.is_armed = False
        self.is_offboard = False
        self.current_pose = None
        self.min_obstacle_dist = float('inf')  # 전방 최소 거리
        
        # 레이턴시 측정용
        self.t1_lidar_stamp_ns = None        # t1: LaserScan.header.stamp (시뮬·센서 축; mavros corr 동일)
        self.t_lidar_rx_ros_ns = None        # 라idar_cb 진입 시각 (이 스캔 도착~t5까지 단일 ROS E2E)
        self.t2_obstacle_detected_ns = None  # t2: 장애물 판정 순간 ROS get_clock()
        self.t3_stop_published_ns = None     # 정지명령 publish 시점 (get_clock)
        self.t3_stop_sim_ns = None           # 정지명령 publish 시점 Gazebo (/clock), t1_bridge 와 동일 축
        self.t5_decel_start_ns = None        # t5_rx: 속도 감소 판정 순간 get_clock() (velocity_cb)
        self.t5_velocity_header_stamp_ns = None   # t5: 그 순간 속도 소스 메시지 header (시뮬 odom 등)
        self.t6_stopped_ns = None            # t6_rx: 정지 판정 순간 get_clock()
        self.t6_velocity_header_stamp_ns = None   # t6: 그 순간 속도 소스 메시지 header
        self.latency_measured = False        # t1~t3 측정 완료 여부
        self.t5_measured = False             # t5 측정 완료 여부
        self.t6_measured = False             # t6 측정 완료 여부
        self.measurement_count = 0           # 측정 횟수
        self.prev_velocity = None            # 이전 속도 (t5 감지용)
        self.stop_command_sent = False       # 정지 명령 전송 여부
        self.transfer_latency_us = 0         # 전송 레이턴시 (t2→t3)
        self._t4_ros_ns = None               # MAVROS CSV 3열(송신 직후); corr 매칭으로 로드
        raw_bridge = getattr(config, 't5_bridge_stamp_topic', None) or ''
        self.t5_stamp_bridge_topic = raw_bridge.strip()
        self.t5_stamp_bridge_type = getattr(
            config, 't5_bridge_msg_type', 'odom')
        self._t5_bridge_stamp_ns = None      # 브릿지 토픽 최신 header (ns)
        self._gz_clock_ns = None             # /clock 최신 시각 (Gazebo sim time, ns)
        self._lidar_last_sim_stamp_ns = None # /clock 없을 때 t3_sim/t5_sim 폴백 (LaserScan.header)
        self._warned_t5_axis_mismatch = False
        self._subscribe_gz_clock = getattr(config, 'subscribe_gz_clock', True)
        self._warned_t3_sim_no_clock = False
        self.gz_clock_topic = getattr(
            config, 'gz_clock_topic', '/clock').strip() or '/clock'
        npad = max(0, int(getattr(config, 'cmd_vel_frame_padding_digits', 0)))
        self.CMD_VEL_FRAME_PADDING_DIGITS = min(npad, 2_000_000)
        if self.CMD_VEL_FRAME_PADDING_DIGITS:
            self._cmd_vel_frame_id_suffix = (
                '_' + ('0' * self.CMD_VEL_FRAME_PADDING_DIGITS))
        else:
            self._cmd_vel_frame_id_suffix = ''

        # 로그 파일 설정
        from datetime import datetime
        log_dir = "/home/rtcl-chmy/mavros_ws/src/mavros/chmy/logs"
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.log_filename = f"{log_dir}/{timestamp}_obstacle_stop_latency.log"
        self.log_file = open(self.log_filename, 'w')
        self.lidar_log_filename = f"{log_dir}/{timestamp}_obstacle_lidar_frames.log"
        self.lidar_log_file = None
        if self.LOG_ALL_LIDAR:
            self.lidar_log_file = open(self.lidar_log_filename, 'w')
            self.lidar_log_file.write(
                "lidar_msg_idx,stamp_ns,min_distance_m,obstacle_detected\n")
        # 로그 파일 헤더
        self.log_file.write("=" * 60 + "\n")
        self.log_file.write("장애물 감지 → 정지 레이턴시 측정 로그\n")
        self.log_file.write("=" * 60 + "\n")
        if self._subscribe_gz_clock:
            clock_qos = QoSProfile(depth=10)
            clock_qos.reliability = ReliabilityPolicy.RELIABLE
            self.create_subscription(
                Clock,
                self.gz_clock_topic,
                self._clock_callback,
                clock_qos)
        t5_src = (
            f'우선 {self.gz_clock_topic} (Gazebo→ROS); '
            + ('/clock 없으면 라이다 header 최신( t1 축); '
               if self._subscribe_gz_clock else '라이다 header 최신( t1 축); ')
            + (
                f'다음 브릿지 {self.t5_stamp_bridge_topic} ({self.t5_stamp_bridge_type}); '
                if self.t5_stamp_bridge_topic
                else '')
            + '없으면 velocity_local.header')
        self.log_file.write(
            "# ns: t1_bridge t2_ros t3_ros t4_mavros t5_sim t5_ros [t6_ros]\n"
            f"#   t5_sim 출처: {t5_src}\n"
            "# ms: dt1_bridge_t5_sim t2t3 t3t4 t4t5_ros [t5t6]\n"
            "#   dt1_bridge_t5_sim = t5_sim - t1_bridge (시뮬 축; 축 불일치 시 na)\n"
            "#   t2t3,t3t4,t4t5_ros,t5t6 = ROS get_clock() 축 (t4_mavros 플러그인 CSV와 동일 축 가정)\n"
            f"#   cmd_vel frame_id 패딩: {self.CMD_VEL_FRAME_PADDING_DIGITS} 자리(숫자)\n\n")
        self.get_logger().info(f'📁 로그 파일: {self.log_filename}')
        
        # QoS 설정
        qos = QoSProfile(depth=10)
        qos.reliability = ReliabilityPolicy.BEST_EFFORT

        mavros_qos = QoSProfile(depth=10)
        mavros_qos.reliability = ReliabilityPolicy.BEST_EFFORT

        # Subscribers
        self.state_sub = self.create_subscription(
            State, '/mavros/state', self.state_callback, 10)
        self.pose_sub = self.create_subscription(
            PoseStamped, '/mavros/local_position/pose', self.pose_callback, mavros_qos)
        self.lidar_sub = self.create_subscription(
            LaserScan, '/lidar', self.lidar_callback, qos)
        self.velocity_sub = self.create_subscription(
            TwistStamped,
            '/mavros/local_position/velocity_local',
            self.velocity_callback,
            mavros_qos)
        if self.t5_stamp_bridge_topic:
            # /clock 이 없을 때를 대비한 보조 헤더 (시뮬 odom 등)
            bqos = qos_profile_sensor_data
            if self.t5_stamp_bridge_type == 'odom':
                self._t5_bridge_sub = self.create_subscription(
                    Odometry,
                    self.t5_stamp_bridge_topic,
                    self._t5_bridge_odom_cb,
                    bqos)
            else:
                self._t5_bridge_sub = self.create_subscription(
                    TwistStamped,
                    self.t5_stamp_bridge_topic,
                    self._t5_bridge_twist_cb,
                    bqos)
        else:
            self._t5_bridge_sub = None
        
        # Publishers
        self.velocity_pub = self.create_publisher(
            TwistStamped, '/mavros/setpoint_velocity/cmd_vel', 10)
        self.position_pub = self.create_publisher(
            PoseStamped, '/mavros/setpoint_position/local', 10)
        
        # Service Clients
        self.arming_client = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.set_mode_client = self.create_client(SetMode, '/mavros/set_mode')
        
        # 타이머 (50Hz)
        self.timer = self.create_timer(0.02, self.control_loop)
        
        # 초기 setpoint 발행용 타이머
        self.setpoint_timer = self.create_timer(0.1, self.publish_setpoint)
        
        if self.AUTO_ARM_OFFBOARD:
            # OFFBOARD 진입 전 setpoint 스트림이 필요하므로 잠시 후 ARM → OFFBOARD
            self._auto_arm_timer = self.create_timer(3.0, self._auto_arm_offboard_trigger)
        
        self.get_logger().info('=== 장애물 감지 정지 테스트 노드 시작 ===')
        self.get_logger().info(
            f'속도 판정: /mavros/local_position/velocity_local')
        if self._subscribe_gz_clock:
            self.get_logger().info(
                f'시뮬 시각: {self.gz_clock_topic} 구독 → 있으면 t3_sim·t5_sim; '
                '없으면 라이다 header 폴백')
        if self.t5_stamp_bridge_topic:
            self.get_logger().info(
                f't5_sim 보조 브릿지: {self.t5_stamp_bridge_topic} '
                f'(/clock 없을 때)')
        if not self._subscribe_gz_clock and not self.t5_stamp_bridge_topic:
            self.get_logger().warn(
                '/clock 미사용·브릿지 없음 → t3_sim/t5_sim 은 velocity_local 헤더 등 폴백')
        elif not self._subscribe_gz_clock:
            self.get_logger().info(
                '/clock 끔 — t5_sim 은 브릿지·velocity_local 순')
        self.get_logger().info(
            f'레이턴시 로그: t1_bridge·t2–t6_ros·t4_mavros·t5_sim + ms 구간 '
            f'(콘솔 {"상세" if self.LOG_FULL_LATENCY_CHAIN else "간략"})')
        self.get_logger().info(f'장애물 감지 거리: {self.OBSTACLE_DISTANCE}m')
        self.get_logger().info(f'전방 감지 각도: ±{self.FRONT_ANGLE_RANGE}°')
        if self.CMD_VEL_FRAME_PADDING_DIGITS:
            self.get_logger().warn(
                f'cmd_vel frame_id \'0\' 패딩 {self.CMD_VEL_FRAME_PADDING_DIGITS}자 — '
                't4 플러그인은 mavros 재빌드 필요(setpoint_velocity 접두사 매칭)')
        if self.REPEAT_E2E_ENABLED:
            self.get_logger().info(
                f'E2E 반복: 최대 {self.MAX_E2E_ITERATIONS}회, '
                f'후퇴 {self.RETREAT_DISTANCE_M}m @ {self.RETREAT_SPEED}m/s, '
                f'정지 대기 {self.POST_E2E_WAIT_SEC}s')
        
    def state_callback(self, msg):
        prev_armed = self.is_armed
        prev_offboard = self.is_offboard
        self.current_state = msg
        self.is_armed = msg.armed
        self.is_offboard = (msg.mode == 'OFFBOARD')
        
        if self.is_armed != prev_armed:
            self.get_logger().info(f'ARM 상태 변경: {self.is_armed}')
        if self.is_offboard != prev_offboard:
            self.get_logger().info(f'OFFBOARD 상태 변경: {self.is_offboard}')
        
    def pose_callback(self, msg):
        self.current_pose = msg

    def _cmd_vel_frame_id(self, base: str) -> str:
        """TwistStamped.header.frame_id 에 '0' 패딩 붙여 메시지 크기 확대 (부하 실험용)."""
        if not self._cmd_vel_frame_id_suffix:
            return base
        return base + self._cmd_vel_frame_id_suffix

    def _clear_measurement_flags_for_next_cycle(self):
        """한 번의 E2E 측정용 변수 초기화 (후퇴 시작 직전·후퇴 완료 후 공통)"""
        self.t1_lidar_stamp_ns = None
        self.t_lidar_rx_ros_ns = None
        self.t2_obstacle_detected_ns = None
        self.t3_stop_published_ns = None
        self.t3_stop_sim_ns = None
        self.t5_decel_start_ns = None
        self.t5_velocity_header_stamp_ns = None
        self.t6_stopped_ns = None
        self.t6_velocity_header_stamp_ns = None
        self.latency_measured = False
        self.t5_measured = False
        self.t6_measured = False
        self.stop_command_sent = False
        self.prev_velocity = None
        self.transfer_latency_us = 0
        self._t5_recorded_at_ns = None
        self._t4_ros_ns = None

    @staticmethod
    def _delta_ms(lo_ns, hi_ns):
        """(hi−lo)/1e6 ms; 인자 하나라도 없으면 None."""
        if lo_ns is None or hi_ns is None:
            return None
        return (hi_ns - lo_ns) / 1_000_000.0

    @staticmethod
    def _stamps_same_axis(a_ns, b_ns):
        """
        t1_bridge(시뮬·센서) vs t5_sim(velocity_local 등) 혼합 시 비교 불가.
        같은 시간축이면 보통 같은 자릿수 대(둘 다 ~1e10 또는 둘 다 ~1e18).
        """
        if a_ns is None or b_ns is None:
            return False
        lo, hi = (a_ns, b_ns) if a_ns <= b_ns else (b_ns, a_ns)
        if lo <= 0:
            return True
        return (hi / lo) < 1_000_000.0

    @staticmethod
    def _header_stamp_ns(header):
        """sensor_msgs/Header stamp → ns. (0,0)이면 None."""
        s = header.stamp
        if s.sec == 0 and s.nanosec == 0:
            return None
        return int(s.sec) * 1_000_000_000 + int(s.nanosec)

    def _clock_callback(self, msg: Clock):
        """Gazebo 가 퍼블리시하는 시뮬 시각 — 라이다 header.stamp 와 동일 축."""
        c = msg.clock
        if c.sec == 0 and c.nanosec == 0:
            self._gz_clock_ns = None
        else:
            self._gz_clock_ns = (
                int(c.sec) * 1_000_000_000 + int(c.nanosec))

    def _sim_time_ns_best(self):
        """시뮬 축 시각(ns): /clock 우선, 없으면 가장 최근 LaserScan.header."""
        if self._gz_clock_ns is not None:
            return self._gz_clock_ns
        return self._lidar_last_sim_stamp_ns

    def _resolve_t4_from_mavros_file(self, lidar_corr_ns):
        """mavros t4 로그 CSV 1열 corr가 lidar_corr_ns와 같은 줄의 3열(ns) 반환."""
        if lidar_corr_ns is None:
            return None
        log_root = Path(self.log_filename).resolve().parent
        try:
            candidates = sorted(
                log_root.glob("*_obstacle_stop_mavros_t4.log"),
                key=lambda p: p.stat().st_mtime,
                reverse=True)
        except OSError:
            return None
        want = int(lidar_corr_ns)
        for fp in candidates:
            try:
                with open(fp, encoding="utf-8", errors="replace") as fh:
                    for raw in fh:
                        line = raw.strip()
                        if not line or line.startswith("#"):
                            continue
                        parts = [p.strip() for p in line.split(",")]
                        if len(parts) < 3:
                            continue
                        try:
                            if int(parts[0]) != want:
                                continue
                        except ValueError:
                            continue
                        try:
                            return int(parts[2])
                        except ValueError:
                            continue
            except OSError:
                continue
        return None

    def _ensure_t4_ros_cached(self):
        if self._t4_ros_ns is not None:
            return
        if self.t1_lidar_stamp_ns:
            got = self._resolve_t4_from_mavros_file(self.t1_lidar_stamp_ns)
            if got is not None:
                self._t4_ros_ns = got

    def _ns_ms_compact_log(self, include_t6: bool) -> str:
        """파일 로그: ns + ms (dt1_bridge_t5_sim, t2t3, t3t4, t4t5_ros, t5t6)."""
        self._ensure_t4_ros_cached()
        t1b = self.t1_lidar_stamp_ns
        t2 = self.t2_obstacle_detected_ns
        t3 = self.t3_stop_published_ns
        t4 = self._t4_ros_ns
        t5s = self.t5_velocity_header_stamp_ns
        t5r = self.t5_decel_start_ns
        t6r = self.t6_stopped_ns if include_t6 else None

        def n(v):
            return str(v) if v is not None else "na"

        ns_line = (
            f"ns  t1_bridge={n(t1b)}  t2_ros={n(t2)}  t3_ros={n(t3)}  "
            f"t4_mavros={n(t4)}  t5_sim={n(t5s)}  t5_ros={n(t5r)}")
        if include_t6:
            ns_line += f"  t6_ros={n(t6r)}"
        else:
            ns_line += "  t6_ros=na"

        dt1_t5s = (
            self._delta_ms(t1b, t5s)
            if self._stamps_same_axis(t1b, t5s) else None)
        t2t3 = self._delta_ms(t2, t3)
        t3t4 = self._delta_ms(t3, t4)
        t4t5_ros = self._delta_ms(t4, t5r)
        t5t6 = self._delta_ms(t5r, t6r) if include_t6 else None

        def fms(d):
            return f"{d:.3f}" if d is not None else "na"

        parts = [
            f"dt1_bridge_t5_sim={fms(dt1_t5s)}",
            f"t2t3={fms(t2t3)}",
            f"t3t4={fms(t3t4)}",
            f"t4t5_ros={fms(t4t5_ros)}",
            f"t5t6={fms(t5t6)}",
        ]

        ms_line = "ms  " + "  ".join(parts)
        return ns_line + "\n" + ms_line

    def _formatted_interval_segments(self, include_t56=False):
        """예전 호출부 호환: compact 문자열."""
        return self._ns_ms_compact_log(include_t6=include_t56)

    def _try_start_repeat_after_e2e(self):
        """간략 모드: t5 직후가 아니라 t6(또는 타임아웃) 후 반복 시퀀스 진입."""
        if not self._t5_done_logged_pulse or not self.REPEAT_E2E_ENABLED:
            return False
        if (not self.LOG_FULL_LATENCY_CHAIN) and not self.t6_measured:
            age = self.get_clock().now().nanoseconds - (self._t5_recorded_at_ns or 0)
            if age < int(15e9):
                return False
            self.get_logger().warn(
                't6 미수신(15s 타임아웃) — 반복 시퀀스 진행')
        self._t5_done_logged_pulse = False
        self._t5_recorded_at_ns = None
        self._batch_phase = 'wait_post_e2e'
        self._wait_post_e2e_start_ns = self.get_clock().now().nanoseconds
        self._clear_measurement_flags_for_next_cycle()
        self.get_logger().warn(
            f'♻ 반복 준비: {self.POST_E2E_WAIT_SEC}s 호버 → '
            f'{self.RETREAT_DISTANCE_M}m 후퇴 → 재전진')
        return True

    def _t5_bridge_odom_cb(self, msg):
        ns = self._header_stamp_ns(msg.header)
        if ns is not None:
            self._t5_bridge_stamp_ns = ns

    def _t5_bridge_twist_cb(self, msg):
        ns = self._header_stamp_ns(msg.header)
        if ns is not None:
            self._t5_bridge_stamp_ns = ns

    def _t5_sim_stamp_ns(self, velocity_local_hdr_ns):
        """t5_sim/t6_sim: /clock → 라이다 최신 헤더 → 브릿지 → velocity_local.header."""
        u = self._sim_time_ns_best()
        if u is not None:
            return u
        if self.t5_stamp_bridge_topic and self._t5_bridge_stamp_ns is not None:
            return self._t5_bridge_stamp_ns
        return velocity_local_hdr_ns

    def velocity_callback(self, msg):
        """속도 모니터링 — velocity_local.linear.x; t5_sim 은 /clock 우선."""
        self._velocity_motion_sample(
            self._header_stamp_ns(msg.header), msg.twist.linear.x)

    def _velocity_motion_sample(self, src_header_stamp_ns, current_vel):
        """t5/t6 판정 (src_header_stamp_ns = velocity_local header)."""
        if self.stop_command_sent and self.t2_obstacle_detected_ns is not None:
            t1s = self.t1_lidar_stamp_ns

            if not self.t5_measured and self.prev_velocity is not None:
                if current_vel < self.prev_velocity - 0.01:
                    self.t5_decel_start_ns = self.get_clock().now().nanoseconds
                    if (self.t5_stamp_bridge_topic
                            and self._sim_time_ns_best() is None
                            and self._t5_bridge_stamp_ns is None):
                        self.get_logger().warn(
                            't5 브릿지 토픽에서 아직 header.stamp 를 못 받음 '
                            '(토픽명·타입·QoS 확인). t5_sim 은 velocity_local 헤더로 기록됨.')
                    self.t5_velocity_header_stamp_ns = self._t5_sim_stamp_ns(
                        src_header_stamp_ns)
                    self.t5_measured = True

                    t5_hdr = self.t5_velocity_header_stamp_ns
                    if (t1s is not None and t5_hdr is not None
                            and not self._stamps_same_axis(t1s, t5_hdr)
                            and not self._warned_t5_axis_mismatch):
                        self._warned_t5_axis_mismatch = True
                        self.get_logger().warn(
                            't1_bridge(라이다·시뮬 header)와 t5_sim 스탬프 축이 다릅니다 '
                            '(기본 velocity_local 은 wall/ROS 시간인 경우가 많음). '
                            't1 과 같은 식으로 쓰려면 시뮬이 찍는 '
                            'Odometry/TwistStamped 를 --t5-bridge-stamp-topic 에 지정하세요. '
                            '로그의 dt1_bridge_t5_sim 은 na 로 둡니다.')
                    if (t1s is not None and t5_hdr is not None
                            and self._stamps_same_axis(t1s, t5_hdr)):
                        e2e_hdr_t1_t5_ms = (t5_hdr - t1s) / 1_000_000
                    else:
                        e2e_hdr_t1_t5_ms = float('nan')

                    if self.LOG_FULL_LATENCY_CHAIN:
                        self.get_logger().warn(
                            f'⏱️  dt1_bridge_t5_sim={e2e_hdr_t1_t5_ms:.2f} ms')
                        self.get_logger().warn(
                            self._ns_ms_compact_log(include_t6=False))
                    else:
                        self.get_logger().warn(
                            f'⏱️  첫_속도감소  trigger_lidar_idx={self.trigger_lidar_msg_idx}')
                        self.measurement_count += 1
                        self.log_file.write(
                            f"[측정 #{self.measurement_count}] 첫_속도감소  "
                            f"trigger_lidar_idx={self.trigger_lidar_msg_idx}\n"
                            f"{self._ns_ms_compact_log(include_t6=False)}\n")
                        self.log_file.flush()
                        self.get_logger().info(
                            f'📁 측정 #{self.measurement_count} 저장 (t6 대기)')
                        if self.REPEAT_E2E_ENABLED:
                            self._t5_done_logged_pulse = True
                            self._t5_recorded_at_ns = self.t5_decel_start_ns

            if not self.t6_measured and self.t5_measured:
                if abs(current_vel) < 0.05:
                    self.t6_stopped_ns = self.get_clock().now().nanoseconds
                    self.t6_velocity_header_stamp_ns = self._t5_sim_stamp_ns(
                        src_header_stamp_ns)
                    self.t6_measured = True

                    if self.LOG_FULL_LATENCY_CHAIN:
                        self.get_logger().warn('정지완료')
                        self.get_logger().warn(
                            self._ns_ms_compact_log(include_t6=True))

                        self.measurement_count += 1
                        self.log_file.write(
                            f"[측정 #{self.measurement_count}] 정지완료\n"
                            f"{self._ns_ms_compact_log(include_t6=True)}\n\n")
                        self.log_file.flush()
                        self.get_logger().info(
                            f'📁 측정 #{self.measurement_count} 로그 저장됨')
                        if self.REPEAT_E2E_ENABLED:
                            self._t5_done_logged_pulse = True
                    else:
                        detail = self._ns_ms_compact_log(include_t6=True)
                        self.log_file.write(
                            f"[측정 #{self.measurement_count}] 정지완료\n"
                            f"{detail}\n")
                        self.log_file.flush()
                        self.get_logger().info(
                            f'📁 [측정 #{self.measurement_count}] 정지완료')

        self.prev_velocity = current_vel

    def lidar_callback(self, msg):
        """라이다 데이터 처리 - 전방 장애물만 감지"""
        rx_now_ns = self.get_clock().now().nanoseconds
        if len(msg.ranges) == 0:
            return
        lh = self._header_stamp_ns(msg.header)
        if lh is not None:
            self._lidar_last_sim_stamp_ns = lh
        self.lidar_msg_counter += 1
        
        # 디버그: 라이다 파라미터 출력 (한 번만)
        if not hasattr(self, '_lidar_debug_printed'):
            self._lidar_debug_printed = True
            self.get_logger().info(f'📡 라이다 정보: 샘플={len(msg.ranges)}, '
                                   f'각도=[{math.degrees(msg.angle_min):.1f}° ~ {math.degrees(msg.angle_max):.1f}°]')
        
        # 라이다 파라미터
        angle_min = msg.angle_min
        angle_increment = msg.angle_increment
        
        # 전방 범위 계산 (라디안) - 0도가 전방
        front_angle_rad = math.radians(self.FRONT_ANGLE_RANGE)
        
        min_distance = float('inf')
        
        for i, distance in enumerate(msg.ranges):
            # 현재 각도 계산 (라디안)
            angle = angle_min + i * angle_increment
            
            # 전방 범위 내인지 확인 (-front_angle ~ +front_angle)
            if -front_angle_rad <= angle <= front_angle_rad:
                # 유효한 거리 값인지 확인
                if distance > 0.1 and distance < 30.0 and not math.isinf(distance) and not math.isnan(distance):
                    if distance < min_distance:
                        min_distance = distance
        
        # 최소 거리 저장
        self.min_obstacle_dist = min_distance

        # 진단: ranges가 전부 0/inf면 장애물 판정 불가 (GZ·브릿지·센서 자세 확인용)
        if not hasattr(self, '_lidar_stat_count'):
            self._lidar_stat_count = 0
        self._lidar_stat_count += 1
        if self._lidar_stat_count % 25 == 1:
            n = len(msg.ranges)
            n_zero = sum(1 for d in msg.ranges if d == 0.0)
            n_inf = sum(1 for d in msg.ranges if math.isinf(d))
            finite = [
                d for d in msg.ranges
                if d > 0.01 and not math.isinf(d) and not math.isnan(d)]
            gmin = min(finite) if finite else float('nan')
            gmax = max(finite) if finite else float('nan')
            self.get_logger().info(
                f'📊 라이다 샘플 진단: N={n}, zero={n_zero}, inf={n_inf}, '
                f'유한개수={len(finite)}, 유한 min/max={gmin:.3f}/{gmax:.3f}, '
                f'range_limits=[{msg.range_min:.3f},{msg.range_max:.3f}], '
                f'전방콘 min={min_distance if min_distance != float("inf") else "inf"}')
        
        # 장애물 감지 판단
        prev_detected = self.obstacle_detected
        self.obstacle_detected = min_distance < self.OBSTACLE_DISTANCE
        if self.LOG_ALL_LIDAR and self.lidar_log_file is not None:
            stamp_ns = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
            min_dist_str = "inf" if min_distance == float('inf') else f"{min_distance:.6f}"
            self.lidar_log_file.write(
                f"{self.lidar_msg_counter},{stamp_ns},{min_dist_str},"
                f"{int(self.obstacle_detected)}\n"
            )
            if self.lidar_msg_counter % 50 == 0:
                self.lidar_log_file.flush()
        
        if self.obstacle_detected and not prev_detected:
            # t1: 센서/시뮬 헤더 (MAVROS corr·cmd_vel 헤더와 동일 숫자)
            self.t1_lidar_stamp_ns = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
            # 이 콜백에서 스캔 처리 시작 시각 — t5까지 동일 ROS(sim) 시계 E2E(t1rx_t5_ros)에 사용
            self.t_lidar_rx_ros_ns = rx_now_ns
            # t2: 장애물 최초 판정 시점 — ROS get_clock() (t3/t5/t6과 동일 축)
            now_ns = self.get_clock().now().nanoseconds
            self.t2_obstacle_detected_ns = now_ns
            self.trigger_lidar_msg_idx = self.lidar_msg_counter
            
            self.latency_measured = False  # 새 측정 시작
            
            self.get_logger().warn(
                f'⚠️  전방 장애물 감지! 거리: {min_distance:.2f}m '
                f'(lidar idx={self.trigger_lidar_msg_idx})')
            if self.LOG_FULL_LATENCY_CHAIN:
                self.get_logger().info(
                    f'📊 t1_sensor(header) ns: {self.t1_lidar_stamp_ns}')
                self.get_logger().info(f'📊 t2_ros(장애물 판정) ns: {now_ns}')
            
        elif not self.obstacle_detected and prev_detected:
            self.get_logger().info('✅ 전방 장애물 해제 - 이동 재개')
            # 후퇴/대기 중에는 여기서 초기화하지 않음 (배치 루프에서 처리)
            if self._batch_phase is None:
                self._clear_measurement_flags_for_next_cycle()
            
    def publish_setpoint(self):
        """OFFBOARD 모드 진입을 위한 setpoint 발행 (OFFBOARD 전에만)"""
        # OFFBOARD 모드가 아닐 때만 발행 (진입용)
        if not self.is_offboard:
            vel_msg = TwistStamped()
            vel_msg.header.stamp = self.get_clock().now().to_msg()
            vel_msg.header.frame_id = self._cmd_vel_frame_id('base_link')
            vel_msg.twist.linear.x = 0.0
            vel_msg.twist.linear.y = 0.0
            vel_msg.twist.linear.z = 0.0
            self.velocity_pub.publish(vel_msg)
        
    def control_loop(self):
        """메인 제어 루프"""
        if self.current_pose is None:
            self.get_logger().info('대기 중... (위치 데이터 없음)', throttle_duration_sec=2.0)
            return
            
        # 현재 고도
        current_alt = self.current_pose.pose.position.z
        
        # 속도 명령 생성
        vel_msg = TwistStamped()
        vel_msg.header.stamp = self.get_clock().now().to_msg()
        vel_msg.header.frame_id = self._cmd_vel_frame_id('base_link')
        
        if not self.is_armed:
            self.get_logger().info(f'대기 중... ARM 필요 (고도: {current_alt:.2f}m)', throttle_duration_sec=2.0)
            return
            
        if not self.is_offboard:
            self.get_logger().info(f'대기 중... OFFBOARD 모드 필요 (고도: {current_alt:.2f}m)', throttle_duration_sec=2.0)
            return

        # 목표 횟수 도달 후 호버만
        if self._e2e_batch_complete:
            vel_msg = TwistStamped()
            vel_msg.header.stamp = self.get_clock().now().to_msg()
            vel_msg.header.frame_id = self._cmd_vel_frame_id('base_link')
            vel_msg.twist.linear.x = 0.0
            vel_msg.twist.linear.y = 0.0
            vel_msg.twist.linear.z = 0.0
            self.velocity_pub.publish(vel_msg)
            self.get_logger().info(
                '🎯 목표 반복 측정 완료 — 정지 유지', throttle_duration_sec=5.0)
            if self.AUTO_EXIT_ON_COMPLETE:
                self.get_logger().info('✅ 반복 종료 조건 도달: 노드 종료 요청')
                rclpy.shutdown()
            return

        # t5(+간략:t6) 또는 --log-full(t6) 후 반복 시퀀스 진입
        if self._try_start_repeat_after_e2e():
            return

        if self._batch_phase == 'wait_post_e2e':
            elapsed = (
                self.get_clock().now().nanoseconds - self._wait_post_e2e_start_ns
            ) / 1e9
            vel_msg = TwistStamped()
            vel_msg.header.stamp = self.get_clock().now().to_msg()
            vel_msg.header.frame_id = self._cmd_vel_frame_id('base_link')
            vel_msg.twist.linear.x = 0.0
            vel_msg.twist.linear.y = 0.0
            vel_msg.twist.linear.z = 0.0
            self.velocity_pub.publish(vel_msg)
            if elapsed < self.POST_E2E_WAIT_SEC:
                self.get_logger().info(
                    f'⏳ 반복 대기 {elapsed:.1f}/{self.POST_E2E_WAIT_SEC}s',
                    throttle_duration_sec=0.5)
                return
            self._batch_phase = 'retreat'
            self._retreat_xy0 = (
                self.current_pose.pose.position.x,
                self.current_pose.pose.position.y)
            self.get_logger().warn(
                f'↩ 후퇴 시작 ({self.RETREAT_SPEED} m/s, 수평 {self.RETREAT_DISTANCE_M}m)')
            # 같은 주기에 아래 retreat 분기로 이어짐

        if self._batch_phase == 'retreat':
            px = self.current_pose.pose.position.x
            py = self.current_pose.pose.position.y
            dx = px - self._retreat_xy0[0]
            dy = py - self._retreat_xy0[1]
            dist = math.hypot(dx, dy)
            vel_msg = TwistStamped()
            vel_msg.header.stamp = self.get_clock().now().to_msg()
            vel_msg.header.frame_id = self._cmd_vel_frame_id('base_link')
            vel_msg.twist.linear.x = -self.RETREAT_SPEED
            vel_msg.twist.linear.y = 0.0
            vel_msg.twist.linear.z = 0.0
            self.velocity_pub.publish(vel_msg)
            self.get_logger().info(
                f'↩ 후퇴 {dist:.2f}/{self.RETREAT_DISTANCE_M}m',
                throttle_duration_sec=0.5)
            if dist >= self.RETREAT_DISTANCE_M:
                self._clear_measurement_flags_for_next_cycle()
                self._batch_phase = None
                if self.measurement_count >= self.MAX_E2E_ITERATIONS:
                    self._e2e_batch_complete = True
                    self.get_logger().warn(
                        f'✅ 후퇴 완료 — 목표 {self.MAX_E2E_ITERATIONS}세트 달성')
                else:
                    self.get_logger().warn('✅ 후퇴 완료 — 재전진')
            return
            
        # 고도 도달 여부 (한 번 도달하면 계속 전진 모드)
        if not hasattr(self, '_reached_altitude'):
            self._reached_altitude = False
        
        if current_alt >= self.FLIGHT_ALTITUDE - 0.5:
            self._reached_altitude = True
            
        if not self._reached_altitude:
            # 목표 고도까지 상승 (전진 없이 상승만)
            vel_msg.twist.linear.x = 0.0
            vel_msg.twist.linear.y = 0.0
            vel_msg.twist.linear.z = 1.0
            self.get_logger().info(f'상승 중... 고도: {current_alt:.2f}m / 목표: {self.FLIGHT_ALTITUDE}m', throttle_duration_sec=1.0)
        elif self.obstacle_detected:
            # 장애물 감지 - 정지 (호버링)
            vel_msg.twist.linear.x = 0.0
            vel_msg.twist.linear.y = 0.0
            vel_msg.twist.linear.z = 0.0
            
            # t3: 정지명령 publish 직전 시점 기록 (최초 1회만)
            if self.t1_lidar_stamp_ns and not self.latency_measured:
                self.t3_stop_published_ns = self.get_clock().now().nanoseconds
                self.t3_stop_sim_ns = self._sim_time_ns_best()
                if self.t3_stop_sim_ns is None and not self._warned_t3_sim_no_clock:
                    self._warned_t3_sim_no_clock = True
                    self.get_logger().warn(
                        't3_sim 기록 불가: /clock 미수신·라이다 헤더도 아직 없음. '
                        f'시뮬이면 ros_gz_bridge 등으로 {self.gz_clock_topic} 퍼블리시 확인.')

                # 메시지 헤더에 라이다 센서 타임스탬프 기록 (C++ 플러그인에서 t4 측정용)
                t1_sec = self.t1_lidar_stamp_ns // 1_000_000_000
                t1_nsec = self.t1_lidar_stamp_ns % 1_000_000_000
                vel_msg.header.stamp.sec = int(t1_sec)
                vel_msg.header.stamp.nanosec = int(t1_nsec)
                vel_msg.header.frame_id = self._cmd_vel_frame_id('obstacle_e2e')

                # 전송 레이턴시 (t2→t3, 동일 ROS 시계)
                self.transfer_latency_us = (self.t3_stop_published_ns - self.t2_obstacle_detected_ns) / 1000
                
                if self.LOG_FULL_LATENCY_CHAIN:
                    self.get_logger().info(
                        f'📊 t3 (정지명령 publish): {self.t3_stop_published_ns} ns')
                    self.get_logger().warn(
                        f'⏱️  t2→t3 (전송): {self.transfer_latency_us:.2f} μs')
                    self.get_logger().info('⏳ t5, t6 측정 대기...')
                self.latency_measured = True
                self.stop_command_sent = True  # t5, t6 측정 시작
            
            self.get_logger().warn(f'🛑 정지 중 (장애물 감지) - 최소거리: {self.min_obstacle_dist:.2f}m', throttle_duration_sec=1.0)
        else:
            # 전진 비행
            vel_msg.twist.linear.x = self.CRUISE_VELOCITY
            vel_msg.twist.linear.z = 0.0
            self.get_logger().info(f'전진 중... 속도: {self.CRUISE_VELOCITY}m/s, 전방거리: {self.min_obstacle_dist:.2f}m', throttle_duration_sec=1.0)
        
        self.velocity_pub.publish(vel_msg)

    def _auto_arm_offboard_trigger(self):
        """시작 후 1회: MAVROS ARM → OFFBOARD (PX4 콘솔 없이 테스트 가능)"""
        if self._auto_arm_timer is not None:
            self._auto_arm_timer.cancel()
            self._auto_arm_timer = None
        if not self.AUTO_ARM_OFFBOARD:
            return
        self.get_logger().info('자동 ARM + OFFBOARD 시도 (MAVROS 서비스)...')
        if not self.arming_client.wait_for_service(timeout_sec=10.0):
            self.get_logger().error(
                '/mavros/cmd/arming 응답 없음. MAVROS·PX4·UDP 연결을 확인하세요.')
            return
        req = CommandBool.Request()
        req.value = True
        fut = self.arming_client.call_async(req)
        fut.add_done_callback(self._on_auto_arm_done)

    def _on_auto_arm_done(self, future):
        try:
            resp = future.result()
            if not resp.success:
                self.get_logger().error(
                    f'ARM 실패: success={resp.success} result={resp.result} '
                    '(프리플라이트·연결·이륙 금지 등 PX4 상태 확인)')
                return
            self.get_logger().info('ARM 성공, OFFBOARD 요청...')
        except Exception as e:
            self.get_logger().error(f'ARM 예외: {e}')
            return
        if not self.set_mode_client.wait_for_service(timeout_sec=10.0):
            self.get_logger().error('/mavros/set_mode 응답 없음')
            return
        req = SetMode.Request()
        req.custom_mode = 'OFFBOARD'
        fut = self.set_mode_client.call_async(req)
        fut.add_done_callback(self._on_auto_mode_done)

    def _on_auto_mode_done(self, future):
        try:
            resp = future.result()
            if not resp.mode_sent:
                self.get_logger().warn(
                    f'OFFBOARD 요청이 FCU에 반영되지 않음: mode_sent={resp.mode_sent}')
            else:
                self.get_logger().info(
                    'OFFBOARD set_mode 전송됨 (적용까지 수 초 걸릴 수 있음)')
        except Exception as e:
            self.get_logger().error(f'OFFBOARD 예외: {e}')
        
    async def arm_drone(self):
        """드론 Arming"""
        self.get_logger().info('Arming 요청...')
        req = CommandBool.Request()
        req.value = True
        future = self.arming_client.call_async(req)
        return future
        
    async def set_offboard_mode(self):
        """OFFBOARD 모드 설정"""
        self.get_logger().info('OFFBOARD 모드 설정...')
        req = SetMode.Request()
        req.custom_mode = 'OFFBOARD'
        future = self.set_mode_client.call_async(req)
        return future


def main(args=None):
    parser = argparse.ArgumentParser(description='Obstacle stop E2E node')
    parser.add_argument('--max-e2e-iterations', type=int, default=100)
    parser.add_argument('--retreat-distance', type=float, default=5.0)
    parser.add_argument('--retreat-speed', type=float, default=0.35)
    parser.add_argument('--post-wait-sec', type=float, default=1.5)
    parser.add_argument('--obstacle-distance', type=float, default=3.0)
    parser.add_argument(
        '--cruise-velocity',
        type=float,
        default=0.5,
        help='전진 순항 속도 (m/s), /mavros/setpoint_velocity/cmd_vel linear.x',
    )
    parser.add_argument(
        '--cmd-vel-frame-padding-digits',
        type=int,
        default=4096,
        help=(
            'cmd_vel TwistStamped.header.frame_id 뒤에 붙일 \'0\' 개수 '
            '(예: 4096 ≈ frame_id 문자만 ~4kiB 추가; 0 이면 패딩 없음)'),
    )
    parser.add_argument('--log-full', action='store_true')
    parser.add_argument('--no-auto-arm-offboard', action='store_true')
    parser.add_argument('--auto-exit-on-complete', action='store_true')
    parser.add_argument('--log-all-lidar', action='store_true')
    parser.add_argument(
        '--gz-clock-topic',
        default='/clock',
        help=(
            '시뮬 시각 토픽 (rosgraph_msgs/Clock). 기본 /clock. '
            '토픽이 없으면 라이다 header 로 폴백'),
    )
    parser.add_argument(
        '--no-gz-clock',
        action='store_true',
        help='Gazebo /clock 미구독 (실기 등). t3_sim·t5_sim 은 브릿지·velocity_local 폴백',
    )
    parser.add_argument(
        '--t5-bridge-stamp-topic',
        default='',
        help=(
            '/clock 이 없을 때 t5_sim/t6_sim 용 header.stamp (시뮬 Odometry/Twist 등). '
            '시뮬에서는 보통 /clock 만으로 충분',
        ),
    )
    parser.add_argument(
        '--t5-bridge-msg-type',
        choices=('odom', 'twist'),
        default='odom',
        help='--t5-bridge-stamp-topic 메시지 타입',
    )
    config = parser.parse_args(args=args)
    config.auto_arm_offboard = not config.no_auto_arm_offboard
    config.subscribe_gz_clock = not config.no_gz_clock
    t = (config.gz_clock_topic or '/clock').strip()
    config.gz_clock_topic = t if t else '/clock'

    rclpy.init(args=None)
    node = ObstacleStopNode(config)
    
    print("\n" + "="*50)
    print("  라이다 장애물 감지 정지 테스트")
    print("="*50)
    print(f"  - 장애물 감지 거리: {node.OBSTACLE_DISTANCE}m")
    print(f"  - 전방 감지 각도: ±{node.FRONT_ANGLE_RANGE}°")
    print(f"  - 순항 속도: {node.CRUISE_VELOCITY}m/s")
    print(
        f"  - cmd_vel frame_id 패딩: {node.CMD_VEL_FRAME_PADDING_DIGITS}자리"
        if node.CMD_VEL_FRAME_PADDING_DIGITS else
        "  - cmd_vel frame_id 패딩: 없음")
    print(f"  - 비행 고도: {node.FLIGHT_ALTITUDE}m")
    print(
        f"  - 시뮬 시각: 토픽={node.gz_clock_topic} "
        f"({'구독' if node._subscribe_gz_clock else '끔'}; 없으면 라이다 header)")
    if node.t5_stamp_bridge_topic:
        print(
            f"  - t5_sim 보조: {node.t5_stamp_bridge_topic} ({node.t5_stamp_bridge_type})")
    print(
        f"  - 레이턴시: 판정=velocity_local, t1~t6 + MAVROS t4 (--log-full 시 콘솔 상세)")
    if node.REPEAT_E2E_ENABLED:
        print(f"  - E2E 반복: 최대 {node.MAX_E2E_ITERATIONS}회 (정지 후 {node.POST_E2E_WAIT_SEC}s → "
              f"{node.RETREAT_DISTANCE_M}m 후퇴 → 재전진)")
    print("="*50)
    print("\n[사용법]")
    if node.AUTO_ARM_OFFBOARD:
        print("  - 자동: 약 3초 후 MAVROS로 ARM + OFFBOARD 요청")
        print("  - 실패 시: PX4에서 commander arm / commander mode offboard")
    else:
        print("  1. PX4 콘솔: commander arm")
        print("  2. PX4 콘솔: commander mode offboard")
    print(f"  - 드론이 상승 후 전진 → 장애물({node.OBSTACLE_DISTANCE}m 이내) 시 정지")
    print("\n  종료: Ctrl+C")
    print("="*50 + "\n")
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n\n프로그램 종료...")
    finally:
        # 로그 파일 닫기
        if hasattr(node, 'log_file') and node.log_file:
            node.log_file.close()
            print(f"📁 로그 파일 저장 완료: {node.log_filename}")
            print(f"   총 {node.measurement_count}회 측정")
        if hasattr(node, 'lidar_log_file') and node.lidar_log_file:
            node.lidar_log_file.close()
            print(f"📁 라이다 프레임 로그 저장: {node.lidar_log_filename}")
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
