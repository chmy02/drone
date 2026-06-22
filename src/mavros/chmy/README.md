# CHMY — MAVROS 레이턴시·장애물 정지 E2E 실험

PX4 SITL + Gazebo + MAVROS 환경에서 **토픽별 통신·처리 레이턴시**와 **라이다 기반 장애물 정지 end-to-end 반응시간**을 측정하는 실험 스택입니다.

경로: `mavros_ws/src/mavros/chmy/`

---

## 목차

1. [사전 요구사항](#사전-요구사항)
2. [설치](#설치)
3. [빌드](#빌드)
4. [시뮬레이션 실행 (PX4 + 라이다)](#시뮬레이션-실행-px4--라이다)
5. [디렉터리 구조](#디렉터리-구조)
6. [Python 테스트 실행](#python-테스트-실행)
7. [측정 시각 (t1~t6)](#측정-시각-t1t6)
8. [로그·분석](#로그분석)
9. [CPU 코어 고정 (선택)](#cpu-코어-고정-선택)
10. [트러블슈팅](#트러블슈팅)

---

## 사전 요구사항

| 구성요소 | 비고 |
|---------|------|
| **Ubuntu 22.04** | ROS 2 Humble 기준 |
| **ROS 2 Humble** | `source /opt/ros/humble/setup.bash` |
| **PX4-Autopilot** | `~/PX4-Autopilot`, Gazebo Harmonic(gz) SITL |
| **MAVROS** | 이 저장소(`mavros_ws`) — 플러그인에 레이턴시 로깅 패치 포함 |
| **ros_gz_bridge** | `sudo apt install ros-humble-ros-gz-bridge` |
| **Python 3** | `rclpy`, `pandas`, `openpyxl` (엑셀보내기용) |

```bash
pip3 install pandas openpyxl
```

선택: **QGroundControl** — 시뮬 모니터링·수동 ARM/이륙 확인용

---

## 설치

### 1. PX4-Autopilot

```bash
cd ~
git clone https://github.com/PX4/PX4-Autopilot.git --recursive
cd PX4-Autopilot
bash ./Tools/setup/ubuntu.sh
make px4_sitl
```

### 2. MAVROS 워크스페이스 (이 프로젝트)

```bash
cd ~
git clone https://github.com/chmy1205/RTCL.git mavros_ws
cd mavros_ws
# 또는 기존 클론: src/mavros 가 mavros 패키지 루트
```

의존성 설치:

```bash
cd ~/mavros_ws
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -y
```

---

## 빌드

레이턴시 CSV 로깅은 **C++ MAVROS 플러그인**에 들어 있으므로, 코드 수정 후 반드시 다시 빌드합니다.

```bash
cd ~/mavros_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select mavros_msgs libmavconn mavros mavros_extras --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

매 터미널에서 실험 전:

```bash
source /opt/ros/humble/setup.bash
source ~/mavros_ws/install/setup.bash
```

---

## 시뮬레이션 실행 (PX4 + 라이다)

실험은 보통 **터미널 3개**로 나눕니다.

### 터미널 1 — PX4 + Gazebo (벽·2D 라이다 월드)

```bash
cd ~/PX4-Autopilot
make px4_sitl gz_x500_lidar_2d__walls
```

PX4 콘솔이 뜨고 Gazebo가 기동될 때까지 대기합니다.

### 터미널 2 — Gazebo → ROS 라이다 브릿지

```bash
source /opt/ros/humble/setup.bash
ros2 run ros_gz_bridge parameter_bridge /lidar@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan
```

### 터미널 3 — 실험 스크립트

```bash
cd ~/mavros_ws/src/mavros/chmy/test
source ~/mavros_ws/install/setup.bash
python3 <스크립트>.py
```

### 동작 확인

```bash
ros2 topic echo /mavros/state --once          # connected: true
ros2 topic echo /lidar --once                 # LaserScan 수신
ros2 topic echo /mavros/local_position/pose --once
```

> **MAVROS**: 대부분의 테스트 스크립트가 `udp://:14540@127.0.0.1:14557` 로 MAVROS를 **자동 기동**합니다.  
> 수동 실행 시: `ros2 run mavros mavros_node --ros-args -p fcu_url:=udp://:14540@127.0.0.1:14557`

---

## 디렉터리 구조

```
chmy/
├── README.md                 # 이 문서
├── affinity_env.py           # CPU affinity 헬퍼
├── nodes/                    # 토픽 부하·레이턴시 측정용 Python 노드 (0~12)
├── auto_drive_test/
│   └── obstacle_stop_test.py # 장애물 정지 E2E 측정 노드
├── test/                     # 실험 러너·분석 스크립트
├── scripts/                  # cpu_affinity.env 예시, affinity 셸 래퍼
└── logs/                     # 실험 로그 (git 제외, 로컬 생성)
```

### 토픽 번호 (0~12)

| 번호 | MAVROS 토픽/서비스 | 노드 스크립트 |
|------|-------------------|---------------|
| 0 | `cmd/arming` (서비스) | `command_arm.py` |
| 1 | `setpoint_raw/local` | `setpoint_raw_local.py` |
| 2 | `setpoint_velocity/cmd_vel` | `setpoint_velocity_cmd_vel.py` |
| 3 | `setpoint_position/local` | `setpoint_position_local.py` |
| 4 | `setpoint_raw/attitude` | `setpoint_raw_attitude.py` |
| 5 | `setpoint_accel/accel` | `setpoint_accel_accel.py` |
| 6 | `setpoint_raw/global` | `setpoint_raw_global.py` |
| 7 | `setpoint_trajectory/local` | `setpoint_trajectory_local.py` |
| 8 | `manual_control/send` | `manual_control_send.py` |
| 9 | `setpoint_position/global` | `setpoint_position_global.py` |
| 10 | `actuator_control` | `actuator_control.py` |
| 11 | `vision_pose/pose` (extras) | `vision_pose_pose.py` |
| 12 | `mocap/pose` (extras) | `mocap_pose.py` |

---

## Python 테스트 실행

작업 디렉터리:

```bash
cd ~/mavros_ws/src/mavros/chmy/test
source ~/mavros_ws/install/setup.bash
```

### A. 다중 토픽 레이턴시 부하 (`0_run_multi_topic.py`)

MAVROS + 선택한 토픽 노드를 동시에 띄워 **t1~t4** 레이턴시 로그를 생성합니다.

```bash
# 서비스(0) + 토픽 1,2
python3 0_run_multi_topic.py 0 1 2

# 토픽 1~5
python3 0_run_multi_topic.py 0-5

# 전체 13개 (0 + 1~12), 시작 간격·안정화·실험 시간 조절
python3 0_run_multi_topic.py -i 0.5 -w 3 -d 120

# Extras 먼저 기동 후 기본 플러그인
python3 0_run_multi_topic.py 11 12 1-10
```

옵션: `-i` 노드 시작 간격(초), `-w` 워밍업(초), `-d` 실험 시간(초)

### B. 장애물 정지 E2E (`5_end_to_end_measurement.py`)

라이다 장애물 감지 → 정지까지 **t1~t6** 체인을 반복 측정합니다. MAVROS·부하 토픽은 1회만 기동합니다.

```bash
# 1회 E2E (부하 토픽 없음)
python3 5_end_to_end_measurement.py 1

# 10회 반복 + 토픽 1,2,4 동시 부하
python3 5_end_to_end_measurement.py 10 1 2 4
```

단독 노드 실행 (디버그):

```bash
python3 ../auto_drive_test/obstacle_stop_test.py --help
```

### C. 자율 주행·이륙 테스트

PX4 + `/lidar` 브릿지가 떠 있어야 합니다.

```bash
# takeoff_local 이륙 → OFFBOARD → 라이다 회피 → 목표 (x,y)
python3 7_goto_xy_nav.py

# 이륙만 검증
python3 8_takeoff_only.py
```

`7_goto_xy_nav.py` 주요 기본값: 고도 5 m, 장애물 거리 2 m, 라이다 최대 5 m·전방 FOV 90°.

### D. 로그 분석·정리

```bash
# 다중 토픽 latency 통계 (logs/ 최신 *.log)
python3 2_analyze_multi_topic.py

# 13토픽 로그 → 엑셀 (logs/excel/)
python3 3_export_to_excel_sheets.py

# E2E 장애물 로그 → 엑셀
python3 4_export_obstacle_e2e_to_excel.py

# logs/ 루트 플랫 파일을 타임스탬프 폴더로 묶기
python3 6_log_folder.py
```

### E. 기타

| 스크립트 | 설명 |
|---------|------|
| `0_run_topic.py` | 단일 토픽 실행 |
| `1_run_topic_auto.py` | 토픽 자동 순회 |
| `0_merge_service_log.py` | 서비스(0) 로그 병합 |
| `system_monitor.py` | CPU 모니터링 (멀티토픽 실험과 병행 가능) |

---

## 측정 시각 (t1~t6)

장애물 정지 E2E (`obstacle_stop_test.py`) 기준:

| 시각 | 의미 |
|------|------|
| **t1_bridge** | `/lidar` `LaserScan.header.stamp` |
| **t2_ros** | 장애물 최초 판정 (`get_clock()`) |
| **t3_ros** | 정지 setpoint publish 시각 |
| **t4_mavros** | MAVROS `setpoint_velocity` 플러그인 송신 직후 (C++ CSV) |
| **t5_sim / t5_ros** | 시뮬/ROS 감속 판정 |
| **t6_ros** | 완전 정지 판정 |

토픽 부하 실험 (플러그인 CSV): **t1** 메시지 stamp → **t3** ROS 수신 → **t4** MAVLink 송신.

E2E 정지 명령은 `frame_id`에 `obstacle_e2e` 태그가 붙으며, `setpoint_velocity` 플러그인이 `*_obstacle_stop_mavros_t4.log` 에 t4를 기록합니다.

---

## 로그·분석

- 기본 출력: `chmy/logs/`
- 파일 예: `YYYYMMDD_HHMMSS_topic2_setpoint_velocity_cmd_vel_latency.log`
- E2E: `YYYYMMDD_HHMMSS_obstacle_stop_latency.log`, `*_obstacle_stop_mavros_t4.log`
- `logs/` 는 `.gitignore` 대상 — 실험 후 로컬에만 쌓입니다.

정리:

```bash
python3 6_log_folder.py    # 루트에 흩어진 로그를 YYMMDD_HHMMSS 폴더로 이동
```

---

## CPU 코어 고정 (선택)

재현성을 위해 프로세스를 코어에 묶을 수 있습니다.

```bash
cp ~/mavros_ws/src/mavros/chmy/scripts/cpu_affinity.env.example \
   ~/mavros_ws/src/mavros/chmy/scripts/cpu_affinity.env
# lscpu 로 코어 번호 편집

set -a && source ~/mavros_ws/src/mavros/chmy/scripts/cpu_affinity.env && set +a
python3 0_run_multi_topic.py 1 2
```

또는:

```bash
~/mavros_ws/src/mavros/chmy/scripts/run_multi_topic_affinity.sh 1 2
~/mavros_ws/src/mavros/chmy/scripts/run_e2e_affinity.sh 5 1 2
```

환경 변수: `CHMY_CPU_MAVROS`, `CHMY_CPU_TOPIC_NODES`, `CHMY_CPU_MAIN`

---

## 트러블슈팅

### `connected: false` (FCU 미연결)

1. PX4 SITL이 먼저 떠 있는지 확인  
2. MAVROS `fcu_url` 이 PX4 UDP 포트와 일치하는지 확인 (`14540` / `14557`)  
3. `commander land` 후에는 Disarm 상태 — 스크립트를 **다시 실행**하거나 PX4 재시작

### `/lidar` 토픽 없음

- `make px4_sitl gz_x500_lidar_2d__walls` 월드인지 확인  
- 터미널 2에서 `ros_gz_bridge parameter_bridge` 가 실행 중인지 확인

### pose / state 구독 안 됨 (QoS)

MAVROS pose는 **SensorDataQoS** 입니다. 노드에서 `qos_profile_sensor_data` 를 사용해야 합니다 (`7_goto_xy_nav.py` 등 참고).

### t4_mavros 로그가 비어 있음

`colcon build` 후 `source install/setup.bash` 를 다시 했는지 확인.  
정지 setpoint의 `frame_id`가 `obstacle_e2e` 인지 확인.

### Python `ModuleNotFoundError: mavros_msgs`

```bash
source ~/mavros_ws/install/setup.bash
```

---

## 표준 실험 순서 (요약)

```bash
# 1) PX4
cd ~/PX4-Autopilot && make px4_sitl gz_x500_lidar_2d__walls

# 2) 라이다 브릿지
ros2 run ros_gz_bridge parameter_bridge /lidar@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan

# 3) 실험
cd ~/mavros_ws/src/mavros/chmy/test
source ~/mavros_ws/install/setup.bash
python3 5_end_to_end_measurement.py 1    # E2E
# 또는
python3 7_goto_xy_nav.py                   # 자율주행
# 또는
python3 0_run_multi_topic.py 1 2 3         # 토픽 부하
```

---

## 라이선스·출처

MAVROS 본체는 [mavros/mavros](https://github.com/mavros/mavros) 라이선스를 따릅니다.  
`chmy/` 실험 코드는 RTCL 연구용 확장입니다.
