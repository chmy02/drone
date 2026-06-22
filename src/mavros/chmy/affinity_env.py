"""
CPU 코어 고정(taskset) — 환경변수만 설정하면 테스트 스크립트가 자동 적용.

설정 예: chmy/scripts/cpu_affinity.env.example 를 복사해 cpu_affinity.env 로 두고
  set -a && source .../cpu_affinity.env && set +a
  python3 0_run_multi_topic.py ...

변수 (비우면 taskset 미사용):
  CHMY_CPU_MAVROS       — ros2 run mavros …
  CHMY_CPU_TOPIC_NODES  — chmy/nodes/*.py 부하 노드
  CHMY_CPU_MAIN         — obstacle_stop / E2E main flow (5_end_to_end_measurement.py)
  CHMY_CPU_MONITOR      — system_monitor.py (미설정 시 CHMY_CPU_TOPIC_NODES 사용)

PX4·QGroundControl·Cursor 는 이 저장소 밖에서 띄우므로, 예시는 cpu_affinity.env.example 참고.
"""

from __future__ import annotations

import os
import shutil
from typing import List, Optional


def _cpus(name: str) -> str:
    return (os.environ.get(name, "") or "").strip()


def cpus_mavros() -> str:
    return _cpus("CHMY_CPU_MAVROS")


def cpus_topic_nodes() -> str:
    return _cpus("CHMY_CPU_TOPIC_NODES")


def cpus_main() -> str:
    return _cpus("CHMY_CPU_MAIN")


def cpus_monitor() -> str:
    return _cpus("CHMY_CPU_MONITOR") or cpus_topic_nodes()


def taskset_prefix(cpus: str) -> List[str]:
    if not cpus or not shutil.which("taskset"):
        return []
    return ["taskset", "-c", cpus]


def popen_argv_topic_python(script_path: str) -> List[str]:
    return taskset_prefix(cpus_topic_nodes()) + ["python3", script_path]


def popen_argv_main_python(argv: List[str]) -> List[str]:
    return taskset_prefix(cpus_main()) + argv


def bash_lc_mavros(inner_bash_lc: str) -> List[str]:
    """MAVROS: bash -lc 로 한 줄 실행. cpus 있으면 taskset 앞에 붙임."""
    c = cpus_mavros()
    cmd = ["bash", "-lc", inner_bash_lc]
    if c:
        if not shutil.which("taskset"):
            return cmd
        return ["taskset", "-c", c] + cmd
    return cmd


def summary_lines() -> List[str]:
    lines = []
    if not shutil.which("taskset"):
        lines.append("affinity: taskset 없음 — CHMY_CPU_* 무시")
        return lines
    m, t, main, mon = cpus_mavros(), cpus_topic_nodes(), cpus_main(), cpus_monitor()
    if not any((m, t, main, mon)):
        lines.append("affinity: CHMY_CPU_* 미설정 — 코어 고정 없음")
        return lines
    lines.append("affinity: taskset 활성")
    if m:
        lines.append(f"  CHMY_CPU_MAVROS={m}")
    if t:
        lines.append(f"  CHMY_CPU_TOPIC_NODES={t}")
    if main:
        lines.append(f"  CHMY_CPU_MAIN={main}")
    if mon and mon != t:
        lines.append(f"  CHMY_CPU_MONITOR={mon}")
    elif mon:
        lines.append(f"  CHMY_CPU_MONITOR=(TOPIC과 동일)")
    return lines
