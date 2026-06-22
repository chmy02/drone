#!/usr/bin/env python3
"""
0.1초마다 시스템 상태(CPU 등)를 기록하는 스크립트.
레이턴시 측정과 별도로 돌려서, 스파이크 시점의 시스템 부하를 분석할 때 사용.

출력 형식: timestamp_ns,cpu_total,proc1:pid:cpu%;proc2:pid:cpu%;...
- timestamp_ns: time.time_ns() (레이턴시 로그의 t1과 같은 시간축)
- cpu_total: 전체 CPU (%)
- proc:name:pid:cpu% 세미콜론으로 구분, CPU 상위 N개

사용법:
  python3 system_monitor.py --duration 60 --interval 0.1
  python3 system_monitor.py -d 60 -i 0.1 -n 20 -o /path/to/output.log
"""

import argparse
import re
import time
import os
from datetime import datetime

try:
    import psutil
except ImportError:
    print("pip install psutil 필요")
    exit(1)

LOG_DIR = "/home/rtcl-chmy/mavros_ws/src/mavros/chmy/logs"


def _sanitize(s):
    """로그 구분자와 충돌하는 문자 제거"""
    if not s:
        return "unknown"
    return re.sub(r'[,;:|]', '_', str(s))[:48]


def get_top_processes(n=20):
    """CPU 사용량 상위 N개 프로세스 반환 (name, pid, cpu%)"""
    procs = []
    for proc in psutil.process_iter(['pid', 'name', 'exe', 'cmdline']):
        try:
            cpu = proc.cpu_percent()
            if cpu <= 0:
                continue
            name = proc.info.get('name') or ''
            if not name and proc.info.get('exe'):
                name = os.path.basename(proc.info['exe'])
            if not name:
                cmdline = proc.info.get('cmdline') or []
                name = os.path.basename(cmdline[0]) if cmdline else f"pid{proc.info['pid']}"
            procs.append((_sanitize(name), proc.info['pid'], cpu))
        except (psutil.NoSuchProcess, psutil.AccessDenied, psutil.ZombieProcess):
            pass
    procs.sort(key=lambda x: x[2], reverse=True)
    return procs[:n]


def main():
    ap = argparse.ArgumentParser(description='0.1초마다 시스템 상태 기록 (전체 프로세스 상위 N개)')
    ap.add_argument('-d', '--duration', type=float, default=60,
                    help='기록 시간 (초), 기본 60')
    ap.add_argument('-i', '--interval', type=float, default=0.1,
                    help='샘플링 간격 (초), 기본 0.1')
    ap.add_argument('-n', '--top', type=int, default=20,
                    help='CPU 상위 N개 프로세스 기록 (기본 20)')
    ap.add_argument('-o', '--output', type=str, default=None,
                    help='출력 파일 경로 (기본: logs/YYYYMMDD_HHMMSS_system_monitor.log)')
    args = ap.parse_args()

    os.makedirs(LOG_DIR, exist_ok=True)
    if args.output:
        out_path = args.output
    else:
        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        out_path = os.path.join(LOG_DIR, f"{ts}_system_monitor.log")

    psutil.cpu_percent()  # 초기화

    n_samples = int(args.duration / args.interval)
    print(f"시스템 모니터 시작: {args.duration}초, {args.interval}s 간격, 상위 {args.top}개 프로세스 → {out_path}")

    with open(out_path, 'w') as f:
        for i in range(n_samples):
            t_ns = time.time_ns()
            cpu_total = psutil.cpu_percent()
            top = get_top_processes(args.top)
            parts = [f"{name}:{pid}:{cpu:.1f}" for name, pid, cpu in top]
            procs_str = ';'.join(parts) if parts else ''
            f.write(f"{t_ns},{cpu_total:.1f},{procs_str}\n")
            if (i + 1) % 100 == 0:
                print(f"   {i+1}/{n_samples} 샘플 기록...")
            time.sleep(args.interval)

    print(f"✅ 완료: {out_path}")


if __name__ == '__main__':
    main()
