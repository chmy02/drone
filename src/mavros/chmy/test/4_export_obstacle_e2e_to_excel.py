#!/usr/bin/env python3
"""
*_obstacle_stop_latency.log → 단일 xlsx
  - 시트 Raw: 측정 번호, ms 줄의 dt1_bridge_t5_sim(또는 구형 t1_bridge_t5_sim), 구형 시 t2–t5_rx 등
  - 시트 Summary: mean/median/…

형식 (v2, 축별 블록; E2E_ros 줄 없음):
  [측정 #N] 첫_속도감소  trigger_lidar_idx=…
  ns …
  ms … dt1_bridge_t5_sim=… (구 로그는 t1_bridge_t5_sim=)

구형 한 줄 (호환): E2E_ros t2→t5 | E2E_stamp …  (로그 파일이 옛날 것일 때)

레거시: E2E t1→t5 …

사용:
  python3 4_export_obstacle_e2e_to_excel.py              # logs 폴더에서 최신 obstacle 로그
  python3 4_export_obstacle_e2e_to_excel.py /path/to.log # 특정 파일
"""

import argparse
import glob
import os
import re

import pandas as pd

LOG_DIR = "/home/rtcl-chmy/mavros_ws/src/mavros/chmy/logs"

# v2: 첫_속도감소 블록 — ms 줄에서 dt1_bridge_t5_sim 또는 t1_bridge_t5_sim 추출
BLOCK_RE_V2 = re.compile(
    r"\[측정 #(\d+)\]\s*첫_속도감소[^\n]*\n"
    r"ns[^\n]+\n"
    r"ms[^\n]*(?:dt1_bridge_t5_sim|t1_bridge_t5_sim)=([\d.]+|na)",
    re.MULTILINE,
)

# 구형 장문 로그: 첫_속도감소 직후 한 줄에 E2E_ros(t2→t5_rx)=… (엑셀에는 t2_t5_rx_ms 로만 기록)
BLOCK_RE_V1_VERBOSE = re.compile(
    r"\[측정 #(\d+)\]\s*첫_속도감소\s*\n"
    r"E2E_ros\(t2→t5_rx\)=([\d.]+|nan)\s*ms",
    re.MULTILINE,
)

LINE_RE_NEW = re.compile(
    r"\[측정 #(\d+)\]\s*"
    r"E2E_ros t2→t5:\s*([\d.]+)\s*ms\s*\|\s*"
    r"E2E_stamp t1→t5:\s*([\d.]+|nan)\s*ms\s*\|\s*"
    r"(?:E2E_hdr t1→t5_hdr:\s*([\d.]+|nan)\s*ms\s*\|\s*)?"
    r"t1_sensor_ns=(\d+)\s+t2_ns=(\d+)\s+t3_ns=(\d+)\s+t5_ns=(\d+)"
)

LINE_RE_OLD = re.compile(
    r"\[측정 #(\d+)\]\s*E2E t1→t5:\s*([\d.]+)\s*ms\s*\|\s*"
    r"t1_ns=(\d+)\s+t2_ns=(\d+)\s+t3_ns=(\d+)\s+t5_ns=(\d+)\s+lidar_corr_ns=(\d+)"
)


def _parse_float_maybe(s: str) -> float:
    s = s.strip()
    if s.lower() == "nan":
        return float("nan")
    return float(s)


def _summarize(series: pd.Series, name: str) -> dict:
    s = series.dropna()
    if s.empty:
        return {"metric": name, "note": "no values"}
    return {
        "metric": name,
        "count": int(s.shape[0]),
        "mean_ms": float(s.mean()),
        "median_ms": float(s.median()),
        "std_ms": float(s.std()) if len(s) > 1 else 0.0,
        "min_ms": float(s.min()),
        "max_ms": float(s.max()),
        "p95_ms": float(s.quantile(0.95)),
        "p99_ms": float(s.quantile(0.99)),
    }


def parse_obstacle_line(line: str):
    """구형 한 줄 (옛 로그 호환)."""
    m = LINE_RE_NEW.search(line)
    if m:
        idx, t2t5_ms, t1t5_stamp_ms, t1t5_hdr_ms, t1s, t2, t3, t5 = m.groups()
        t1_si = int(t1s)
        t2_i = int(t2)
        t5_i = int(t5)
        stamp = _parse_float_maybe(t1t5_stamp_ms)
        hdr_ms = (
            float("nan")
            if t1t5_hdr_ms is None
            else _parse_float_maybe(t1t5_hdr_ms)
        )
        return {
            "measurement_id": int(idx),
            "format": "new_oneline",
            "t1_bridge_t5_sim_ms_logged": float("nan"),
            "t2_t5_rx_ms_logged": float(t2t5_ms),
            "t1_t5_stamp_ms_logged": stamp,
            "t1_t5_hdr_ms_logged": hdr_ms,
            "t1_sensor_ns": t1_si,
            "t2_ns": t2_i,
            "t3_ns": int(t3),
            "t5_rx_ns": t5_i,
            "t2_t5_rx_verify_ms": round((t5_i - t2_i) / 1_000_000.0, 6),
        }
    m = LINE_RE_OLD.search(line)
    if not m:
        return None
    idx, e2e_ms_old, _t1_old, t2, t3, t5, lc = m.groups()
    t1_sensor = int(lc)
    t2_i = int(t2)
    t5_i = int(t5)
    e2e_ros = (t5_i - t2_i) / 1_000_000.0
    e2e_mix = (t5_i - t1_sensor) / 1_000_000.0
    return {
        "measurement_id": int(idx),
        "format": "legacy",
        "t1_bridge_t5_sim_ms_logged": float("nan"),
        "t2_t5_rx_ms_logged": float(e2e_ms_old),
        "t1_t5_stamp_ms_logged": round(e2e_mix, 6),
        "t1_t5_hdr_ms_logged": float("nan"),
        "t1_sensor_ns": t1_sensor,
        "t2_ns": t2_i,
        "t3_ns": int(t3),
        "t5_rx_ns": t5_i,
        "t2_t5_rx_verify_ms": round(e2e_ros, 6),
        "legacy_t1_ros_ns": int(_t1_old),
    }


def parse_obstacle_log(path: str) -> pd.DataFrame:
    rows = []
    with open(path, encoding="utf-8", errors="replace") as f:
        text = f.read()
    for m in BLOCK_RE_V2.finditer(text):
        idx, t1b5s = m.groups()
        hlogged = float("nan") if t1b5s == "na" else _parse_float_maybe(t1b5s)
        rows.append(
            {
                "measurement_id": int(idx),
                "format": "v2_block",
                "t1_bridge_t5_sim_ms_logged": hlogged,
                "t2_t5_rx_ms_logged": float("nan"),
                "t1_t5_stamp_ms_logged": float("nan"),
                "t1_t5_hdr_ms_logged": float("nan"),
                "t1_sensor_ns": None,
                "t2_ns": None,
                "t3_ns": None,
                "t5_rx_ns": None,
                "t2_t5_rx_verify_ms": None,
            }
        )
    if not rows:
        for m in BLOCK_RE_V1_VERBOSE.finditer(text):
            idx, t2t5s = m.groups()
            rows.append(
                {
                    "measurement_id": int(idx),
                    "format": "v1_verbose_block",
                    "t1_bridge_t5_sim_ms_logged": float("nan"),
                    "t2_t5_rx_ms_logged": _parse_float_maybe(t2t5s),
                    "t1_t5_stamp_ms_logged": float("nan"),
                    "t1_t5_hdr_ms_logged": float("nan"),
                    "t1_sensor_ns": None,
                    "t2_ns": None,
                    "t3_ns": None,
                    "t5_rx_ns": None,
                    "t2_t5_rx_verify_ms": None,
                }
            )
    if not rows:
        with open(path, encoding="utf-8", errors="replace") as f:
            for line in f:
                row = parse_obstacle_line(line)
                if row:
                    rows.append(row)
    return pd.DataFrame(rows)


def find_latest_obstacle_log():
    pattern = os.path.join(LOG_DIR, "*_obstacle_stop_latency.log")
    files = glob.glob(pattern)
    if not files:
        return None
    return max(files, key=os.path.getmtime)


def summary_stats(df: pd.DataFrame) -> pd.DataFrame:
    if df.empty:
        return pd.DataFrame([{"note": "no data rows"}])
    out = []
    t15 = pd.Series(df["t1_bridge_t5_sim_ms_logged"], dtype=float)
    if t15.notna().any():
        out.append(_summarize(t15, "t1_bridge_t5_sim_ms_logged"))
    t2t5 = pd.Series(df["t2_t5_rx_ms_logged"], dtype=float)
    if t2t5.notna().any():
        out.append(_summarize(t2t5, "t2_t5_rx_ms_logged"))
    if not out:
        return pd.DataFrame([{"note": "no numeric columns"}])
    return pd.DataFrame(out)


def main():
    ap = argparse.ArgumentParser(description="obstacle_stop_latency.log → xlsx")
    ap.add_argument(
        "logfile",
        nargs="?",
        default=None,
        help="로그 파일 경로 (생략 시 logs 폴더에서 최신 obstacle 로그)",
    )
    ap.add_argument(
        "-o",
        "--output",
        default=None,
        help="출력 xlsx 경로 (기본: 로그와 같은 타임스탬프로 LOG_DIR에 저장)",
    )
    args = ap.parse_args()

    log_path = args.logfile or find_latest_obstacle_log()
    if not log_path or not os.path.isfile(log_path):
        print("❌ obstacle_stop_latency.log 를 찾을 수 없습니다.")
        return 1

    df = parse_obstacle_log(log_path)
    if df.empty:
        print(f"❌ 파싱된 측정 행이 없습니다: {log_path}")
        return 1

    base = os.path.basename(log_path).replace("_obstacle_stop_latency.log", "")
    out_path = args.output or os.path.join(LOG_DIR, f"{base}_obstacle_e2e.xlsx")

    summary = summary_stats(df)

    with pd.ExcelWriter(out_path, engine="openpyxl") as writer:
        df.to_excel(writer, sheet_name="Raw", index=False)
        summary.to_excel(writer, sheet_name="Summary", index=False)

    print(f"✅ 저장: {out_path}")
    print(f"   행 수: {len(df)}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
