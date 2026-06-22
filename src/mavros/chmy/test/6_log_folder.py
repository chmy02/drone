#!/usr/bin/env python3
"""
logs/ 루트에만 있는 모든 파일을 — 하위 폴더 안은 제외하고 — 실행할 때마다 한 폴더로 한꺼번에 옮깁습니다.
파일명·확장자·타임스탬프 초 단위 차이 무관하게, “폴더 밖 플랫 파일”만 수집합니다.

폴더 이름: 이 스크립트 실행 시각 (연도 YY 2자리 + 월일_시분초 → 예: 260508_143022), 매 실행마다 새 이름.

예:
  python3 6_log_folder.py
  python3 6_log_folder.py --dry-run
"""

from __future__ import annotations

import argparse
import shutil
from datetime import datetime
from pathlib import Path
from typing import List, Optional, Sequence

LOG_DIR_DEFAULT = "/home/rtcl-chmy/mavros_ws/src/mavros/chmy/logs"


def collect_root_files_only(log_dir: Path) -> List[Path]:
    """log_dir 직속 자식만: 파일만(디렉터리 제외)."""
    out: List[Path] = []
    if not log_dir.is_dir():
        return out
    for path in sorted(log_dir.iterdir()):
        if path.is_file():
            out.append(path)
    return out


def move_into(
    files: Sequence[Path],
    dest_dir: Path,
    dry_run: bool,
    force: bool,
) -> tuple[int, int]:
    done, skipped = 0, 0
    if not dry_run:
        dest_dir.mkdir(parents=True, exist_ok=True)
    else:
        print(f"[dry-run] mkdir -p {dest_dir}")

    for src in sorted(files, key=lambda p: p.name):
        dst = dest_dir / src.name
        if dst.resolve() == src.resolve():
            skipped += 1
            continue
        if dst.exists() and not force:
            print(f"⚠ 존재 — 스킵: {dst.name}")
            skipped += 1
            continue
        if dst.exists() and force:
            bk = dst.with_suffix(dst.suffix + ".bak")
            if not dry_run:
                bk.unlink(missing_ok=True)
                shutil.move(str(dst), str(bk))
            print(f"… 백업: {bk.name}")
        if dry_run:
            print(f"  [dry-run] {src.name}")
        else:
            shutil.move(str(src), str(dst))
            print(f"  ✓ {src.name}")
        done += 1
    return done, skipped


def main(argv: Optional[Sequence[str]] = None) -> int:
    ap = argparse.ArgumentParser(
        description="logs 루트의 모든 플랫 파일을 실행 시각 폴더로 이동"
    )
    ap.add_argument(
        "--log-dir",
        type=Path,
        default=Path(LOG_DIR_DEFAULT),
        help=f"스캔 디렉터리 (기본: {LOG_DIR_DEFAULT})",
    )
    ap.add_argument(
        "--dry-run",
        action="store_true",
        help="이동하지 않고 대상만 출력",
    )
    ap.add_argument(
        "--force",
        action="store_true",
        help="목적지에 같은 이름 있으면 .bak 백업 후 덮음",
    )
    ap.add_argument(
        "--folder",
        type=str,
        default=None,
        metavar="NAME",
        help="실행 시각 대신 하위 폴더 이름(로그 디렉터리 바로 아래만)",
    )
    args = ap.parse_args(list(argv) if argv is not None else None)

    log_dir = args.log_dir.resolve()
    roots = collect_root_files_only(log_dir)

    if not roots:
        print(f"이동할 루트 파일 없음 (하위 폴더 미포함): {log_dir}")
        return 1

    if args.folder:
        folder_name = args.folder.strip()
        if not folder_name or "/" in folder_name or ".." in folder_name:
            print("잘못된 --folder")
            return 1
    else:
        folder_name = datetime.now().strftime("%y%m%d_%H%M%S")

    dest_dir = (log_dir / folder_name).resolve()
    try:
        dest_dir.relative_to(log_dir.resolve())
    except ValueError:
        print("--folder 가 log-dir 바깥으로 나감")
        return 1

    print(f"루트 플랫 파일 {len(roots)}개 → {dest_dir.relative_to(log_dir)}")
    dd, ds = move_into(roots, dest_dir, args.dry_run, args.force)
    if args.dry_run:
        print(f"[dry-run] 이동 예상 {dd}, 스킵 {ds}")
    else:
        print(f"완료. 이동 {dd}, 스킵 {ds}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
