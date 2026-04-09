#!/usr/bin/env python3
import argparse
import os
import shutil
import sys
import time
from datetime import datetime
from pathlib import Path
from typing import List, Optional


def _safe_int(value: str, default: int) -> int:
    try:
        return int(value)
    except Exception:
        return default


def _open_log_file(log_dir: Path, prefix: str, ext: str) -> Path:
    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    name = f"{prefix}_{ts}.{ext}"
    path = log_dir / name
    return path


def _update_symlink(link_path: Path, target_path: Path) -> None:
    try:
        tmp = link_path.with_suffix(link_path.suffix + ".tmp")
        if tmp.exists() or tmp.is_symlink():
            tmp.unlink(missing_ok=True)  # type: ignore[arg-type]
        os.symlink(target_path.name, tmp)
        os.replace(tmp, link_path)
    except Exception:
        try:
            if link_path.exists() or link_path.is_symlink():
                link_path.unlink(missing_ok=True)  # type: ignore[arg-type]
            os.symlink(target_path.name, link_path)
        except Exception:
            return


def _prune_old_logs(log_dir: Path, prefix: str, ext: str, keep: int) -> List[str]:
    if keep <= 0:
        return []
    try:
        files = sorted(
            log_dir.glob(f"{prefix}_*.{ext}"),
            key=lambda p: p.stat().st_mtime,
            reverse=True,
        )
    except Exception:
        return []
    removed: List[str] = []
    for p in files[keep:]:
        try:
            p.unlink()
            removed.append(p.stem)
        except Exception:
            pass
    return removed


def _write_bucket_file(bucket_path: Path, bucket_name: str) -> None:
    try:
        bucket_path.parent.mkdir(parents=True, exist_ok=True)
        tmp = bucket_path.with_name(bucket_path.name + ".tmp")
        tmp.write_text(bucket_name + "\n", encoding="utf-8")
        os.replace(tmp, bucket_path)
    except Exception:
        return


def _prune_bucket_dirs(bucket_root: Path, buckets: List[str]) -> None:
    for name in buckets:
        if not name:
            continue
        path = bucket_root / name
        try:
            if path.is_dir():
                shutil.rmtree(path)
        except Exception:
            pass


def run() -> int:
    ap = argparse.ArgumentParser(description="Rotate stdin stream into time-sliced log files.")
    ap.add_argument("--dir", dest="log_dir", required=True, help="Log directory")
    ap.add_argument("--prefix", default="external_receiver", help="Log file prefix")
    ap.add_argument("--ext", default="log", help="Log file extension")
    ap.add_argument("--rotate-seconds", type=int, default=300, help="Rotate interval seconds")
    ap.add_argument("--keep", type=int, default=30, help="Max number of files to keep")
    ap.add_argument("--symlink", default="", help="Symlink path to point at latest file (optional)")
    ap.add_argument("--tee-stdout", type=int, default=1, help="Also write stream to stdout (0/1)")
    ap.add_argument("--bucket-file", default="", help="Write current bucket name to file (optional)")
    ap.add_argument("--bucket-root", default="", help="Root dir for bucket deletion (optional)")
    ap.add_argument("--prune-json", action="store_true", help="Delete bucket dirs for pruned logs (requires --bucket-root)")
    args = ap.parse_args()

    log_dir = Path(args.log_dir)
    log_dir.mkdir(parents=True, exist_ok=True)
    prefix = str(args.prefix)
    ext = str(args.ext).lstrip(".") or "log"
    rotate_seconds = max(1, int(args.rotate_seconds))
    keep = max(0, int(args.keep))
    tee_stdout = bool(int(args.tee_stdout))
    symlink_path: Optional[Path] = Path(args.symlink) if args.symlink else None
    bucket_path: Optional[Path] = Path(args.bucket_file) if args.bucket_file else None
    bucket_root: Optional[Path] = Path(args.bucket_root) if args.bucket_root else None
    prune_json = bool(args.prune_json)

    current_path = _open_log_file(log_dir, prefix, ext)
    f = current_path.open("ab", buffering=1024 * 1024)
    if bucket_path is not None:
        _write_bucket_file(bucket_path, current_path.stem)
    if symlink_path is not None:
        try:
            _update_symlink(symlink_path, current_path)
        except Exception:
            pass
    removed = _prune_old_logs(log_dir, prefix, ext, keep)
    if prune_json and bucket_root is not None and removed:
        _prune_bucket_dirs(bucket_root, removed)

    next_rotate = time.monotonic() + rotate_seconds
    last_flush = time.monotonic()

    stdin_fd = sys.stdin.fileno()
    stdout = sys.stdout.buffer

    try:
        while True:
            try:
                chunk = os.read(stdin_fd, 64 * 1024)
            except InterruptedError:
                continue
            if not chunk:
                break
            f.write(chunk)
            if tee_stdout:
                try:
                    stdout.write(chunk)
                except Exception:
                    tee_stdout = False

            now = time.monotonic()
            if now - last_flush >= 1.0:
                try:
                    f.flush()
                except Exception:
                    pass
                if tee_stdout:
                    try:
                        stdout.flush()
                    except Exception:
                        tee_stdout = False
                last_flush = now

            if now >= next_rotate:
                try:
                    f.flush()
                except Exception:
                    pass
                try:
                    f.close()
                except Exception:
                    pass

                current_path = _open_log_file(log_dir, prefix, ext)
                f = current_path.open("ab", buffering=1024 * 1024)
                if bucket_path is not None:
                    _write_bucket_file(bucket_path, current_path.stem)
                if symlink_path is not None:
                    try:
                        _update_symlink(symlink_path, current_path)
                    except Exception:
                        pass
                removed = _prune_old_logs(log_dir, prefix, ext, keep)
                if prune_json and bucket_root is not None and removed:
                    _prune_bucket_dirs(bucket_root, removed)
                next_rotate = now + rotate_seconds
    finally:
        try:
            f.flush()
        except Exception:
            pass
        try:
            f.close()
        except Exception:
            pass
        if tee_stdout:
            try:
                stdout.flush()
            except Exception:
                pass
    return 0


if __name__ == "__main__":
    raise SystemExit(run())
