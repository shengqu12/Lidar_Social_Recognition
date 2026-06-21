#!/usr/bin/env python3
"""
NAS Archiver
============
Archives recorded bags, CSV tracklets, and background models to the
Synology NAS at 172.24.72.224 via rsync over SSH (password auth via sshpass).

Usage:
    python3 nas_archive.py --check-space
    python3 nas_archive.py --archive-bags data/rosbags/session_X --node node1
    python3 nas_archive.py --archive-csvs data/tracklets/ --node node1
    python3 nas_archive.py --archive-model models/background_statistical_node1.npz --node node1

NAS safety rule (NEVER REMOVE):
    This Synology NAS freezes when ls/find/du/wc runs on large directories.
    Permitted remote operations:
      - stat on a NAMED file path
      - test -f / test -d on a NAMED path
      - mkdir -p <path>
      - df /volume1  (volume mount point only, never a subdirectory)
      - rsync PUSH (local->NAS only; never use --list-only or read NAS as source)
"""

import argparse
import shutil
import subprocess
import sys
from dataclasses import dataclass, field
from pathlib import Path

# ─── NAS connection constants ──────────────────────────────────────────────────

NAS_HOST = "172.24.72.224"
NAS_PORT = 5999
NAS_USER = "shengq"
NAS_VOLUME = "/volume1"          # Synology primary volume mount point
NAS_ARCHIVE_ROOT = "/volume1/lidar_archive"


# ─── Startup checks ───────────────────────────────────────────────────────────

def _check_prerequisites() -> str:
    """Return the NAS password string or exit with clear instructions."""
    if not shutil.which("sshpass"):
        print("ERROR: sshpass is not installed.")
        print("Install it with:  sudo apt install sshpass -y")
        sys.exit(1)

    pw_file = Path.home() / ".nas_password"
    if not pw_file.exists():
        print("ERROR: ~/.nas_password not found.")
        print("Create it with:")
        print("  echo 'YOUR_NAS_PASSWORD' > ~/.nas_password")
        print("  chmod 600 ~/.nas_password")
        sys.exit(1)

    return pw_file.read_text().strip()


# ─── SSH / rsync helpers ───────────────────────────────────────────────────────

def _ssh_run(password: str, remote_cmd: str,
             capture: bool = True) -> subprocess.CompletedProcess:
    """Run a single command on the NAS via SSH + sshpass."""
    cmd = [
        "sshpass", "-p", password,
        "ssh", "-p", str(NAS_PORT), "-o", "StrictHostKeyChecking=no",
        f"{NAS_USER}@{NAS_HOST}",
        remote_cmd,
    ]
    return subprocess.run(cmd, capture_output=capture, text=True)


def _rsync_push(password: str,
                local_src: str,
                remote_dst: str,
                extra_flags: list = None) -> int:
    """
    Push local_src to NAS:remote_dst via rsync over sshpass-authenticated SSH.
    The -e flag embeds sshpass so rsync can authenticate without a prompt.
    Never pass --list-only or set NAS as the source directory.
    """
    ssh_e_flag = (
        f"sshpass -p {password} ssh -p {NAS_PORT} -o StrictHostKeyChecking=no"
    )
    cmd = [
        "rsync", "-avz", "--partial",
        "-e", ssh_e_flag,
        local_src,
        f"{NAS_USER}@{NAS_HOST}:{remote_dst}",
    ]
    if extra_flags:
        cmd[1:1] = extra_flags
    result = subprocess.run(cmd)
    return result.returncode


# ─── Space check ──────────────────────────────────────────────────────────────

def check_space(password: str) -> None:
    """
    Print free space on the NAS primary volume.
    Uses 'df /volume1' — safe per NAS safety rule (volume mount, not a subdir).
    """
    print(f"Checking NAS free space on {NAS_HOST}:{NAS_PORT} ...")
    result = _ssh_run(password, f"df -h {NAS_VOLUME} | tail -1")
    if result.returncode != 0:
        print(f"ERROR: Could not reach NAS.\n  {result.stderr.strip()}")
        sys.exit(1)
    line = result.stdout.strip()
    parts = line.split()
    if len(parts) >= 4:
        size, used, avail = parts[1], parts[2], parts[3]
        print(f"  NAS volume {NAS_VOLUME}:")
        print(f"    Total:     {size}")
        print(f"    Used:      {used}")
        print(f"    Available: {avail}")
    else:
        print(f"  {line}")
    print("NAS free space check OK")


# ─── Ensure remote directory exists ───────────────────────────────────────────

def _ensure_remote_dir(password: str, remote_path: str) -> None:
    """mkdir -p on the NAS. Safe per NAS safety rule (no directory listing)."""
    result = _ssh_run(password, f"mkdir -p {remote_path}")
    if result.returncode != 0:
        print(f"  WARNING: mkdir -p {remote_path} failed: {result.stderr.strip()}")


# ─── Archive operations ───────────────────────────────────────────────────────

def archive_bags(password: str, local_paths: list, node: str) -> None:
    """Push rosbag files/directories to NAS under lidar_archive/<node>/bags/."""
    remote_dst = f"{NAS_ARCHIVE_ROOT}/{node}/bags/"
    _ensure_remote_dir(password, remote_dst)
    for local in local_paths:
        p = Path(local)
        if not p.exists():
            print(f"  WARNING: {local} does not exist, skipping")
            continue
        print(f"  Archiving bag: {p.name} -> NAS:{remote_dst}")
        rc = _rsync_push(password, str(p), remote_dst)
        if rc != 0:
            print(f"  ERROR: rsync failed for {p.name} (exit {rc})")
        else:
            print(f"  OK: {p.name}")


def archive_csvs(password: str, csv_dir: Path, node: str) -> None:
    """Push all session_*.csv files to NAS under lidar_archive/<node>/csvs/."""
    remote_dst = f"{NAS_ARCHIVE_ROOT}/{node}/csvs/"
    _ensure_remote_dir(password, remote_dst)
    csvs = sorted(csv_dir.glob("session_*.csv"))
    if not csvs:
        print(f"  No session CSVs found in {csv_dir}")
        return
    print(f"  Archiving {len(csvs)} session CSV(s) -> NAS:{remote_dst}")
    for csv in csvs:
        rc = _rsync_push(password, str(csv), remote_dst)
        if rc != 0:
            print(f"  ERROR: rsync failed for {csv.name}")
        else:
            print(f"  OK: {csv.name}")


def archive_model(password: str, model_path: Path, node: str) -> None:
    """Push a background model .npz to NAS under lidar_archive/<node>/models/."""
    if not model_path.exists():
        print(f"  WARNING: Model not found at {model_path}, skipping")
        return
    remote_dst = f"{NAS_ARCHIVE_ROOT}/{node}/models/"
    _ensure_remote_dir(password, remote_dst)
    print(f"  Archiving model: {model_path.name} -> NAS:{remote_dst}")
    rc = _rsync_push(password, str(model_path), remote_dst)
    if rc != 0:
        print(f"  ERROR: rsync failed for {model_path.name} (exit {rc})")
    else:
        print(f"  OK: {model_path.name}")


# ─── Full idempotent sync ─────────────────────────────────────────────────────

def _human_bytes(n: int) -> str:
    for unit in ("B", "KB", "MB", "GB"):
        if n < 1024:
            return f"{n:.1f} {unit}"
        n /= 1024
    return f"{n:.1f} TB"


@dataclass
class _SyncStats:
    local: int = 0
    already_on_nas: int = 0
    uploaded: int = 0
    failed: int = 0
    bytes_freed: int = 0


def sync_all(password: str, project_root: Path, node: str) -> None:
    """
    Idempotent catch-all sync — uploads every local item not yet on NAS.
    Rosbags: purge local after verified upload (they're big).
    CSVs and models: copy only, keep local (tiny).
    NAS safety rule: one `test` call per item, never ls/find/du on NAS dirs.
    """
    all_stats: dict[str, _SyncStats] = {}

    # ── rosbags ───────────────────────────────────────────────────────────────
    bags_stats = _SyncStats()
    bags_dir = project_root / "data" / "rosbags"
    remote_bags = f"{NAS_ARCHIVE_ROOT}/{node}/bags/"

    if bags_dir.exists():
        bag_dirs = [p for p in sorted(bags_dir.iterdir())
                    if p.is_dir() and not p.name.startswith(".")]
        bags_stats.local = len(bag_dirs)
        if bag_dirs:
            _ensure_remote_dir(password, remote_bags)
        for bag in bag_dirs:
            chk = _ssh_run(password, f"test -d {remote_bags}{bag.name}")
            if chk.returncode == 0:
                bags_stats.already_on_nas += 1
                continue
            bag_size = sum(f.stat().st_size for f in bag.rglob("*") if f.is_file())
            print(f"  [bag] uploading {bag.name} ...")
            rc = _rsync_push(password, str(bag), remote_bags)
            if rc != 0:
                bags_stats.failed += 1
                print(f"  ERROR: rsync failed for bag {bag.name} (exit {rc})")
            else:
                bags_stats.uploaded += 1
                bags_stats.bytes_freed += bag_size
                shutil.rmtree(bag)
                print(f"  OK (freed {_human_bytes(bag_size)}): {bag.name}")

    all_stats["rosbags"] = bags_stats

    # ── tracklet CSVs ─────────────────────────────────────────────────────────
    tracklets_stats = _SyncStats()
    tracklets_dir = project_root / "data" / "tracklets"
    remote_csvs = f"{NAS_ARCHIVE_ROOT}/{node}/csvs/"

    if tracklets_dir.exists():
        csvs = sorted(tracklets_dir.glob("session_*.csv"))
        tracklets_stats.local = len(csvs)
        if csvs:
            _ensure_remote_dir(password, remote_csvs)
        for csv in csvs:
            chk = _ssh_run(password, f"test -f {remote_csvs}{csv.name}")
            if chk.returncode == 0:
                tracklets_stats.already_on_nas += 1
                continue
            rc = _rsync_push(password, str(csv), remote_csvs)
            if rc != 0:
                tracklets_stats.failed += 1
                print(f"  ERROR: rsync failed for {csv.name}")
            else:
                tracklets_stats.uploaded += 1
                print(f"  OK: {csv.name}")

    all_stats["tracklets"] = tracklets_stats

    # ── encounters CSVs ───────────────────────────────────────────────────────
    encounters_stats = _SyncStats()
    encounters_dir = project_root / "data" / "encounters"
    remote_encounters = f"{NAS_ARCHIVE_ROOT}/{node}/encounters/"

    if encounters_dir.exists():
        enc_csvs = sorted(encounters_dir.glob("*.csv"))
        encounters_stats.local = len(enc_csvs)
        if enc_csvs:
            _ensure_remote_dir(password, remote_encounters)
        for csv in enc_csvs:
            # Encounters files grow over time — always rsync (fast delta, small file)
            rc = _rsync_push(password, str(csv), remote_encounters)
            if rc != 0:
                encounters_stats.failed += 1
                print(f"  ERROR: rsync failed for {csv.name}")
            else:
                encounters_stats.uploaded += 1

    all_stats["encounters"] = encounters_stats

    # ── background models ─────────────────────────────────────────────────────
    models_stats = _SyncStats()
    models_dir = project_root / "models"
    remote_models = f"{NAS_ARCHIVE_ROOT}/{node}/models/"

    if models_dir.exists():
        model_files = sorted(models_dir.glob("background_statistical*.npz"))
        models_stats.local = len(model_files)
        if model_files:
            _ensure_remote_dir(password, remote_models)
        for model in model_files:
            chk = _ssh_run(password, f"test -f {remote_models}{model.name}")
            if chk.returncode == 0:
                models_stats.already_on_nas += 1
                continue
            rc = _rsync_push(password, str(model), remote_models)
            if rc != 0:
                models_stats.failed += 1
                print(f"  ERROR: rsync failed for {model.name}")
            else:
                models_stats.uploaded += 1
                print(f"  OK: {model.name}")

    all_stats["models"] = models_stats

    # ── summary table ─────────────────────────────────────────────────────────
    total_freed = sum(s.bytes_freed for s in all_stats.values())
    print()
    print("  ┌──────────────┬───────┬──────────┬──────────┬────────┬────────────┐")
    print("  │ Kind         │ Local │ On NAS   │ Uploaded │ Failed │ Freed      │")
    print("  ├──────────────┼───────┼──────────┼──────────┼────────┼────────────┤")
    for kind, s in all_stats.items():
        freed = _human_bytes(s.bytes_freed) if s.bytes_freed else "—"
        print(f"  │ {kind:<12} │ {s.local:>5} │ {s.already_on_nas:>8} │"
              f" {s.uploaded:>8} │ {s.failed:>6} │ {freed:>10} │")
    print("  └──────────────┴───────┴──────────┴──────────┴────────┴────────────┘")
    if total_freed:
        print(f"  Total freed: {_human_bytes(total_freed)}")


# ─── main ─────────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(
        description="NAS Archiver for LiDAR social recognition pipeline",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    parser.add_argument("--check-space", action="store_true",
                        help="Check free space on the NAS volume (safe: uses df /volume1)")
    parser.add_argument("--archive-bags", nargs="+", metavar="PATH",
                        help="Rosbag files or directories to push to NAS")
    parser.add_argument("--archive-csvs", metavar="DIR",
                        help="Directory of session CSVs to push to NAS")
    parser.add_argument("--archive-model", metavar="FILE",
                        help="Background model .npz to push to NAS")
    parser.add_argument("--node", default="node1",
                        help="Node name used as NAS subdirectory (default: node1)")
    args = parser.parse_args()

    if not any([args.check_space, args.archive_bags, args.archive_csvs,
                args.archive_model]):
        parser.print_help()
        return

    password = _check_prerequisites()

    if args.check_space:
        check_space(password)

    if args.archive_bags:
        archive_bags(password, args.archive_bags, args.node)

    if args.archive_csvs:
        archive_csvs(password, Path(args.archive_csvs), args.node)

    if args.archive_model:
        archive_model(password, Path(args.archive_model), args.node)


if __name__ == "__main__":
    main()
