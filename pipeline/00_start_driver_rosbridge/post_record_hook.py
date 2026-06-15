#!/usr/bin/env python3
"""
Post-Recording Hook
===================
Automatically called after pipeline --stop or after rebuild_background.py
completes to push new session data to the Synology NAS.

This script is a no-op when ~/.nas_password does not exist, so it never
breaks pipeline runs that are not configured for NAS access.

Usage (called by launcher.py after --stop):
    python3 post_record_hook.py --all-pending --with-csvs --node node1

Usage (called by rebuild_background.py after model download):
    python3 post_record_hook.py --with-model --node node1

NAS safety rule (NEVER REMOVE):
    Never run ls/find/du/wc on NAS directories — they freeze this Synology.
    Only mkdir -p, stat on named files, test on named paths, df /volume1,
    and rsync push (local->NAS) are permitted.
"""

import argparse
import sys
from pathlib import Path

# Import NAS utilities from nas_archive.py in the same directory.
_HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(_HERE))
from nas_archive import _check_prerequisites, archive_bags, archive_csvs, archive_model

# Project root is two levels up from pipeline/00_start_driver_rosbridge/
_PROJECT_ROOT = _HERE.parent.parent


def main():
    parser = argparse.ArgumentParser(
        description="Post-recording NAS archive hook",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    parser.add_argument("--all-pending", action="store_true",
                        help="Archive all rosbag directories to NAS")
    parser.add_argument("--with-csvs", action="store_true",
                        help="Also archive session CSV tracklets")
    parser.add_argument("--with-model", action="store_true",
                        help="Archive the background model for --node")
    parser.add_argument("--node", default="node1",
                        help="Node name in nodes_config.yaml (default: node1)")
    args = parser.parse_args()

    if not any([args.all_pending, args.with_csvs, args.with_model]):
        parser.print_help()
        return

    # Silently skip when NAS is not configured — do not break callers that use
    # check=False but still expect a clean zero exit.
    pw_file = Path.home() / ".nas_password"
    if not pw_file.exists():
        print("[NAS hook] ~/.nas_password not found — skipping NAS archive")
        return

    password = _check_prerequisites()

    if args.all_pending:
        bags_dir = _PROJECT_ROOT / "data" / "rosbags"
        if bags_dir.exists():
            bag_paths = [
                str(p) for p in sorted(bags_dir.iterdir())
                if p.is_dir() and not p.name.startswith(".")
            ]
            if bag_paths:
                print(f"[NAS hook] Archiving {len(bag_paths)} rosbag(s) ...")
                archive_bags(password, bag_paths, args.node)
            else:
                print("[NAS hook] No rosbag directories found to archive")
        else:
            print(f"[NAS hook] Rosbags directory not found: {bags_dir}")

    if args.with_csvs:
        csv_dir = _PROJECT_ROOT / "data" / "tracklets"
        if csv_dir.exists():
            print("[NAS hook] Archiving session CSV tracklets ...")
            archive_csvs(password, csv_dir, args.node)
        else:
            print(f"[NAS hook] CSV directory not found: {csv_dir}")

    if args.with_model:
        model_path = (_PROJECT_ROOT / "models" /
                      f"background_statistical_{args.node}.npz")
        print("[NAS hook] Archiving background model ...")
        archive_model(password, model_path, args.node)

    print("[NAS hook] Done")


if __name__ == "__main__":
    main()
