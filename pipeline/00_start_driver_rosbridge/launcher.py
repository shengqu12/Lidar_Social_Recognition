#!/usr/bin/env python3
"""
Pipeline Launcher
=================
One-script startup for the full LiDAR social recognition pipeline.

Usage:
    # Start everything (rosbridge + LiDAR driver + BG removal on Jetson)
    python3 launcher.py --start

    # Stop everything
    python3 launcher.py --stop

    # Check status of all services
    python3 launcher.py --status

    # Start and then immediately launch clustering node locally
    python3 launcher.py --start --with-clustering

Config (edit the CONFIG block below to match your setup):
    JETSON_IP, JETSON_USER, BG_MODEL_PATH, etc.
"""

import argparse
import subprocess
import sys
import time
import os
from pathlib import Path


# ─── CONFIG — edit these to match your setup ──────────────────────────────────

CONFIG = {
    # Jetson Node 1
    "jetson_ip":   "172.26.42.167",
    "jetson_user": "kelrod",
    "jetson_port": 22,

    # rosbridge websocket port
    "rosbridge_port": 9090,

    # Background model path on Jetson
    "bg_model_jetson": "~/background_statistical.npz",

    # Background removal parameters
    "bg_sigma":  2.0,
    "bg_z_min": -2.8,
    "bg_z_max": -0.5,

    # Topics
    "lidar_topic":      "/livox/lidar",
    "foreground_topic": "/livox/lidar_foreground",

    # Local paths (relative to this file's directory)
    "bg_node_local": "pipeline/01_background_removal/statistical_bg_node.py",

    # Clustering parameters (used with --with-clustering)
    "cluster_tol":   0.4,
    "cluster_min":   8,
    "cluster_max":   800,
    "max_persons":   20,
}

# ─── SSH helper ───────────────────────────────────────────────────────────────

def ssh(cmd: str, background: bool = False, log_file: str = None) -> str:
    """
    Run a command on the Jetson over SSH.

    Args:
        cmd:        Shell command to run on Jetson
        background: If True, run with nohup in background
        log_file:   If background=True, redirect output here (on Jetson)

    Returns:
        stdout string (only meaningful when background=False)
    """
    ip   = CONFIG["jetson_ip"]
    user = CONFIG["jetson_user"]

    if background:
        log = log_file or "/tmp/pipeline_unnamed.log"
        full_cmd = f"nohup bash -c '{cmd}' > {log} 2>&1 &"
    else:
        full_cmd = cmd

    result = subprocess.run(
        ["ssh", f"{user}@{ip}", full_cmd],
        capture_output=True, text=True
    )
    return result.stdout.strip()


def ssh_check(cmd: str) -> bool:
    """Return True if SSH command exits with code 0."""
    ip   = CONFIG["jetson_ip"]
    user = CONFIG["jetson_user"]
    result = subprocess.run(
        ["ssh", f"{user}@{ip}", cmd],
        capture_output=True
    )
    return result.returncode == 0


# ─── Individual service controls ──────────────────────────────────────────────

def start_rosbridge():
    print("  [1/3] Starting rosbridge...")
    cmd = (
        "source /opt/ros/humble/setup.bash && "
        "source ~/ros2_ws/install/setup.bash && "
        "ros2 launch rosbridge_server rosbridge_websocket_launch.xml"
    )
    ssh(cmd, background=True, log_file="/tmp/rosbridge.log")
    time.sleep(2)
    # Verify it's up
    if ssh_check(f"ss -tlnp | grep -q {CONFIG['rosbridge_port']}"):
        print(f"  [1/3] rosbridge OK — port {CONFIG['rosbridge_port']}")
        return True
    else:
        print(f"  [1/3] rosbridge may still be starting — "
              f"check: ssh {CONFIG['jetson_user']}@{CONFIG['jetson_ip']} "
              f"'tail /tmp/rosbridge.log'")
        return False


def start_lidar_driver():
    print("  [2/3] Starting LiDAR driver (livox_ros_driver2)...")
    cmd = (
        "source /opt/ros/humble/setup.bash && "
        "source ~/ros2_ws/install/setup.bash && "
        "ros2 launch livox_ros_driver2 msg_MID360_launch.py"
    )
    ssh(cmd, background=True, log_file="/tmp/lidar_driver.log")
    time.sleep(3)
    print(f"  [2/3] LiDAR driver started — "
          f"check: ssh {CONFIG['jetson_user']}@{CONFIG['jetson_ip']} "
          f"'tail /tmp/lidar_driver.log'")
    return True


def start_bg_removal():
    print("  [3/3] Starting statistical background removal node...")

    # Check model exists on Jetson
    model_path = CONFIG["bg_model_jetson"]
    if not ssh_check(f"test -f {model_path}"):
        print(f"\n  ERROR: Background model not found on Jetson at {model_path}")
        print(f"  Run first:")
        print(f"    python3 pipeline/01_background_removal/statistical_bg_build.py \\")
        print(f"        --bag ./data/rosbags/empty_scene \\")
        print(f"        --output ./models/background_statistical.npz")
        print(f"    scp ./models/background_statistical.npz "
              f"{CONFIG['jetson_user']}@{CONFIG['jetson_ip']}:~/")
        return False

    # Check node file exists on Jetson
    node_path = "~/ros2_ws/src/lidar_filtering/lidar_filtering/statistical_bg_node.py"
    if not ssh_check(f"test -f {node_path}"):
        print(f"  WARNING: BG node not found on Jetson at {node_path}")
        print(f"  Uploading now...")
        local_node = Path(__file__).parent / CONFIG["bg_node_local"]
        subprocess.run([
            "scp", str(local_node),
            f"{CONFIG['jetson_user']}@{CONFIG['jetson_ip']}:{node_path}"
        ])

    cmd = (
        f"source /opt/ros/humble/setup.bash && "
        f"source ~/ros2_ws/install/setup.bash && "
        f"python3 {node_path} "
        f"--model {model_path} "
        f"--sigma {CONFIG['bg_sigma']} "
        f"--z_min {CONFIG['bg_z_min']} "
        f"--z_max {CONFIG['bg_z_max']} "
        f"--input_topic {CONFIG['lidar_topic']} "
        f"--output_topic {CONFIG['foreground_topic']}"
    )
    ssh(cmd, background=True, log_file="/tmp/bg_removal.log")
    time.sleep(2)
    print(f"  [3/3] BG removal node started")
    print(f"         Monitor: ssh {CONFIG['jetson_user']}@{CONFIG['jetson_ip']} "
          f"'tail -f /tmp/bg_removal.log'")
    return True


def stop_all():
    print("Stopping all pipeline services on Jetson...")
    ssh("pkill -f statistical_bg_node; "
        "pkill -f livox_ros_driver2; "
        "pkill -f rosbridge_websocket_launch; "
        "pkill -f rosbridge_server")
    time.sleep(1)
    print("Done.")


def check_status():
    print(f"Checking services on {CONFIG['jetson_ip']}...")
    print()

    # rosbridge port
    if ssh_check(f"ss -tlnp | grep -q {CONFIG['rosbridge_port']}"):
        print(f"  rosbridge       ✓  (port {CONFIG['rosbridge_port']} open)")
    else:
        print(f"  rosbridge       ✗  (port {CONFIG['rosbridge_port']} not found)")

    # livox driver process
    if ssh_check("pgrep -f livox_ros_driver2 > /dev/null"):
        print(f"  LiDAR driver    ✓  (livox_ros_driver2 running)")
    else:
        print(f"  LiDAR driver    ✗  (not running)")

    # bg removal process
    if ssh_check("pgrep -f statistical_bg_node > /dev/null"):
        print(f"  BG removal      ✓  (statistical_bg_node running)")
    else:
        print(f"  BG removal      ✗  (not running)")

    # Check lidar topic is publishing (requires rosbridge to be up)
    print()
    print(f"  Foxglove:  ws://{CONFIG['jetson_ip']}:{CONFIG['rosbridge_port']}")
    print(f"  Topics to add in Foxglove:")
    print(f"    {CONFIG['lidar_topic']:<35} — raw point cloud")
    print(f"    {CONFIG['foreground_topic']:<35} — foreground (after BG removal)")
    print(f"    /detection_boxes              — person bounding boxes (after clustering)")


# ─── Local clustering (runs on Legion) ───────────────────────────────────────

def start_clustering_local():
    """Launch clustering node locally (connects to Jetson via rosbridge)."""
    print()
    print("Starting clustering node locally...")

    script = Path(__file__).parent / "pipeline/02_detection/clustering_node.py"
    if not script.exists():
        print(f"  ERROR: {script} not found")
        return

    cmd = [
        sys.executable, str(script),
        "--jetson_ip",    CONFIG["jetson_ip"],
        "--port",         str(CONFIG["rosbridge_port"]),
        "--topic",        CONFIG["foreground_topic"],
        "--cluster_tol",  str(CONFIG["cluster_tol"]),
        "--min_points",   str(CONFIG["cluster_min"]),
        "--max_points",   str(CONFIG["cluster_max"]),
        "--max_persons",  str(CONFIG["max_persons"]),
    ]
    print(f"  Running: {' '.join(cmd[1:])}")
    print(f"  Press Ctrl+C to stop\n")
    # Replace current process — clustering node runs in foreground
    os.execv(sys.executable, cmd)


# ─── main ─────────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(
        description="LiDAR Social Recognition Pipeline Launcher",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__
    )
    parser.add_argument("--start",  action="store_true",
                        help="Start rosbridge + LiDAR driver + BG removal on Jetson")
    parser.add_argument("--stop",   action="store_true",
                        help="Stop all pipeline services on Jetson")
    parser.add_argument("--status", action="store_true",
                        help="Check status of all services")
    parser.add_argument("--with-clustering", action="store_true",
                        help="After --start, also launch clustering node locally")
    args = parser.parse_args()

    if not any([args.start, args.stop, args.status]):
        parser.print_help()
        return

    if args.stop:
        stop_all()
        return

    if args.status:
        check_status()
        return

    if args.start:
        print("=" * 55)
        print("LiDAR Social Recognition Pipeline — Starting Up")
        print("=" * 55)
        print(f"Target Jetson: {CONFIG['jetson_user']}@{CONFIG['jetson_ip']}")
        print()

        ok1 = start_rosbridge()
        ok2 = start_lidar_driver()
        ok3 = start_bg_removal()

        print()
        print("=" * 55)
        if ok1 and ok2 and ok3:
            print("All services started.")
        else:
            print("Some services may have failed — check logs above.")

        print()
        print("Next steps:")
        print(f"  1. Open Foxglove → ws://{CONFIG['jetson_ip']}:{CONFIG['rosbridge_port']}")
        print(f"  2. Add PointCloud2 panel → topic: {CONFIG['lidar_topic']}")
        print(f"  3. Add PointCloud2 panel → topic: {CONFIG['foreground_topic']}")
        print(f"  4. Run clustering:  python3 launcher.py --with-clustering")
        print()
        print("To stop everything:")
        print("  python3 launcher.py --stop")
        print("=" * 55)

        if args.with_clustering:
            time.sleep(2)
            start_clustering_local()  # this replaces the process (os.execv)


if __name__ == "__main__":
    main()
