#!/usr/bin/env python3
"""
Pipeline Launcher
=================
Start, stop, restart, and check all LiDAR social recognition services on a
remote Jetson node.  Node-specific settings live in nodes_config.yaml next to
this file; the launcher selects one node with --node (default: node1).

Usage:
    python3 launcher.py --start   [--node node1]
    python3 launcher.py --stop    [--node node1]
    python3 launcher.py --restart [--node node1]
    python3 launcher.py --status  [--node node1]
    python3 launcher.py --start --with-clustering
    python3 launcher.py --start --with-tracking

    # Node 2 (once configured in nodes_config.yaml):
    python3 launcher.py --restart --node node2
"""

import argparse
import os
import subprocess
import sys
import time
from pathlib import Path

import yaml


# ─── Config loader ────────────────────────────────────────────────────────────

def load_node_config(config_path: str, node_name: str) -> dict:
    with open(config_path) as f:
        data = yaml.safe_load(f)
    nodes = data.get("nodes", {})
    if node_name not in nodes:
        available = list(nodes.keys())
        raise ValueError(f"Node '{node_name}' not found in {config_path}. "
                         f"Available: {available}")
    return nodes[node_name]


# ─── SSH helpers ──────────────────────────────────────────────────────────────

def _ssh_args(cfg: dict) -> list:
    return ["ssh", f"{cfg['jetson_user']}@{cfg['jetson_ip']}"]


def ssh_run(cfg: dict, cmd: str, background: bool = False,
            log_file: str = None) -> tuple[str, int]:
    if background:
        log = log_file or "/tmp/pipeline_unnamed.log"
        full_cmd = f"nohup bash -c '{cmd}' > {log} 2>&1 &"
    else:
        full_cmd = cmd
    result = subprocess.run(
        _ssh_args(cfg) + [full_cmd],
        capture_output=True, text=True
    )
    return result.stdout.strip(), result.returncode


def ssh_check(cfg: dict, cmd: str) -> bool:
    result = subprocess.run(
        _ssh_args(cfg) + [cmd],
        capture_output=True
    )
    return result.returncode == 0


# ─── Stop all services ────────────────────────────────────────────────────────

def stop_all(cfg: dict) -> bool:
    """
    Kill every pipeline process and guarantee the rosbridge port is free.
    Tries up to 3 times with direct PID kill if pkill is not enough.
    Returns True on success, False if the port is still occupied after 3 tries.
    """
    ip = cfg["jetson_ip"]
    port = cfg["rosbridge_port"]
    print(f"Stopping all services on {ip}...")

    # Broad pkill sweep — patterns cover both launch wrappers and actual nodes
    kill_cmd = (
        "pkill -f rosbridge_websocket 2>/dev/null || true; "
        "pkill -f rosapi             2>/dev/null || true; "
        "pkill -f livox_ros_driver2  2>/dev/null || true; "
        "pkill -f statistical_bg_node 2>/dev/null || true; "
        "pkill -f msg_MID360_launch  2>/dev/null || true; "
        "pkill -f rosbridge_server   2>/dev/null || true"
    )
    ssh_run(cfg, kill_cmd)
    time.sleep(2)

    # Reset ROS2 daemon to clear stale node/topic cache
    print("  Resetting ROS2 daemon...")
    ssh_run(cfg,
        "source /opt/ros/humble/setup.bash && "
        "ros2 daemon stop 2>/dev/null || true; "
        "sleep 1; "
        "ros2 daemon start 2>/dev/null || true"
    )

    # Verify port is free; kill by PID if not
    for attempt in range(1, 4):
        out, _ = ssh_run(cfg, f"ss -tlnp | grep :{port}")
        if not out.strip():
            print(f"  Port {port} is free.")
            return True

        print(f"  Port {port} still occupied (attempt {attempt}/3)...")
        pid_out, _ = ssh_run(
            cfg,
            f"ss -tlnp | grep :{port} | grep -oP 'pid=\\K[0-9]+' | head -1"
        )
        pid = pid_out.strip()
        if pid:
            print(f"  kill -9 PID {pid}...")
            ssh_run(cfg, f"kill -9 {pid} 2>/dev/null || true")
        time.sleep(1)

    # Final check
    out, _ = ssh_run(cfg, f"ss -tlnp | grep :{port}")
    if out.strip():
        print(f"  ERROR: Port {port} still occupied after 3 attempts.")
        print(f"  Manual fix: ssh {cfg['jetson_user']}@{ip} "
              f"\"kill -9 $(ss -tlnp | grep :{port} | "
              f"grep -oP 'pid=\\\\K[0-9]+')\"")
        return False

    print("  All services stopped.")
    return True


# ─── Individual service starts ────────────────────────────────────────────────

def start_rosbridge(cfg: dict) -> bool:
    print("  [1/3] Starting rosbridge...")
    cmd = (
        "source /opt/ros/humble/setup.bash && "
        "source ~/ros2_ws/install/setup.bash && "
        "ros2 launch rosbridge_server rosbridge_websocket_launch.xml"
    )
    ssh_run(cfg, cmd, background=True, log_file="/tmp/rosbridge.log")

    port = cfg["rosbridge_port"]
    for _ in range(10):
        time.sleep(1)
        if ssh_check(cfg, f"ss -tlnp | grep -q :{port}"):
            print(f"  [1/3] rosbridge OK — port {port} open")
            return True

    print(f"  [1/3] ERROR: rosbridge port {port} did not open within 10 s")
    print(f"         Check: ssh {cfg['jetson_user']}@{cfg['jetson_ip']} "
          f"'tail /tmp/rosbridge.log'")
    return False


def start_lidar_driver(cfg: dict) -> bool:
    print("  [2/3] Starting LiDAR driver (livox_ros_driver2)...")
    cmd = (
        "source /opt/ros/humble/setup.bash && "
        "source ~/ros2_ws/install/setup.bash && "
        "ros2 launch livox_ros_driver2 msg_MID360_launch.py"
    )
    ssh_run(cfg, cmd, background=True, log_file="/tmp/lidar_driver.log")

    topic = cfg["lidar_topic"]
    check_cmd = (
        "source /opt/ros/humble/setup.bash && "
        "source ~/ros2_ws/install/setup.bash && "
        f"ros2 topic list 2>/dev/null | grep -q '{topic}'"
    )
    for _ in range(15):
        time.sleep(1)
        if ssh_check(cfg, check_cmd):
            print(f"  [2/3] LiDAR driver OK — {topic} visible")
            return True

    # Non-fatal: driver can take longer on cold start
    print(f"  [2/3] WARNING: {topic} not visible after 15 s "
          f"(driver may still be starting)")
    print(f"         Check: ssh {cfg['jetson_user']}@{cfg['jetson_ip']} "
          f"'tail /tmp/lidar_driver.log'")
    return True


def start_bg_removal(cfg: dict) -> bool:
    print("  [3/3] Starting statistical background removal...")

    model = cfg["bg_model_path"]
    if not ssh_check(cfg, f"test -f {model}"):
        print(f"\n  ERROR: Background model not found at {model} on Jetson")
        print(f"  Build and upload it first:")
        print(f"    python3 pipeline/01_background_removal/statistical_bg_build.py "
              f"--bag ./data/rosbags/empty_scene "
              f"--output ./models/background_statistical.npz")
        print(f"    scp ./models/background_statistical.npz "
              f"{cfg['jetson_user']}@{cfg['jetson_ip']}:~/")
        return False

    node_path = ("~/ros2_ws/src/lidar_filtering/lidar_filtering/"
                 "statistical_bg_node.py")
    cmd = (
        f"source /opt/ros/humble/setup.bash && "
        f"source ~/ros2_ws/install/setup.bash && "
        f"python3 {node_path} "
        f"--model {model} "
        f"--sigma {cfg['bg_sigma']} "
        f"--z_min {cfg['bg_z_min']} "
        f"--z_max {cfg['bg_z_max']} "
        f"--input_topic {cfg['lidar_topic']} "
        f"--output_topic {cfg['foreground_topic']}"
    )
    ssh_run(cfg, cmd, background=True, log_file="/tmp/bg_removal.log")

    topic = cfg["foreground_topic"]
    check_cmd = (
        "source /opt/ros/humble/setup.bash && "
        "source ~/ros2_ws/install/setup.bash && "
        f"ros2 topic list 2>/dev/null | grep -q '{topic}'"
    )
    for _ in range(15):
        time.sleep(1)
        if ssh_check(cfg, check_cmd):
            print(f"  [3/3] BG removal OK — {topic} visible")
            print(f"         Monitor: ssh {cfg['jetson_user']}@{cfg['jetson_ip']} "
                  f"'tail -f /tmp/bg_removal.log'")
            return True

    print(f"  [3/3] WARNING: {topic} not visible after 15 s")
    print(f"         Check: ssh {cfg['jetson_user']}@{cfg['jetson_ip']} "
          f"'tail -f /tmp/bg_removal.log'")
    return True


def check_status(cfg: dict) -> None:
    ip = cfg["jetson_ip"]
    port = cfg["rosbridge_port"]
    print(f"Status on {cfg['jetson_user']}@{ip}:")
    print()

    if ssh_check(cfg, f"ss -tlnp | grep -q :{port}"):
        print(f"  rosbridge       OK  (port {port} open)")
    else:
        print(f"  rosbridge       FAIL  (port {port} not listening)")

    if ssh_check(cfg, "pgrep -f livox_ros_driver2 > /dev/null"):
        print(f"  LiDAR driver    OK  (livox_ros_driver2 running)")
    else:
        print(f"  LiDAR driver    FAIL")

    if ssh_check(cfg, "pgrep -f statistical_bg_node > /dev/null"):
        print(f"  BG removal      OK  (statistical_bg_node running)")
    else:
        print(f"  BG removal      FAIL")

    print()
    print(f"  Foxglove:  ws://{ip}:{port}")
    print(f"  Topics:")
    print(f"    {cfg['lidar_topic']:<42} raw point cloud")
    print(f"    {cfg['foreground_topic']:<42} foreground (BG removed)")
    print(f"    /detection_boxes{'':<26} person bounding boxes")


# ─── Local node launchers ─────────────────────────────────────────────────────

def launch_clustering_local(cfg: dict, config_path: str, node_name: str) -> None:
    script = Path(__file__).parent.parent / "02_detection" / "clustering_node.py"
    if not script.exists():
        print(f"  ERROR: {script} not found")
        return

    cmd = [
        sys.executable, str(script),
        "--config", config_path,
        "--node",   node_name,
    ]
    print(f"  Running: {' '.join(cmd[1:])}")
    print("  Press Ctrl+C to stop\n")
    os.execv(sys.executable, cmd)


def launch_tracking_local(cfg: dict, config_path: str, node_name: str) -> None:
    """Launch tracking node (includes detection; replaces clustering-only node)."""
    script = Path(__file__).parent.parent / "03_tracking" / "tracking_node.py"
    if not script.exists():
        print(f"  ERROR: {script} not found")
        return

    cmd = [
        sys.executable, str(script),
        "--config", config_path,
        "--node",   node_name,
    ]
    print(f"  Running: {' '.join(cmd[1:])}")
    print("  Publishes: /tracked_boxes  /tracked_centers")
    print("  Logs CSV:  data/tracklets/session_<time>.csv")
    print("  Press Ctrl+C to stop\n")
    os.execv(sys.executable, cmd)


# ─── main ─────────────────────────────────────────────────────────────────────

def main() -> None:
    default_config = str(Path(__file__).parent / "nodes_config.yaml")

    parser = argparse.ArgumentParser(
        description="LiDAR Social Recognition Pipeline Launcher",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    parser.add_argument("--start",   action="store_true",
                        help="Start rosbridge + LiDAR driver + BG removal (always stops first)")
    parser.add_argument("--stop",    action="store_true",
                        help="Stop all pipeline services on Jetson")
    parser.add_argument("--restart", action="store_true",
                        help="Stop then start (equivalent to --stop + --start)")
    parser.add_argument("--status",  action="store_true",
                        help="Check status of all services")
    parser.add_argument("--with-clustering", action="store_true",
                        help="After --start/--restart, also launch clustering node locally")
    parser.add_argument("--with-tracking", action="store_true",
                        help="After --start/--restart, launch tracking node (detection + tracking + CSV)")
    parser.add_argument("--node",   default="node1",
                        help="Node name in nodes_config.yaml (default: node1)")
    parser.add_argument("--config", default=default_config,
                        help=f"Path to nodes_config.yaml (default: {default_config})")
    args = parser.parse_args()

    if not any([args.start, args.stop, args.restart, args.status]):
        parser.print_help()
        return

    try:
        cfg = load_node_config(args.config, args.node)
    except (FileNotFoundError, ValueError) as e:
        print(f"Config error: {e}")
        sys.exit(1)

    if args.status:
        check_status(cfg)
        return

    # --stop, --start, and --restart all begin with a full cleanup
    if args.stop or args.start or args.restart:
        ok = stop_all(cfg)
        if not ok:
            print("Stop sequence failed — cannot guarantee clean state. Aborting.")
            sys.exit(1)

    if args.stop:
        return

    # Start sequence
    print()
    print("=" * 58)
    print("LiDAR Social Recognition Pipeline — Starting Up")
    print("=" * 58)
    print(f"Node:   {args.node}")
    print(f"Jetson: {cfg['jetson_user']}@{cfg['jetson_ip']}")
    print()

    ok1 = start_rosbridge(cfg)
    if not ok1:
        print("\nrosbridge failed to start — aborting (nothing else would work).")
        sys.exit(1)

    ok2 = start_lidar_driver(cfg)
    ok3 = start_bg_removal(cfg)

    print()
    print("=" * 58)
    if ok1 and ok2 and ok3:
        print("All services started successfully.")
    else:
        print("Started with warnings — check logs above.")
    print()
    print(f"  Open Foxglove → ws://{cfg['jetson_ip']}:{cfg['rosbridge_port']}")
    print(f"  Run clustering:  python3 launcher.py --with-clustering --node {args.node}")
    print(f"  Run tracking:    python3 launcher.py --with-tracking  --node {args.node}")
    print("=" * 58)

    if args.with_tracking:
        time.sleep(2)
        launch_tracking_local(cfg, args.config, args.node)
    elif args.with_clustering:
        time.sleep(2)
        launch_clustering_local(cfg, args.config, args.node)


if __name__ == "__main__":
    main()
