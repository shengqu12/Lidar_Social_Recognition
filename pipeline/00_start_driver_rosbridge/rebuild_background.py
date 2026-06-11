#!/usr/bin/env python3
"""
Background Model Reconstruction Script
=======================================
Records an empty-scene rosbag on the Jetson, builds a new statistical
background model, downloads it to the local models/ directory, and cleans
up the bag to free disk space.

Usage:
    conda activate livox
    python3 pipeline/00_start_driver_rosbridge/rebuild_background.py

    # Custom options
    python3 pipeline/00_start_driver_rosbridge/rebuild_background.py \
        --node node1 \
        --duration 60 \
        --voxel_size 0.15 \
        --output models/background_statistical_v3.npz

Requirements:
    - Jetson services must be running (LiDAR driver at minimum)
    - The room MUST be empty during recording
    - Run: python3 pipeline/00_start_driver_rosbridge/launcher.py --start first
      (only LiDAR driver is needed, BG removal and rosbridge are optional)

What this script does:
    1. Checks Jetson disk space (warns if < 2GB free)
    2. Records a 60-second empty-scene rosbag on the Jetson
    3. Builds a statistical background model from the bag (on the Jetson,
       using ROS2 Humble — avoids the rosbag2_py version mismatch on Legion)
    4. Downloads the model to local models/ directory
    5. Deletes the rosbag from the Jetson to free disk space
    6. Prints the command to update nodes_config.yaml

Algorithm reference:
    PALMAR (Ul Alam et al. 2021) - voxelized feature representation
    Brscic et al. 2013 - background subtraction for indoor LiDAR
"""

import argparse
import subprocess
import sys
import time
import os
from pathlib import Path
import yaml


# ─── Config loader ────────────────────────────────────────────────────────────

def load_node_config(config_path: str, node_name: str) -> dict:
    with open(config_path) as f:
        cfg = yaml.safe_load(f)
    nodes = cfg.get('nodes', {})
    if node_name not in nodes:
        print(f"ERROR: node '{node_name}' not found in {config_path}")
        print(f"Available nodes: {list(nodes.keys())}")
        sys.exit(1)
    return nodes[node_name]


# ─── SSH helpers ──────────────────────────────────────────────────────────────

def ssh_run(ip: str, user: str, cmd: str,
            capture: bool = True, check: bool = False) -> subprocess.CompletedProcess:
    result = subprocess.run(
        ["ssh", f"{user}@{ip}", cmd],
        capture_output=capture,
        text=True
    )
    if check and result.returncode != 0:
        print(f"ERROR: SSH command failed:\n  {cmd}")
        print(f"  stderr: {result.stderr.strip()}")
        sys.exit(1)
    return result


def ssh_ok(ip: str, user: str, cmd: str) -> bool:
    return ssh_run(ip, user, cmd).returncode == 0


# ─── Steps ────────────────────────────────────────────────────────────────────

def check_disk_space(ip: str, user: str):
    print("Checking Jetson disk space...")
    result = ssh_run(ip, user, "df -h / | tail -1")
    line = result.stdout.strip()
    print(f"  {line}")

    # Parse available space
    parts = line.split()
    if len(parts) >= 4:
        avail = parts[3]
        # Rough check: warn if less than 2G
        if avail.endswith('M') or (avail.endswith('G') and float(avail[:-1]) < 2.0):
            print(f"  WARNING: Only {avail} available on Jetson.")
            print("  Consider cleaning up before recording.")
            answer = input("  Continue anyway? [y/N] ").strip().lower()
            if answer != 'y':
                print("Aborted.")
                sys.exit(0)
    print("  Disk space OK.")


def check_lidar_publishing(ip: str, user: str, topic: str):
    print(f"Checking LiDAR is publishing on {topic}...")
    result = ssh_run(
        ip, user,
        f"source /opt/ros/humble/setup.bash && "
        f"source ~/ros2_ws/install/setup.bash && "
        f"timeout 5 ros2 topic hz {topic} --window 5 2>&1 | head -5"
    )
    if "average rate" in result.stdout:
        print(f"  LiDAR is publishing.")
    else:
        print(f"  WARNING: LiDAR does not appear to be publishing on {topic}.")
        print(f"  Make sure the LiDAR driver is running:")
        print(f"    python3 pipeline/00_start_driver_rosbridge/launcher.py --start")
        answer = input("  Continue anyway? [y/N] ").strip().lower()
        if answer != 'y':
            sys.exit(0)


def record_empty_scene(ip: str, user: str, topic: str,
                       duration: int, bag_name: str = "empty_scene_rebuild"):
    print(f"\nRecording empty scene for {duration} seconds...")
    print(f"  IMPORTANT: Make sure the room is EMPTY (no people in LiDAR view)")
    print(f"  Recording to: ~/{bag_name}/")

    # Check if old bag exists and clean it
    if ssh_ok(ip, user, f"test -d ~/{bag_name}"):
        print(f"  Removing old ~/{bag_name}/ ...")
        ssh_run(ip, user, f"rm -rf ~/{bag_name}", check=True)

    # Start recording in background with max-bag-duration to auto-split files
    cmd = (
        f"source /opt/ros/humble/setup.bash && "
        f"source ~/ros2_ws/install/setup.bash && "
        f"nohup ros2 bag record {topic} -o ~/{bag_name} > /tmp/bag_record.log 2>&1 & "
        f"echo $!"
    )
    result = ssh_run(ip, user, cmd, check=True)
    record_pid = result.stdout.strip()
    print(f"  Recording started (PID {record_pid})")

    # Wait with countdown
    for remaining in range(duration, 0, -10):
        print(f"  {remaining}s remaining...")
        time.sleep(min(10, remaining))

    # Stop recording
    print("  Stopping recording...")
    ssh_run(ip, user, f"kill {record_pid} 2>/dev/null; pkill -f 'ros2 bag record' 2>/dev/null")
    time.sleep(3)  # Allow rosbag2 to flush cache

    # Check what was recorded
    result = ssh_run(ip, user, f"ls -lh ~/{bag_name}/ 2>/dev/null")
    if result.returncode != 0 or not result.stdout.strip():
        print("  ERROR: No bag files found after recording.")
        sys.exit(1)

    print(f"  Recorded files:")
    for line in result.stdout.strip().split('\n'):
        print(f"    {line}")

    # Return the first db3 file path
    result = ssh_run(ip, user, f"ls ~/{bag_name}/*.db3 | head -1")
    first_db3 = result.stdout.strip()
    if not first_db3:
        print("  ERROR: No .db3 files found.")
        sys.exit(1)
    print(f"  Using: {first_db3}")
    return first_db3, bag_name


def build_model_on_jetson(ip: str, user: str, db3_path: str,
                          output_path: str, voxel_size: float,
                          max_frames: int):
    print(f"\nBuilding statistical background model on Jetson...")
    print(f"  voxel_size={voxel_size}, max_frames={max_frames}")

    # Check build script exists on Jetson
    if not ssh_ok(ip, user, "test -f ~/statistical_bg_build.py"):
        print("  Build script not found on Jetson, uploading...")
        local_script = Path(__file__).parent.parent / \
            "01_background_removal" / "statistical_bg_build.py"
        if not local_script.exists():
            print(f"  ERROR: Cannot find {local_script}")
            sys.exit(1)
        subprocess.run([
            "scp", str(local_script),
            f"{user}@{ip}:~/statistical_bg_build.py"
        ], check=True)
        print("  Uploaded.")

    cmd = (
        f"source /opt/ros/humble/setup.bash && "
        f"source ~/ros2_ws/install/setup.bash && "
        f"python3 ~/statistical_bg_build.py "
        f"--bag {db3_path} "
        f"--output {output_path} "
        f"--voxel_size {voxel_size} "
        f"--max_frames {max_frames}"
    )
    print("  Running build (may take 30-60s)...")
    result = subprocess.run(
        ["ssh", f"{user}@{ip}", cmd],
        capture_output=False,  # Show output in real time
        text=True
    )
    if result.returncode != 0:
        print("  ERROR: Model build failed.")
        sys.exit(1)

    # Verify model was created
    if not ssh_ok(ip, user, f"test -f {output_path}"):
        print(f"  ERROR: Output model not found at {output_path}")
        sys.exit(1)

    size_result = ssh_run(ip, user, f"du -sh {output_path}")
    print(f"  Model built: {size_result.stdout.strip()}")


def download_model(ip: str, user: str,
                   remote_path: str, local_path: str):
    print(f"\nDownloading model to {local_path}...")
    Path(local_path).parent.mkdir(parents=True, exist_ok=True)

    result = subprocess.run([
        "scp",
        f"{user}@{ip}:{remote_path}",
        local_path
    ])
    if result.returncode != 0:
        print("  ERROR: Download failed.")
        sys.exit(1)
    size = os.path.getsize(local_path)
    print(f"  Downloaded ({size/1024:.0f} KB)")


def cleanup_bag(ip: str, user: str, bag_name: str):
    print(f"\nCleaning up rosbag (~/{bag_name}/) from Jetson...")
    ssh_run(ip, user, f"rm -rf ~/{bag_name}")

    result = ssh_run(ip, user, "df -h / | tail -1")
    print(f"  Disk after cleanup: {result.stdout.strip()}")


def print_next_steps(local_model_path: str, config_path: str, node_name: str):
    print("\n" + "=" * 60)
    print("Background model rebuilt successfully!")
    print("=" * 60)
    print(f"\nNew model: {local_model_path}")
    print(f"\nUpdate your nodes_config.yaml:")
    print(f"  Open: {config_path}")
    print(f"  Under nodes → {node_name}, change:")
    print(f"    bg_model_path: ~/{Path(local_model_path).name}")
    print(f"\nThen restart the pipeline:")
    print(f"  python3 pipeline/00_start_driver_rosbridge/launcher.py --restart --node {node_name}")
    print(f"\nTo verify improvement, check foreground point count in empty scene:")
    print(f"  ssh kelrod@<jetson_ip> 'tail -f /tmp/bg_removal.log'")
    print(f"  Target: avg_fg < 50 in empty scene")


# ─── main ─────────────────────────────────────────────────────────────────────

def main():
    # Find project root relative to this script
    script_dir = Path(__file__).parent
    project_root = script_dir.parent.parent
    default_config = str(script_dir / "nodes_config.yaml")

    parser = argparse.ArgumentParser(
        description="Rebuild statistical background model for LiDAR pipeline",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__
    )
    parser.add_argument('--node', type=str, default='node1',
                        help='Node name from nodes_config.yaml (default: node1)')
    parser.add_argument('--config', type=str, default=default_config,
                        help='Path to nodes_config.yaml')
    parser.add_argument('--duration', type=int, default=60,
                        help='Recording duration in seconds (default: 60). '
                             'More = better model. 30s minimum.')
    parser.add_argument('--voxel_size', type=float, default=0.15,
                        help='Voxel size for background model in meters '
                             '(default: 0.15). Larger = more robust to noise.')
    parser.add_argument('--max_frames', type=int, default=300,
                        help='Max frames to use for model building (default: 300)')
    parser.add_argument('--output', type=str, default=None,
                        help='Local output path for the model '
                             '(default: models/background_statistical_<node>.npz)')
    parser.add_argument('--bag_name', type=str, default='empty_scene_rebuild',
                        help='Name for the temporary rosbag on Jetson')
    parser.add_argument('--keep_bag', action='store_true',
                        help='Do not delete the rosbag after building '
                             '(warning: uses disk space on Jetson)')
    args = parser.parse_args()

    if args.duration < 30:
        print("ERROR: --duration must be at least 30 seconds.")
        sys.exit(1)

    # Load node config
    node = load_node_config(args.config, args.node)
    ip = node['jetson_ip']
    user = node['jetson_user']
    topic = node.get('lidar_topic', '/livox/lidar')

    # Determine output paths
    remote_model = f"~/background_statistical_{args.node}.npz"
    if args.output:
        local_model = args.output
    else:
        local_model = str(project_root / "models" /
                         f"background_statistical_{args.node}.npz")

    print("=" * 60)
    print(f"Background Model Reconstruction — {args.node}")
    print("=" * 60)
    print(f"  Jetson:      {user}@{ip}")
    print(f"  Topic:       {topic}")
    print(f"  Duration:    {args.duration}s")
    print(f"  Voxel size:  {args.voxel_size}m")
    print(f"  Output:      {local_model}")
    print()

    # Confirm room is empty
    print("IMPORTANT: The room must be completely empty during recording.")
    answer = input("Is the room empty? [y/N] ").strip().lower()
    if answer != 'y':
        print("Please clear the room first, then re-run.")
        sys.exit(0)

    # Run all steps
    check_disk_space(ip, user)
    check_lidar_publishing(ip, user, topic)

    db3_path, bag_name = record_empty_scene(
        ip, user, topic, args.duration, args.bag_name)

    build_model_on_jetson(
        ip, user, db3_path, remote_model,
        args.voxel_size, args.max_frames)

    download_model(ip, user, remote_model, local_model)

    if not args.keep_bag:
        cleanup_bag(ip, user, bag_name)

    print_next_steps(local_model, args.config, args.node)


if __name__ == '__main__':
    main()
