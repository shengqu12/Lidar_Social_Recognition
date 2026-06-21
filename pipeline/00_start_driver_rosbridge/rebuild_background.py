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
    2. Verifies LiDAR is actively publishing (aborts if not, unless --force)
    3. Records a 60-second empty-scene rosbag on the Jetson
    4. Validates the bag file is large enough to contain real data (aborts if not)
    5. Builds a statistical background model to a TEMPORARY file on the Jetson
       (using ROS2 Humble — avoids the rosbag2_py version mismatch on Legion)
    6. Validates voxel count >= 100 before overwriting the existing model
    7. Atomically renames the temporary file over the existing model
    8. Downloads the model to local models/ directory
    9. Deletes the rosbag from the Jetson to free disk space
   10. Prints the command to update nodes_config.yaml

Algorithm reference:
    PALMAR (Ul Alam et al. 2021) - voxelized feature representation
    Brscic et al. 2013 - background subtraction for indoor LiDAR
"""

import argparse
import os
import re
import subprocess
import sys
import threading
import time
from pathlib import Path
import yaml

# Minimum .db3 size (bytes) for a real recording.
# A 60s LiDAR bag is hundreds of MB; 24KB = empty bag (driver not publishing).
BAG_MIN_BYTES = 1_000_000  # 1 MB

# Minimum background voxels to consider a build valid.
VOXEL_MIN_COUNT = 100


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

def check_disk_space(ip: str, user: str, interactive: bool = True):
    print("Checking Jetson disk space...")
    result = ssh_run(ip, user, "df -h / | tail -1")
    line = result.stdout.strip()
    print(f"  {line}")

    parts = line.split()
    if len(parts) >= 4:
        avail = parts[3]
        if avail.endswith('M') or (avail.endswith('G') and float(avail[:-1]) < 2.0):
            print(f"  WARNING: Only {avail} available on Jetson.")
            print("  Consider cleaning up before recording.")
            if not interactive:
                print("  (Non-interactive: proceeding despite low disk space.)")
            else:
                answer = input("  Continue anyway? [y/N] ").strip().lower()
                if answer != 'y':
                    print("Aborted.")
                    sys.exit(0)
    print("  Disk space OK.")


def _print_lidar_guidance(user: str, ip: str, node_name: str, topic: str):
    print(f"  Make sure services are running BEFORE rebuilding:")
    print(f"    python3 pipeline/00_start_driver_rosbridge/launcher.py "
          f"--start --node {node_name}")
    print(f"  Then verify the LiDAR is publishing:")
    print(f"    ssh {user}@{ip} \"ros2 topic hz {topic}\"")


def check_lidar_publishing(ip: str, user: str, topic: str,
                           node_name: str = 'node1', force: bool = False):
    print(f"Checking LiDAR is publishing on {topic}...")
    result = ssh_run(
        ip, user,
        f"source /opt/ros/humble/setup.bash && "
        f"source ~/ros2_ws/install/setup.bash && "
        f"timeout 5 ros2 topic hz {topic} --window 5 2>&1 | head -5"
    )
    if "average rate" in result.stdout:
        for line in result.stdout.splitlines():
            if "average rate" in line:
                print(f"  LiDAR is publishing: {line.strip()}")
                break
        return

    print(f"\nERROR: LiDAR does not appear to be publishing on {topic}.")
    _print_lidar_guidance(user, ip, node_name, topic)
    if force:
        print("  --force passed, continuing anyway.")
    else:
        sys.exit(1)


def record_empty_scene(ip: str, user: str, topic: str,
                       duration: int, bag_name: str = "empty_scene_rebuild",
                       abort=None):
    print(f"\nRecording empty scene for {duration} seconds...")
    print(f"  IMPORTANT: Make sure the room is EMPTY (no people in LiDAR view)")
    print(f"  Recording to: ~/{bag_name}/")

    # Check if old bag exists and clean it
    if ssh_ok(ip, user, f"test -d ~/{bag_name}"):
        print(f"  Removing old ~/{bag_name}/ ...")
        ssh_run(ip, user, f"rm -rf ~/{bag_name}", check=True)

    # Use 'timeout' to auto-stop recording after duration seconds.
    # This is reliable: timeout kills the child process directly,
    # no need to track PID across SSH connections.
    cmd = (
        f"source /opt/ros/humble/setup.bash && "
        f"source ~/ros2_ws/install/setup.bash && "
        f"nohup timeout {duration} "
        f"ros2 bag record {topic} -o ~/{bag_name} "
        f"> /tmp/bag_record.log 2>&1 & echo STARTED"
    )
    result = ssh_run(ip, user, cmd, check=True)
    if "STARTED" not in result.stdout:
        print("  ERROR: Failed to start recording.")
        sys.exit(1)
    print(f"  Recording started (auto-stops after {duration}s)")

    # Real-time countdown printed to console
    print(f"  Progress: ", end="", flush=True)
    for elapsed in range(duration):
        # Check abort each second so a peer failure stops this countdown promptly.
        if abort is not None and abort.is_set():
            print("\n  Abort signal received — exiting record countdown.")
            sys.exit(1)
        time.sleep(1)
        if (elapsed + 1) % 10 == 0:
            remaining = duration - elapsed - 1
            print(f"{elapsed+1}s", end="", flush=True)
            if remaining > 0:
                print("...", end="", flush=True)
    print(" done")

    # Wait for rosbag2 to flush write cache
    print("  Flushing rosbag2 cache...")
    time.sleep(4)

    # Belt-and-suspenders: kill any stray recording process
    ssh_run(ip, user, "pkill -9 -f 'ros2 bag record' 2>/dev/null; true")

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


def validate_bag_size(ip: str, user: str, db3_path: str,
                      node_name: str, topic: str):
    """Abort if the .db3 is too small to contain real LiDAR data."""
    print("Validating bag file size...")
    result = ssh_run(ip, user, f"stat -c %s {db3_path}")
    try:
        size_bytes = int(result.stdout.strip())
    except ValueError:
        print("  WARNING: Could not determine bag size, proceeding.")
        return

    size_kb = size_bytes / 1024
    size_mb = size_bytes / (1024 * 1024)
    if size_bytes < BAG_MIN_BYTES:
        print(f"\nERROR: Recorded bag is only {size_kb:.0f}KB — "
              f"the LiDAR driver was likely not publishing.")
        _print_lidar_guidance(user, ip, node_name, topic)
        print("Do NOT proceed to build or overwrite the existing model.")
        sys.exit(1)
    print(f"  Bag size OK: {size_mb:.1f} MB")


def build_model_on_jetson(ip: str, user: str, db3_path: str,
                          output_path: str, voxel_size: float,
                          max_frames: int, abort=None) -> tuple:
    """
    Build the model to a temp file on the Jetson, streaming output live.

    Returns (temp_path, captured_output). The caller must call
    validate_and_promote_model() to rename temp_path → output_path.
    """
    print(f"\nBuilding statistical background model on Jetson...")
    print(f"  voxel_size={voxel_size}, max_frames={max_frames}")

    # Build to a temp filename so a crashed/empty build never silently
    # overwrites a working model.
    if output_path.endswith('.npz'):
        temp_path = output_path[:-4] + '_new.npz'
    else:
        temp_path = output_path + '_new.npz'
    print(f"  Building to temp: {temp_path}")

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
        f"--output {temp_path} "
        f"--voxel_size {voxel_size} "
        f"--max_frames {max_frames}"
    )

    print("  Running build (may take 30-60s)...")
    # Stream output to console while capturing for voxel-count validation.
    proc = subprocess.Popen(
        ["ssh", f"{user}@{ip}", cmd],
        stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True
    )
    captured_lines = []
    for line in proc.stdout:
        # Check abort each output line; kill the SSH process so proc.wait()
        # returns non-zero and the existing cleanup block (rm -f temp) fires.
        if abort is not None and abort.is_set():
            print("\n  Abort signal received — killing model build.")
            proc.kill()
            break   # fall through to proc.wait(); non-zero rc triggers cleanup
        print(line, end="")
        sys.stdout.flush()
        captured_lines.append(line)
    proc.wait()
    captured_output = "".join(captured_lines)

    if proc.returncode != 0:
        print("  ERROR: Model build failed (non-zero exit).")
        ssh_run(ip, user, f"rm -f {temp_path}")
        sys.exit(1)

    # Verify temp model was created
    if not ssh_ok(ip, user, f"test -f {temp_path}"):
        print(f"  ERROR: Temp model not found at {temp_path}")
        sys.exit(1)

    size_result = ssh_run(ip, user, f"du -sh {temp_path}")
    print(f"  Temp model built: {size_result.stdout.strip()}")

    return temp_path, captured_output


def validate_and_promote_model(ip: str, user: str,
                               temp_path: str, final_path: str,
                               build_output: str):
    """
    Parse build output for frame/voxel counts. If valid, rename
    temp_path → final_path on the Jetson. On failure, delete temp_path
    and exit without touching final_path.
    """
    print("\nValidating build output...")

    # "Background voxels (>= N points): M" — post-filter count (authoritative)
    voxel_count = None
    m = re.search(r'Background voxels[^:]*:\s*(\d+)', build_output)
    if m:
        voxel_count = int(m.group(1))
    else:
        # Fallback: "Model saved → ... (N background voxels)"
        m2 = re.search(r'Model saved.*\((\d+) background voxels\)', build_output)
        if m2:
            voxel_count = int(m2.group(1))

    # "Build complete: N frames, M voxels occupied"
    frames_match = re.search(r'Build complete:\s*(\d+)\s+frames', build_output)
    frame_count = int(frames_match.group(1)) if frames_match else None

    if frame_count is not None and frame_count == 0:
        print(f"\nERROR: Build produced 0 frames — "
              f"the bag contained no valid point cloud frames.")
        print("The existing model was NOT overwritten.")
        ssh_run(ip, user, f"rm -f {temp_path}")
        sys.exit(1)

    if voxel_count is not None and voxel_count < VOXEL_MIN_COUNT:
        print(f"\nERROR: Build produced only {voxel_count} background voxels "
              f"(minimum required: {VOXEL_MIN_COUNT}).")
        print("The existing model was NOT overwritten.")
        ssh_run(ip, user, f"rm -f {temp_path}")
        sys.exit(1)

    if voxel_count is None:
        print("  WARNING: Could not parse voxel count from build output.")
        print("  Proceeding based on non-zero exit code and file existence.")
    else:
        print(f"  Voxel count OK: {voxel_count} background voxels")

    # Atomic rename: temp → final. Old model is only replaced here, after
    # all validation passes.
    print(f"  Promoting {temp_path} → {final_path}")
    ssh_run(ip, user, f"mv {temp_path} {final_path}", check=True)
    return final_path


def download_model(ip: str, user: str,
                   remote_path: str, local_path: str):
    print(f"\nDownloading model to {local_path}...")
    Path(local_path).parent.mkdir(parents=True, exist_ok=True)

    # scp to a sibling .part file; atomic rename on success so a mid-transfer
    # kill never overwrites the previous good model with a truncated file.
    part_path = local_path + ".part"
    result = subprocess.run([
        "scp",
        f"{user}@{ip}:{remote_path}",
        part_path
    ])
    if result.returncode != 0:
        if os.path.exists(part_path):
            os.remove(part_path)
        print("  ERROR: Download failed.")
        sys.exit(1)
    os.replace(part_path, local_path)   # atomic same-directory rename
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


# ─── Fused-node helpers ───────────────────────────────────────────────────────

def load_fusion_sources(config_path: str) -> list:
    """Return ordered list of physical node names from fusion.sources in config."""
    with open(config_path) as f:
        data = yaml.safe_load(f)
    sources = data.get('fusion', {}).get('sources', [])
    return [s['node'] for s in sources]


def cleanup_failed_node(node_name: str, args, config_path: str) -> None:
    """Best-effort cleanup of Jetson + local artifacts after a failed or aborted rebuild.

    NEVER removes the final good model (background_statistical_nodeX.npz on the
    Jetson or models/background_statistical_nodeX.npz locally).
    Each step is individually try/except so one failure does not skip the rest.
    """
    node = load_node_config(config_path, node_name)
    ip   = node['jetson_ip']
    user = node['jetson_user']
    bag_name = (args.bag_name if args.bag_name is not None
                else node.get('rebuild_background', {}).get('bag_name', 'empty_scene_rebuild'))
    remote_model = node.get('bg_model_path', f"~/background_statistical_{node_name}.npz")
    # Only the _new.npz temp file — NEVER the final good model.
    temp_model = (remote_model[:-4] + '_new.npz' if remote_model.endswith('.npz')
                  else remote_model + '_new.npz')

    project_root = Path(__file__).parent.parent.parent
    # .part is left by an interrupted download_model scp.
    local_part = str(project_root / "models" /
                     f"background_statistical_{node_name}.npz") + ".part"

    print(f"\n[{node_name}] Running failure cleanup...")

    # 1. Stop any decoupled Jetson recording (launched with nohup, runs independently).
    try:
        ssh_run(ip, user, "pkill -9 -f 'ros2 bag record' 2>/dev/null; true")
        print(f"  [{node_name}] pkill ros2 bag record — done")
    except Exception as exc:
        print(f"  [{node_name}] pkill failed: {exc}")

    # 2. Remove bag dir from Jetson.
    try:
        ssh_run(ip, user, f"rm -rf ~/{bag_name}")
        print(f"  [{node_name}] rm -rf ~/{bag_name} — done")
    except Exception as exc:
        print(f"  [{node_name}] bag removal failed: {exc}")

    # 3. Remove temp model from Jetson (_new.npz only — NOT the final good model).
    try:
        ssh_run(ip, user, f"rm -f {temp_model}")
        print(f"  [{node_name}] rm -f {temp_model} — done")
    except Exception as exc:
        print(f"  [{node_name}] temp model removal failed: {exc}")

    # 4. Remove local .part file if a partial scp download was interrupted.
    try:
        if os.path.exists(local_part):
            os.remove(local_part)
            print(f"  [{node_name}] removed local {local_part}")
    except Exception as exc:
        print(f"  [{node_name}] local .part removal failed: {exc}")


# ─── Single-node rebuild ──────────────────────────────────────────────────────

def run_single_node(node_name: str, args, config_path: str,
                    interactive: bool = True,
                    nas_lock=None,
                    abort=None) -> bool:
    """Run the full 10-step rebuild for one physical node. Returns True on success.

    interactive=False  skips all input() prompts (parallel worker mode;
                       run_fused hoists them before spawning threads).
    nas_lock           threading.Lock passed by run_fused to serialise NAS
                       rsync pushes; None on the single-node path (no locking).
    abort              threading.Event checked at step boundaries and inside the
                       recording countdown and model-build loops so a peer's
                       failure stops this node within ~1 s of being set.

    sys.exit(0) from interactive prompts is re-raised (user abort → stop all).
    sys.exit(1) from step failures is caught and converted to False so the
    caller (run_fused worker) can run per-node cleanup.
    """
    project_root = Path(__file__).parent.parent.parent
    try:
        node = load_node_config(config_path, node_name)
        ip   = node['jetson_ip']
        user = node['jetson_user']
        topic = node.get('lidar_topic', '/livox/lidar')

        # Resolution order: CLI arg (if explicitly passed) > config value > hardcoded default
        rb_cfg     = node.get('rebuild_background', {})
        duration   = args.duration   if args.duration   is not None else rb_cfg.get('duration',  60)
        voxel_size = args.voxel_size if args.voxel_size is not None else rb_cfg.get('voxel_size', 0.15)
        max_frames = args.max_frames if args.max_frames is not None else rb_cfg.get('max_frames', 300)
        bag_name   = args.bag_name   if args.bag_name   is not None else rb_cfg.get('bag_name',  'empty_scene_rebuild')

        if duration < 30:
            print("ERROR: --duration must be at least 30 seconds.")
            sys.exit(1)

        # Determine output paths — bg_model_path in config is the remote (Jetson) path
        remote_model = node.get('bg_model_path', f"~/background_statistical_{node_name}.npz")
        if args.output:
            local_model = args.output
        else:
            local_model = str(project_root / "models" /
                             f"background_statistical_{node_name}.npz")

        print("=" * 60)
        print(f"Background Model Reconstruction — {node_name}")
        print("=" * 60)
        print(f"  Jetson:      {user}@{ip}")
        print(f"  Topic:       {topic}")
        print(f"  Duration:    {duration}s")
        print(f"  Voxel size:  {voxel_size}m")
        print(f"  Output:      {local_model}")
        if args.force:
            print(f"  --force:     pre-recording LiDAR check will be skipped")
        print()

        # Room-empty confirmation — skipped in parallel mode; run_fused hoists it.
        if interactive:
            print("IMPORTANT: The room must be completely empty during recording.")
            answer = input("Is the room empty? [y/N] ").strip().lower()
            if answer != 'y':
                print("Please clear the room first, then re-run.")
                sys.exit(0)

        if abort is not None and abort.is_set():
            sys.exit(1)

        # Step 1: pre-flight checks
        check_disk_space(ip, user, interactive=interactive)
        check_lidar_publishing(ip, user, topic, node_name=node_name, force=args.force)

        if abort is not None and abort.is_set():
            sys.exit(1)

        # Step 2: record (countdown loop checks abort each second)
        db3_path, bag_name = record_empty_scene(
            ip, user, topic, duration, bag_name, abort=abort)

        if abort is not None and abort.is_set():
            sys.exit(1)

        # Step 3: validate bag before touching the existing model
        validate_bag_size(ip, user, db3_path, node_name, topic)

        if abort is not None and abort.is_set():
            sys.exit(1)

        # Step 4: build to temp file (stdout loop checks abort each line)
        temp_path, build_output = build_model_on_jetson(
            ip, user, db3_path, remote_model,
            voxel_size, max_frames, abort=abort)

        if abort is not None and abort.is_set():
            sys.exit(1)

        # Step 5: validate voxel count, then atomically rename temp → final
        validate_and_promote_model(ip, user, temp_path, remote_model, build_output)

        if abort is not None and abort.is_set():
            sys.exit(1)

        # Step 6: download (only reached after successful promotion)
        download_model(ip, user, remote_model, local_model)

        # Trigger NAS archive hook for the new model if NAS is configured.
        # Uses check=False so a NAS connectivity problem never aborts a successful rebuild.
        # Serialised behind nas_lock when running in parallel to protect the Synology.
        if Path.home().joinpath(".nas_password").exists():
            hook = Path(__file__).parent / "post_record_hook.py"
            if nas_lock is not None:
                nas_lock.acquire()
            try:
                subprocess.run(
                    [sys.executable, str(hook),
                     "--with-model", "--node", node_name],
                    check=False,
                )
            finally:
                if nas_lock is not None:
                    nas_lock.release()

        if not args.keep_bag:
            cleanup_bag(ip, user, bag_name)

        print_next_steps(local_model, config_path, node_name)
        return True

    except SystemExit as e:
        if e.code == 0:
            raise   # user-initiated abort (room not empty, disk-space prompt) — propagate
        return False


def run_fused(args, config_path: str):
    """Rebuild background for every physical node in fusion.sources, in parallel.

    Interactive prompts are hoisted here before threads start so workers run
    non-interactively.  The first failing worker sets abort; all workers check
    it cooperatively at step boundaries and inside long loops.  NAS rsync
    pushes are serialised via nas_lock; recording + model build run fully in
    parallel.
    """
    if args.output:
        print("ERROR: --output is ambiguous when rebuilding multiple nodes (--node fused).")
        print("       Remove --output; each node uses its own bg_model_path from config.")
        sys.exit(2)

    node_names = load_fusion_sources(config_path)

    # (a) Hoist interactive prompts before spawning threads — one prompt for all nodes.
    print("IMPORTANT: ALL LiDAR-covered areas must be completely empty during recording.")
    answer = input("Is the room empty? [y/N] ").strip().lower()
    if answer != 'y':
        print("Please clear the room first, then re-run.")
        sys.exit(0)

    # (b) Concurrency primitives.
    abort        = threading.Event()   # set by first failing worker; peers check it
    nas_lock     = threading.Lock()    # serialises NAS rsync — NOT the build
    results      = {}                  # node_name -> bool, written by workers
    results_lock = threading.Lock()    # guards results dict

    def worker(node_name):
        ok = run_single_node(
            node_name, args, config_path,
            interactive=False,
            nas_lock=nas_lock,
            abort=abort,
        )
        if not ok:
            # Signal all other workers to stop at their next abort checkpoint.
            if not abort.is_set():
                print(f"\n[{node_name}] Failed — signalling abort to peer nodes.")
            abort.set()
            # (e) Per-node failure cleanup — bag, temp model, local .part.
            cleanup_failed_node(node_name, args, config_path)
        with results_lock:
            results[node_name] = ok

    # Spawn one thread per physical node and wait for all to finish.
    threads = [
        threading.Thread(target=worker, args=(n,), daemon=True, name=n)
        for n in node_names
    ]
    for t in threads:
        t.start()
    for t in threads:
        t.join()

    # (f) Summary + exit.
    print()
    print("=" * 60)
    print("Fused rebuild summary:")
    print("=" * 60)
    all_ok = True
    for node_name in node_names:
        ok = results.get(node_name, False)
        print(f"  {node_name}  {'✓' if ok else '✗'}")
        if not ok:
            all_ok = False

    sys.exit(0 if all_ok else 1)


# ─── main ─────────────────────────────────────────────────────────────────────

def main():
    script_dir = Path(__file__).parent
    project_root = script_dir.parent.parent
    default_config = str(project_root / "config" / "nodes_config.yaml")

    parser = argparse.ArgumentParser(
        description="Rebuild statistical background model for LiDAR pipeline",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__
    )
    parser.add_argument('--node', type=str, default='node1',
                        help='Node name from nodes_config.yaml (default: node1)')
    parser.add_argument('--config', type=str, default=default_config,
                        help='Path to nodes_config.yaml')
    parser.add_argument('--duration', type=int, default=None,
                        help='Recording duration in seconds (default: from config, 60). '
                             'More = better model. 30s minimum.')
    parser.add_argument('--voxel_size', type=float, default=None,
                        help='Voxel size for background model in meters '
                             '(default: from config, 0.15). Larger = more robust to noise.')
    parser.add_argument('--max_frames', type=int, default=None,
                        help='Max frames to use for model building (default: from config, 300)')
    parser.add_argument('--output', type=str, default=None,
                        help='Local output path for the model '
                             '(default: models/background_statistical_<node>.npz)')
    parser.add_argument('--bag_name', type=str, default=None,
                        help='Name for the temporary rosbag on Jetson '
                             '(default: from config, empty_scene_rebuild)')
    parser.add_argument('--keep_bag', action='store_true',
                        help='Do not delete the rosbag after building '
                             '(warning: uses disk space on Jetson)')
    parser.add_argument('--force', action='store_true',
                        help='Skip the pre-recording LiDAR publishing check. '
                             'Use only for debugging — bag size and voxel '
                             'count checks still run unconditionally.')
    args = parser.parse_args()

    # Early virtual-node check — before any jetson_user access.
    node_dict = load_node_config(args.config, args.node)
    if node_dict.get('virtual', False):
        run_fused(args, args.config)
        return  # run_fused calls sys.exit

    # Single-node path: interactive=True (prompts active), nas_lock=None, abort=None.
    ok = run_single_node(args.node, args, args.config)
    sys.exit(0 if ok else 1)


if __name__ == '__main__':
    main()
