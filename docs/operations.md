# Operations

A runbook for starting, stopping, rebuilding, archiving, and troubleshooting
the pipeline. Everything is driven by `--node`, which selects a block from
`config/nodes_config.yaml`. Commands below assume you are at the repository
root.

## Starting and stopping the pipeline

`pipeline/00_start_driver_rosbridge/launcher.py` controls all services for a
node.

Bring up the full fused pipeline — both LiDARs, the fusion overlay, and
tracking:

```bash
python3 pipeline/00_start_driver_rosbridge/launcher.py --start --node fused --with-tracking
```

Bring up a single node with tracking:

```bash
python3 pipeline/00_start_driver_rosbridge/launcher.py --start --node node1 --with-tracking
```

Use `--with-clustering` instead of `--with-tracking` if you only want detection
boxes without tracks.

Stopping a node:

```bash
python3 pipeline/00_start_driver_rosbridge/launcher.py --stop --node fused
```

For the fused node, `--stop` by default stops only the **local** overlay and
tracking — it leaves node1 and node3 running, since you usually want to restart
the overlay without re-recording the edge nodes. To also stop the underlying
physical nodes, add `--stop-deps`:

```bash
python3 pipeline/00_start_driver_rosbridge/launcher.py --stop --node fused --stop-deps
```

Checking status and restarting:

```bash
python3 pipeline/00_start_driver_rosbridge/launcher.py --status --node fused
python3 pipeline/00_start_driver_rosbridge/launcher.py --restart --node fused
```

### launcher.py flags

| Flag | Effect |
|------|--------|
| `--start` | Start services for the node |
| `--stop` | Stop services (fused: local only, unless `--stop-deps`) |
| `--restart` | Restart services |
| `--status` | Report which services are running |
| `--sync-nas` | Sync pending data to the NAS |
| `--with-clustering` | Also launch the clustering node locally |
| `--with-tracking` | Also launch the tracking node locally |
| `--stop-deps` | (fused) also stop node1/node3 when stopping fused |
| `--node` | Node key from the config (default `node1`) |
| `--config` | Path to `nodes_config.yaml` |

When the fused stack starts, it checks each physical node's health first; a node
already running is left in place rather than restarted. If a required physical
node cannot be started, the fused stack aborts rather than coming up half-formed.

## Rebuilding background models

A background model is an empty-scene snapshot. Rebuild whenever the furniture or
fixed environment changes, or when you see furniture leaking through as false
detections.

Rebuild a single node:

```bash
python3 pipeline/00_start_driver_rosbridge/rebuild_background.py --node node1
```

Rebuild **both** physical nodes for the fused setup:

```bash
python3 pipeline/00_start_driver_rosbridge/rebuild_background.py --node fused
```

### How the fused rebuild behaves

`--node fused` rebuilds node1 and node3 **in parallel**, and is built around the
fact that they share one physical empty scene:

- You are asked **once**, up front, to confirm the room is empty — not once per
  node. After you confirm, both nodes record simultaneously, so they capture the
  **same time window** of the same empty room. (This is the main reason the
  rebuild is parallel rather than serial: a serial rebuild records two different
  windows, and someone can walk into the room between them.)
- Recording and model-building run fully in parallel. Only the final NAS push is
  serialised between the two nodes, to avoid two simultaneous connections to the
  Synology.
- It is **fail-fast**. If either node fails, the other is aborted: the script
  kills the local build process *and* sends an SSH `pkill` to that Jetson to stop
  its recording (recording is detached on the Jetson with `nohup`, so killing the
  local script alone does not stop it).
- On failure or abort, each affected node cleans up **only its own temporary
  artifacts** — the empty-scene bag, the `_new.npz` work-in-progress model on the
  Jetson, and any partial `.npz.part` download locally. The **previous good model
  is never touched**: the live model file is only replaced at the very last
  atomic step, which a failed run never reaches.
- The run exits 0 only if **both** nodes succeed; otherwise it exits non-zero
  and prints a per-node summary.

The single-node rebuild (`--node node1`) is unchanged by any of this — it still
prompts interactively and runs the same ten-step sequence as before.

### rebuild_background.py flags

| Flag | Effect |
|------|--------|
| `--node` | Node key, or `fused` for both physical nodes (default `node1`) |
| `--config` | Path to `nodes_config.yaml` |
| `--duration` | Recording seconds (overrides config) |
| `--voxel_size` | Voxel size in metres (overrides config) |
| `--max_frames` | Max rosbag frames used (overrides config) |
| `--output` | Local output path for the `.npz` (rejected with `--node fused`) |
| `--bag_name` | Temp rosbag dir name on the Jetson |
| `--keep_bag` | Don't delete the bag after the model is built |
| `--force` | Skip the LiDAR liveness check |

`--output` names a single file, which is ambiguous when rebuilding two nodes, so
it is rejected in fused mode; let each node write to its own `bg_model_path` from
the config instead.

## Archiving to the NAS

Recorded data and models are pushed to the Synology NAS via rsync. The launcher
can sync pending data (`--sync-nas`), and `nas_archive.py` can push specific
items:

```bash
python3 pipeline/00_start_driver_rosbridge/nas_archive.py --check-space
python3 pipeline/00_start_driver_rosbridge/nas_archive.py --archive-model models/background_statistical_node1.npz --node node1
```

`post_record_hook.py` is invoked automatically after a stop or a rebuild and is a
no-op when no NAS password is configured.

> **NAS safety rule.** Never run `ls`, `find`, `du`, `wc`, or `df` against NAS
> directories — on a Synology with millions of files these freeze the unit for
> roughly half an hour. The archiver only ever uses permitted operations (`stat`
> on named paths, `df` on the volume mount point, directory listing via
> `os.scandir` over SSH, and rsync push from local to NAS). Do not add ad-hoc
> NAS shell commands to any script. See
> [hardware_network.md](hardware_network.md) for the full rule.

## Troubleshooting

**A code or config change has no effect.** ROS 2 launches run from the installed
copy under `install/`, not from `src/`. Editing a file in `src/` and re-running
`ros2 launch` runs the old installed version. Rebuild/reinstall, or confirm which
copy is actually being executed, before concluding a change "didn't work."

**A config value doesn't behave as written.** A value being present in
`nodes_config.yaml` does not guarantee it is the value in effect. Tuned numbers
have in the past lived only in evaluation scripts and never been promoted into
the production config. When in doubt, verify the running value directly rather
than trusting the file.

**A Jetson is unreachable / its IP changed.** Campus DHCP reassigns Jetson IPs,
and a node can drop off the network mid-session. The durable fix is Tailscale
plus a registered CMU-DEVICE MAC rather than relying on the DHCP-assigned campus
address; see [hardware_network.md](hardware_network.md). If a node is offline,
fused operations that depend on it will fail by design.

**A fused rebuild was aborted, but a bag may still be on the Jetson.** The abort
path stops a node's recording with an SSH `pkill`. If that node happens to be the
one that went unreachable, the `pkill` cannot land, and the detached recording
keeps running for the rest of its duration, filling Jetson disk. After any
aborted fused rebuild, SSH into the affected Jetson and confirm
`~/empty_scene_rebuild` is gone (and no `ros2 bag record` is still running)
before retrying.

**Furniture shows up as detections.** The background model is stale or was built
at too coarse a voxel size. Rebuild it; for nodes prone to furniture leakage use
the finer 0.10 m voxel size. See
[calibration_tuning.md](calibration_tuning.md).

**Two nearby people merge into one detection.** This is a clustering-tolerance
problem and it is research-critical — the social distances being studied must not
be clustered away. Do not raise `cluster_tol` to paper over it; the rationale and
the safe range are in [calibration_tuning.md](calibration_tuning.md).

**Clocks disagree across nodes.** "Each node is synced" is not the same as "the
nodes are synced to each other" — they can each be locked to a different NTP
source. The nodes must chain to a common reference (node1 acts as the server,
node3 syncs to node1). See [hardware_network.md](hardware_network.md).

### Working discipline

Two practices that this project has repeatedly found necessary: **change one
thing at a time**, and **diagnose before changing, verify after**. Most of the
hard-to-find problems above came from changing several things at once, or from
assuming a change took effect without confirming it.
