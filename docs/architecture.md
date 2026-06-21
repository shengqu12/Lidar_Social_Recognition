# Architecture

This document describes how the system is put together: the edge/workstation
split, the single-node and fused topologies, the data flow through the pipeline
stages, the fused virtual node, and the single-source configuration model.

## System overview

The system is **offline by design**. The Jetson edge nodes run only the LiDAR
driver, statistical background removal, and a rosbridge WebSocket server. All
detection, tracking, and encounter analysis run on the development workstation,
which connects to each Jetson's rosbridge over the network. There is no
real-time inference on the Jetsons.

This split keeps the edge nodes light (they publish foreground point clouds and
nothing more) and keeps the compute-heavy, frequently-changing parts of the
pipeline on a single machine where they can be tuned and re-run against
recorded data.

Two topologies are supported, selected by the `--node` argument throughout the
tooling:

- **Single node** (`--node node1`): one LiDAR, the simplest path. Used for
  per-sensor calibration, background rebuilds, and debugging.
- **Fused** (`--node fused`): two overlapping LiDARs (node1 + node3) combined
  into one common frame. This is the production configuration.

## Data flow

In a **single-node** path, everything downstream of the foreground topic runs
on the workstation:

```
Jetson (node1)                          Workstation
──────────────                          ───────────
LiDAR driver
   │  /livox/lidar
   ▼
statistical_bg_node
   │  /livox/lidar_foreground
   ▼
rosbridge  ───── WebSocket ──────────►  tracking_node --node node1
                                           │  ROI → accumulate → cluster
                                           │  → Kalman + Hungarian
                                           ▼
                                        /tracked_boxes, /tracked_centers
                                        data/tracklets/session_<time>.csv
```

In the **fused** path, each Jetson independently produces its own foreground.
The overlay node on the workstation pulls both, transforms node3 into node1's
frame using the ICP calibration, unions them, and republishes a single fused
foreground. Detection and tracking then run on that fused cloud exactly as they
would on a single node:

```
Jetson (node1)            Jetson (node3)              Workstation
──────────────            ──────────────              ───────────
LiDAR → bg removal        LiDAR → bg removal
   │ /livox/lidar_fg         │ /livox/lidar_fg
   ▼                         ▼
rosbridge ──┐          ┌── rosbridge
            │          │
            ▼          ▼
        overlay_node (06_fusion)
          - colours node1 (orange), node3 (blue)
          - applies ICP transform to node3
          - unions both foregrounds
          - republishes via node1's rosbridge
                 │  /fused/foreground
                 ▼
        tracking_node --node fused
          ROI → accumulate → cluster → track
                 │
                 ▼
        /tracked_boxes, /tracked_centers
        data/tracklets/session_<time>.csv
                 │
                 ▼
        04_encounter_detection (offline, from tracklets)
```

The overlay node runs on the workstation but publishes `/fused/foreground`
*through node1's rosbridge*, so that the tracking node — which only knows how to
talk to a rosbridge — can subscribe to the fused cloud the same way it would
subscribe to a single node's foreground. This is what the `publish_via: node1`
setting in the fusion config means.

## Pipeline stages

The `pipeline/` directory is numbered in execution order. Each stage subscribes
to the previous stage's output topic.

**00_start_driver_rosbridge** — orchestration, not signal processing.
`launcher.py` starts and stops services on the Jetsons and the workstation for
a given node (including bringing up the whole fused stack). `rebuild_background.py`
records an empty scene and builds a fresh background model.
`nas_archive.py` / `post_record_hook.py` push recorded data and models to the
NAS.

**01_background_removal** — `statistical_bg_build.py` builds a voxelised model
from an empty-scene rosbag, storing per-voxel mean and standard deviation (the
LiDAR's few-centimetre ranging noise is absorbed into the std).
`statistical_bg_node.py` runs on the Jetson, checking each incoming point
against its voxel's mean/std and keeping points that are more than `sigma`
standard deviations from the background. Input `/livox/lidar`, output
`/livox/lidar_foreground`.

**02_detection** — `clustering_node.py` takes a foreground cloud, applies the
ROI crop, accumulates frames (adaptively: more frames for slow/stationary
targets, fewer for fast movers), runs Euclidean clustering (BFS over a KD-tree),
validates each cluster's shape against person-like size and vertical-extent
bounds, and emits 3D bounding boxes and centroids. Its `detect()`,
`apply_roi()`, and `is_valid_person_cluster()` functions are importable, so the
tracking stage reuses them rather than duplicating the logic.

**03_tracking** — `tracking_node.py` integrates detection and tracking in a
single process: foreground → ROI + accumulate + cluster + shape validation →
Kalman prediction + Hungarian association → confirmed tracks. It publishes
`/tracked_boxes` (MarkerArray) and `/tracked_centers` (PointCloud2), and logs
one CSV per session to `data/tracklets/`. Behaviour classification (walking /
stationary / talking) runs alongside; sitting and standing are merged into a
single "stationary" label because vertical extent is unreliable in sparse
overhead clouds.

**04_encounter_detection** — `collision_detection.py` operates offline on the
tracklet output, identifying encounters between trajectories. It is
library-style, driven from the eval tooling.

**05_visualization** — `visualize_detections.py` renders point clouds and
detection boxes for inspection.

**06_fusion** — `overlay_node.py` is the dual-LiDAR combiner described above; it
also serves as a calibration diagnostic (in `--raw` mode it shows the dense
walls so two clouds can be visually aligned). `verify_fused_detection.py` is an
offline synthetic test of the fused-detection logic with no network or Jetsons
involved.

## The fused virtual node

`fused` is defined in the config alongside the physical nodes, but it is
**virtual**: it has no Jetson of its own. It carries `virtual: true`, the fused
foreground topic, and the full clustering / tracking / behaviour / ROI
parameters — but deliberately **no** `jetson_user`, `lidar_topic`,
`bg_model_path`, or `rebuild_background` block, because there is no physical
sensor behind it.

This is why operations on `fused` fan out to the real nodes. Starting the fused
stack iterates over `fusion.sources` (node1, node3) to bring up each physical
node, then launches the overlay locally. Rebuilding the background for `fused`
rebuilds each physical node's model in turn — there is no separate "fused
background", because fusion happens at the foreground level, after each node has
already removed its own background. Anything that reads `fused` for per-sensor
fields will, and should, fail fast rather than silently treat it as a real node.

## Single-source configuration

`config/nodes_config.yaml` is the **single source of truth**. Everything — the
launcher, the rebuild script, the clustering and tracking nodes, the overlay
node — reads node parameters from here via `--node`. There are no per-script
parameter copies.

The file has two top-level sections:

- **`nodes:`** — one block per node. Each physical node (node1, node3, and the
  not-yet-deployed node2) carries its connection details (`jetson_ip`,
  `jetson_user`, `rosbridge_port`), its background model settings, and its
  detection / tracking / behaviour / ROI parameters. The `fused` block lives
  here too, with the virtual-node caveats above.
- **`fusion:`** — the dual-LiDAR wiring: the ordered `sources` list (node1 is
  the reference frame and carries no transform; node3 carries its
  `transform_to_common` ICP matrix), the `output_topic` (`/fused/foreground`),
  the `output_frame`, and `publish_via` (which node's rosbridge relays the fused
  output).

A standing lesson from development: **a value being present in this file does
not mean it is in effect.** ROS 2 launches run from the installed copy under
`install/`, not from `src/`, and some tuned values have in the past lived only
in evaluation scripts without being promoted into the config that production
actually reads. When a parameter does not behave as the config suggests it
should, confirm which copy is actually running before changing anything. See
[operations.md](operations.md) for the verification steps.
