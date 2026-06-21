# LiDAR Social Recognition

Indoor multi-LiDAR sensing pipeline for studying how the physical layout of a
shared space shapes spontaneous social interaction. Ceiling-mounted Livox
MID-360 sensors on Jetson edge nodes produce point clouds; an offline pipeline
removes the static background, detects and tracks people, and derives
encounters between them. Developed for deployment in the Hunt Library
congregation area (CMU NSF SAI Award #2425121).

The system currently runs a **dual-LiDAR fused pipeline** (node1 + node3,
overlapping fields of view, ICP-calibrated into a common frame). It is offline
by design: recording happens on the Jetsons, inference runs on the development
workstation.

## Current Deployment Stage

! [node1](../lidar_node1.jpeg)
! [node2](../lidar_node3.jpeg)
## Quickstart

Bring up the full fused pipeline (both LiDARs, background removal, fusion
overlay, and tracking) with one command:

```bash
python3 pipeline/00_start_driver_rosbridge/launcher.py --start --node fused --with-tracking
```

Stop it (by default this stops only the local overlay/tracking; add
`--stop-deps` to also stop node1/node3):

```bash
python3 pipeline/00_start_driver_rosbridge/launcher.py --stop --node fused
```

Check what is running:

```bash
python3 pipeline/00_start_driver_rosbridge/launcher.py --status --node fused
```

Rebuild background models for both physical nodes (records an empty scene on
each Jetson and builds a fresh statistical model):

```bash
python3 pipeline/00_start_driver_rosbridge/rebuild_background.py --node fused
```

A single physical node can always be targeted directly with `--node node1` or
`--node node3`.

## Repository layout

```
config/
  nodes_config.yaml          Single source of truth: per-node params,
                             the fused virtual node, and the fusion section
                             (ICP transform, sources, output topic).

pipeline/                    The processing stages, in execution order:
  00_start_driver_rosbridge/ launcher, NAS archiver, background rebuild
  01_background_removal/      statistical background model build + removal node
  02_detection/              Euclidean clustering person detection, ROI tools
  03_tracking/               AB3DMOT-style tracking, behaviour classification
  04_encounter_detection/    collision / encounter detection
  05_visualization/          detection + box visualisation
  06_fusion/                 dual-LiDAR overlay node, fused-detection verifier

eval/                        ATC-dataset loading, validation, detection and
                             tracking parameter sweeps.

models/                      Per-node statistical background models (.npz).

calib_out/                   ICP calibration output (node3 → node1 transform).

docs/                        Full documentation (see below).
```

## Documentation

| Doc | Covers |
|-----|--------|
| [docs/architecture.md](docs/architecture.md) | Dual-LiDAR fusion, the pipeline stages and data flow, the fused virtual node, the single-source config model |
| [docs/setup.md](docs/setup.md) | Workstation setup: Ubuntu, ROS2 Jazzy, the `livox` conda env, CUDA for the RTX 5070, building `livox_ros_driver2` |
| [docs/hardware_network.md](docs/hardware_network.md) | Nodes and IPs, **MID-360 vs MID-360S differences**, campus networking and Tailscale, the NAS and its safety rules, clock sync |
| [docs/calibration_tuning.md](docs/calibration_tuning.md) | ICP calibration (the 180° symmetry trap and how it's resolved), background voxel sizing, ROI, static-person handling, detection/tracking parameters and their rationale |
| [docs/operations.md](docs/operations.md) | Runbook: every launcher command and flag, background rebuild (including the parallel fused rebuild), and troubleshooting |

## Status

Dual-LiDAR fusion is implemented and verified end to end: node3 is configured
and calibrated to node1's frame, clocks are synchronised, the fused foreground
feeds detection and tracking, and ghost-track suppression and static-person
detection are in place. Full Hunt Library deployment, including ROI re-tuning
for the real space and additional MID-360S nodes, is scheduled for July 2026.
