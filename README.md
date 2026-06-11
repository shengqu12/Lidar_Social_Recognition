# LiDAR Social Recognition

NSF SAI Project (Award #2425121) — Studying how Hunt Library's physical space layout shapes spontaneous social interaction and social capital formation among students.

**Institution:** Carnegie Mellon University  
**Advisors:** Prof. Mario Berges (CEE/HCI), Prof. Katherine Flanigan  
**Hardware:** Livox Mid-360 LiDAR sensors, NVIDIA Jetson Orin Nano nodes

---

## Pipeline Overview

```
Livox Mid-360 (ceiling-mounted, inverted)
          |
          v  /livox/lidar
  [00] launcher.py  ──── SSH ────>  Jetson (172.26.42.167)
                                      |
                                      v  ROS2 Humble
                              [01] statistical_bg_node.py
                                    (background removal)
                                      |
                                      v  /livox/lidar_foreground
                              rosbridge ws://jetson:9090
                                      |
          <────────── roslibpy ───────+
          |
          v  /livox/lidar_foreground
  [02] clustering_node.py  (local, conda activate livox)
       Euclidean clustering → person bounding boxes
          |
          +──> /detection_boxes   (MarkerArray, Foxglove)
          +──> /detection_centers (PointCloud2, for tracking)
          |
          v
  [03] tracking/  (pending — AB3DMOT)
          |
          v
  [04] collision_detection.py
       proximity + heading + deceleration → encounter events
          |
          v  encounters_raw.csv
  [05] visualize_detections.py
       Replays .bin frames + prediction boxes in Foxglove
```

---

## Repository Structure

```
lidar_social_recognition/
├── pipeline/
│   ├── 00_start_driver_rosbridge/   # One-script launcher: rosbridge + LiDAR driver + BG node on Jetson
│   ├── 01_background_removal/       # Statistical BG model builder + ROS2 filtering node
│   ├── 02_detection/                # Euclidean clustering node (runs locally via roslibpy)
│   ├── 03_tracking/                 # Pending — AB3DMOT tracker (no source files yet)
│   ├── 04_encounter_detection/      # Proximity + heading + deceleration encounter detector
│   └── 05_visualization/            # ROS2 node: replay .bin frames + detection boxes in Foxglove
├── eval/
│   ├── load_atc.py                  # Load ATC pedestrian dataset, detect encounters
│   └── validation.py                # Precision / Recall / F1 evaluation against ATC ground truth
├── dataset/
│   ├── ATC_dataset/                 # ATC pedestrian CSV + group interaction ground truth
│   ├── hunt_library_bins/           # .bin point cloud frames (x y z intensity, float32)
│   └── human_test_20260607_154314/  # ROS2 bag recorded during live test
├── data/
│   └── encounters/                  # Output CSV: encounters_raw.csv
└── card_reader/                     # Card reader data collection (separate sub-project)
```

---

## Quick Start

**Prerequisites:** SSH access to Jetson at `172.26.42.167` (user `kelrod`), background model already built and copied to `~/background_statistical.npz` on the Jetson.

```bash
# Start rosbridge + LiDAR driver + BG removal on Jetson
python3 pipeline/00_start_driver_rosbridge/launcher.py --start

# Check that all three services are running
python3 pipeline/00_start_driver_rosbridge/launcher.py --status

# Start Jetson services AND immediately launch clustering locally
python3 pipeline/00_start_driver_rosbridge/launcher.py --start --with-clustering

# Run only clustering locally (Jetson already running)
conda activate livox
python3 pipeline/02_detection/clustering_node.py \
    --jetson_ip 172.26.42.167 \
    --topic /livox/lidar_foreground

# Stop all Jetson services
python3 pipeline/00_start_driver_rosbridge/launcher.py --stop
```

**Build the background model** (run once from an empty-scene rosbag):

```bash
python3 pipeline/01_background_removal/statistical_bg_build.py \
    --bag ./data/rosbags/empty_scene \
    --output ./models/background_statistical.npz

scp ./models/background_statistical.npz kelrod@172.26.42.167:~/
```

---

## Hardware Setup

| Parameter | Value |
|---|---|
| Jetson IP | 172.26.42.167 |
| Jetson user | kelrod |
| SSH port | 22 |
| rosbridge websocket port | 9090 |
| LiDAR ROS topic | /livox/lidar |
| Foreground topic | /livox/lidar_foreground |
| LiDAR driver | livox_ros_driver2 (msg_MID360_launch.py) |
| ROS distribution | ROS2 Humble |

**Foxglove visualization:** Connect to `ws://172.26.42.167:9090` and add PointCloud2 panels for `/livox/lidar`, `/livox/lidar_foreground`, and a MarkerArray panel for `/detection_boxes`.

---

## Background Removal Parameters

| Parameter | Default | Meaning |
|---|---|---|
| sigma | 2.0 | Points more than sigma * std from background mean are foreground |
| z_min | -2.8 m | Minimum z (sensor inverted, so negative) |
| z_max | -0.5 m | Maximum z (cuts off ceiling plane) |
| voxel_size | 0.10 m | Spatial resolution of background model |

---

## Validation

Encounter detection is validated against the ATC pedestrian dataset (`dataset/ATC_dataset/`). The ATC CSV files contain columns `timestamp, person_id, x, y, z, velocity, angle1, angle2` with coordinates in millimeters (converted to meters on load). Ground truth social interaction pairs are in `groups_ATC-1.dat` (interaction type 1 = genuine social interaction).

The encounter detection pipeline in `eval/load_atc.py` applies: (1) displacement filter (>1.0 m total travel), (2) artifact ID removal (persons appearing in >30% of frames), (3) proximity check (distance < 1.5 m), (4) heading check (angle difference >= 90 degrees), (5) deceleration filter (velocity drop > 0.2 m/s in approach window), (6) deduplication (same pair within 3.0 s counts once). Precision/Recall/F1 are computed in `eval/validation.py`. The paper baseline cited in the source is Precision = 0.861.

---

## Dependencies

| Package | Used in |
|---|---|
| numpy | all modules |
| scipy (KDTree) | clustering_node.py |
| pandas | load_atc.py, validation.py |
| matplotlib | load_atc.py, validation.py |
| rclpy (ROS2 Humble) | statistical_bg_node.py, visualize_detections.py |
| sensor_msgs, visualization_msgs | statistical_bg_node.py, visualize_detections.py |
| roslibpy | clustering_node.py |
| rosbag2_py | statistical_bg_build.py |

Conda environment for local (non-Jetson) modules: `livox`
