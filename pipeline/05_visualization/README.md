# Step 05: Visualization

## What this does

`visualize_detections.py` is a ROS2 node that replays pre-recorded Hunt Library point cloud frames and their corresponding detection results in Foxglove. It reads `.bin` files (x, y, z, intensity as float32) from a local directory and matching `.txt` prediction files, then publishes them at 10 Hz so they can be viewed in Foxglove Studio.

## Input / Output

| | Topic / File |
|---|---|
| Input files | `~/Desktop/research/summer_research/dataset/hunt_library_bins/*.bin` |
| Input preds | `.../O-LiPeDeT-.../outputs/preds_hunt_library/run_00/preds/*.bin.txt` |
| Output | `/livox/lidar` (PointCloud2, replayed point cloud) |
| Output | `/detected_boxes` (visualization_msgs/MarkerArray, bounding boxes) |

## Usage

```bash
# Requires ROS2 Humble sourced
python3 visualize_detections.py
# Then open Foxglove Studio and connect to the running ROS2 instance
```

## Prediction file format

Each `.txt` file contains one detection per line:

```
<class> <x> <y> <z> <dx> <dy> <dz> <heading> <confidence>
```

The node reads fields 1–6 (x, y, z, dx, dy, dz) and optionally field 8 (confidence). The marker z position is shifted by `dz/2` to convert from bottom-center to center.

## Key parameters

| Parameter | Value | Effect |
|---|---|---|
| Timer rate | 10 Hz (0.1 s) | Playback speed |
| frame_id | livox_frame | ROS coordinate frame for all published messages |
| Marker lifetime | 0.2 s | Boxes disappear if not refreshed |
| Marker color | orange (r=1.0, g=0.5, b=0.0, a=0.4) | Bounding box appearance in Foxglove |
| Point cloud fields | x, y, z, intensity | float32, 16 bytes/point |
