# Step 01: Background Removal

## What this does

Two scripts handle background removal.

**`statistical_bg_build.py`** reads an empty-scene rosbag and builds a voxelized statistical background model: for each voxel it records the mean and standard deviation of all observed point positions. The LiDAR's ±2–3 cm noise is absorbed into the standard deviation so it does not produce false foreground detections. The model is saved as a `.npz` file.

**`statistical_bg_node.py`** runs on the Jetson as a ROS2 node. For each incoming point cloud frame it (1) clips to the configured z range, then (2) checks each point against the background model — any point whose distance from its voxel's background mean exceeds `sigma * max(std)` is kept as foreground.

References: PALMAR (Ul Alam et al. 2021), Brščić et al. 2013.

## Input / Output

| | Topic / File |
|---|---|
| Input (node) | `/livox/lidar` (PointCloud2, from Livox driver) |
| Output (node) | `/livox/lidar_foreground` (PointCloud2) |
| Input (builder) | empty-scene rosbag2 directory |
| Output (builder) | `background_statistical.npz` |

## Usage

```bash
# Build background model from empty-scene rosbag
python3 statistical_bg_build.py \
    --bag /path/to/empty_scene \
    --output ~/background_statistical.npz \
    --voxel_size 0.10 \
    --topic /livox/lidar

# Quick self-test (no rosbag needed)
python3 statistical_bg_build.py --test

# Convert an older voxel-occupancy model to statistical format
python3 statistical_bg_build.py \
    --convert ~/background_model.npz \
    --output ~/background_statistical.npz

# Run the node on Jetson (usually started by launcher.py)
python3 statistical_bg_node.py \
    --model ~/background_statistical.npz \
    --sigma 2.0 \
    --input_topic /livox/lidar \
    --output_topic /livox/lidar_foreground \
    --z_min -2.8 \
    --z_max -0.5
```

## Key parameters

| Parameter | Default | Effect |
|---|---|---|
| voxel_size | 0.10 m | Spatial resolution of background model |
| min_points | 3 | Min observations per voxel to include in model |
| max_frames | 500 | Max rosbag frames used for model building |
| sigma | 2.0 | Foreground threshold: distance > sigma*max(std) → foreground |
| z_min | -2.8 m | Lower z clip (sensor inverted, z is negative) |
| z_max | -0.5 m | Upper z clip (removes ceiling plane) |
