# Step 02: Person Detection (Euclidean Clustering)

## What this does

`clustering_node.py` runs locally (not on the Jetson) and connects to the Jetson's rosbridge WebSocket via `roslibpy`. It subscribes to the foreground point cloud, runs Euclidean clustering (BFS + KDTree) to group nearby points into person-sized blobs, and publishes the results back through rosbridge.

From a ceiling-mounted sensor at ~3 m, a person's top-down silhouette is a compact elliptical blob (~30–80 points). A tolerance of 0.4 m connects points belonging to the same person while keeping adjacent people (typically >0.5 m apart) separate.

References: Yamaguchi et al. 2024; Gómez et al. 2023; PCL EuclideanClusterExtraction.

## Input / Output

| | Topic |
|---|---|
| Input | `/livox/lidar_foreground` (PointCloud2, via rosbridge) |
| Output | `/detection_boxes` (visualization_msgs/MarkerArray) |
| Output | `/detection_centers` (sensor_msgs/PointCloud2 of centroids) |

## Usage

```bash
conda activate livox

# Connect to Jetson and start clustering
python3 clustering_node.py \
    --jetson_ip 172.26.42.167 \
    --topic /livox/lidar_foreground \
    --cluster_tol 0.4 \
    --min_points 8 \
    --max_points 800

# Run algorithm self-test without ROS or roslibpy
python3 clustering_node.py --test
```

## Key parameters

| Parameter | Default | Effect |
|---|---|---|
| jetson_ip | 172.26.42.167 | rosbridge host |
| port | 9090 | rosbridge WebSocket port |
| cluster_tol | 0.4 m | Max distance between two points in the same cluster. Too small (<0.25) splits one person; too large (>0.6) merges adjacent people. |
| min_points | 8 | Clusters smaller than this are discarded as noise |
| max_points | 800 | Clusters larger than this are discarded (e.g. carts, pillars) |
| max_persons | 20 | Maximum number of detections output per frame |
