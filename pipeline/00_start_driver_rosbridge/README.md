# Step 00: Start Driver and rosbridge

## What this does

`launcher.py` is a single-script controller that SSHes into the Jetson node and starts (or stops, or checks) three services in sequence: the rosbridge WebSocket server, the Livox LiDAR ROS2 driver, and the statistical background removal node. It can also launch the clustering node locally after the Jetson services are up.

## Input / Output

| | Topic / Service |
|---|---|
| Starts on Jetson | rosbridge WebSocket (port 9090) |
| Starts on Jetson | livox_ros_driver2 → publishes `/livox/lidar` |
| Starts on Jetson | statistical_bg_node → publishes `/livox/lidar_foreground` |
| Optionally starts locally | clustering_node (via `--with-clustering`) |

## Usage

```bash
# Start all three Jetson services
python3 launcher.py --start

# Start Jetson services, then also launch clustering locally
python3 launcher.py --start --with-clustering

# Check which services are running
python3 launcher.py --status

# Stop all Jetson services
python3 launcher.py --stop
```

## Key parameters (CONFIG dict in launcher.py)

| Parameter | Value | Effect |
|---|---|---|
| jetson_ip | 172.26.42.167 | SSH target for all remote commands |
| jetson_user | kelrod | SSH username |
| rosbridge_port | 9090 | WebSocket port checked after start |
| bg_sigma | 2.0 | Passed to statistical_bg_node |
| bg_z_min | -2.8 | Passed to statistical_bg_node |
| bg_z_max | -0.5 | Passed to statistical_bg_node |
| cluster_tol | 0.4 | Passed to clustering_node (--with-clustering) |
| cluster_min | 8 | Minimum points per cluster |
| cluster_max | 800 | Maximum points per cluster |
| max_persons | 20 | Maximum detection outputs |

## Prerequisites

- Passwordless SSH to `kelrod@172.26.42.167`
- Background model `~/background_statistical.npz` present on Jetson (see Step 01 to build it)
- ROS2 Humble sourced at `/opt/ros/humble` and `~/ros2_ws/install` on Jetson

## Logs (on Jetson)

| Service | Log file |
|---|---|
| rosbridge | /tmp/rosbridge.log |
| LiDAR driver | /tmp/lidar_driver.log |
| BG removal | /tmp/bg_removal.log |
