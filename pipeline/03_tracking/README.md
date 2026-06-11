# Step 03: Multi-Object Tracking

## Status

This module is **pending implementation**. No source files exist in this directory yet.

## Planned approach

The tracker will consume detection centroids from `/detection_centers` (published by Step 02) and maintain consistent person identities across frames. The intended algorithm is **AB3DMOT** (3D Multi-Object Tracking using the 3D Kalman filter and Hungarian assignment), which is well-suited for overhead LiDAR with sparse, top-down point clouds.

## Expected Input / Output

| | Topic |
|---|---|
| Input | `/detection_centers` (PointCloud2 of per-frame centroids, from clustering_node) |
| Output | `/tracked_persons` (tracked identities with position history) |

## Next steps

1. Integrate AB3DMOT or a compatible tracker
2. Feed tracked trajectories into Step 04 (encounter detection)
3. Replace the ATC-dataset-based encounter logic with live tracked trajectories
