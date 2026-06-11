#!/usr/bin/env python3
"""
Euclidean Clustering Node - Person Detection
=============================================
Receives foreground point clouds from background removal,
applies Euclidean clustering to segment individual person blobs,
and publishes per-person 3D bounding boxes (for Foxglove visualization)
and centroid positions (for AB3DMOT tracking).

Pipeline position:
  /livox/lidar
      -> [statistical_bg_node]
      -> /livox/lidar_foreground
      -> [clustering_node]  <- this file
      -> /detection_boxes   (MarkerArray, Foxglove visualization)
      -> /detection_centers (PointCloud2, tracking input)

Launch (Legion laptop, connects to Jetson rosbridge via websocket):
  conda activate livox
  python3 clustering_node.py \\
      --jetson_ip 172.26.42.167 \\
      --topic /livox/lidar_foreground \\
      --cluster_tol 0.4 \\
      --min_points 8 \\
      --max_points 800

Algorithm:
  Euclidean clustering (a.k.a. Euclidean cluster extraction):
  Groups points whose pairwise distance is <= cluster_tol into the same cluster.
  Implemented as BFS + KDTree nearest-neighbor search.
  Well-suited for overhead ceiling-view scenes: a human body seen from above
  forms an elliptical blob; cluster_tol=0.4 m connects points on the same person
  while keeping adjacent people (typically >0.5 m apart) in separate clusters.

References:
  Yamaguchi et al. 2024 - Euclidean clustering for indoor LiDAR tracking
  Gomez et al. 2023 - Euclidean distance-based segmentation
  PCL EuclideanClusterExtraction (same algorithm in C++)

Subscription layer: uses websocket-client directly against rosbridge v2
protocol (ws://jetson:9090) rather than roslibpy, which has a blocking
run() call and is unreliable with high-frequency binary topics on
rosbridge 2.0.6 / ROS2 Humble.
"""

import argparse
import base64
import json
import struct
import threading
import time
from typing import List, Tuple

import numpy as np
from scipy.spatial import KDTree


# ─── Core Algorithm (no ROS dependency, independently testable) ──────────────

def euclidean_clustering(
    pts: np.ndarray,
    cluster_tol: float = 0.4,
    min_points: int = 8,
    max_points: int = 800,
) -> List[np.ndarray]:
    """
    Euclidean clustering on a foreground point cloud.

    Args:
        pts:          np.ndarray (N, 3), foreground points after BG removal
        cluster_tol:  Points within this distance (metres) join the same cluster.
                      Recommended range for 3 m ceiling view: 0.35-0.45 m
        min_points:   Clusters with fewer points are discarded (noise filter).
                      At 3 m height a person yields ~30-80 pts; 8 is conservative.
        max_points:   Clusters with more points are discarded (large objects/carts).

    Returns:
        clusters: list of np.ndarray, each shaped (M, 3), sorted large-to-small
    """
    if len(pts) < min_points:
        return []

    tree = KDTree(pts)
    visited = np.zeros(len(pts), dtype=bool)
    clusters = []

    for seed_idx in range(len(pts)):
        if visited[seed_idx]:
            continue

        # BFS expansion for this cluster
        cluster_indices = []
        queue = [seed_idx]
        visited[seed_idx] = True

        while queue:
            current = queue.pop(0)
            cluster_indices.append(current)
            neighbors = tree.query_ball_point(
                pts[current], cluster_tol)
            for nb in neighbors:
                if not visited[nb]:
                    visited[nb] = True
                    queue.append(nb)

        if min_points <= len(cluster_indices) <= max_points:
            clusters.append(pts[cluster_indices])

    # Sort by point count descending (more points = more likely a real person)
    clusters.sort(key=lambda c: len(c), reverse=True)
    return clusters


def cluster_to_bbox(cluster: np.ndarray) -> dict:
    """
    Compute 3D bounding box and centroid for one cluster.

    Returns dict with:
        center:   np.ndarray (3,) - centroid xyz
        size:     np.ndarray (3,) - extent (dx, dy, dz)
        bbox_min: np.ndarray (3,) - minimum corner
        bbox_max: np.ndarray (3,) - maximum corner
        n_points: int             - point count
    """
    bbox_min = cluster.min(axis=0)
    bbox_max = cluster.max(axis=0)
    center = (bbox_min + bbox_max) / 2.0
    size = bbox_max - bbox_min
    return {
        'center':   center,
        'size':     size,
        'bbox_min': bbox_min,
        'bbox_max': bbox_max,
        'n_points': len(cluster),
    }


# ─── RosBridge WebSocket Client ───────────────────────────────────────────────

try:
    import websocket
    WEBSOCKET_AVAILABLE = True
except ImportError:
    WEBSOCKET_AVAILABLE = False
    print("[WARN] websocket-client not installed. "
          "Run: pip install websocket-client")


class RosBridgeClient:
    """
    Minimal rosbridge v2 WebSocket client using websocket-client.

    Replaces roslibpy because roslibpy's run() blocks the calling thread
    and is unreliable with high-frequency binary topics (PointCloud2) on
    rosbridge 2.0.6 / ROS2 Humble.

    Protocol: https://github.com/RobotWebTools/rosbridge_suite/blob/ros2/ROSBRIDGE_PROTOCOL.md
    """

    def __init__(self, host: str, port: int):
        self.url = f"ws://{host}:{port}"
        self._topic_callbacks = {}   # topic -> callback(msg_dict)
        self._ws = None
        self._thread = None
        self._connected_event = threading.Event()
        self.is_connected = False

    def connect(self, timeout: float = 10.0) -> bool:
        self._ws = websocket.WebSocketApp(
            self.url,
            on_open=self._on_open,
            on_message=self._on_message,
            on_error=self._on_error,
            on_close=self._on_close,
        )
        self._thread = threading.Thread(
            target=self._ws.run_forever,
            kwargs={"ping_interval": 20, "ping_timeout": 10},
            daemon=True,
        )
        self._thread.start()
        connected = self._connected_event.wait(timeout=timeout)
        return connected

    def _on_open(self, ws):
        self.is_connected = True
        self._connected_event.set()

    def _on_message(self, ws, raw):
        try:
            data = json.loads(raw)
        except Exception:
            return
        if data.get("op") == "publish":
            topic = data.get("topic")
            cb = self._topic_callbacks.get(topic)
            if cb is not None:
                try:
                    cb(data.get("msg", {}))
                except Exception as e:
                    print(f"[WARN] callback error on {topic}: {e}")

    def _on_error(self, ws, err):
        print(f"[WARN] WebSocket error: {err}")

    def _on_close(self, ws, code, msg):
        self.is_connected = False
        print(f"[INFO] WebSocket closed: {code}")

    def subscribe(self, topic: str, msg_type: str,
                  callback, throttle_rate: int = 0):
        self._topic_callbacks[topic] = callback
        self._send({
            "op": "subscribe",
            "topic": topic,
            "type": msg_type,
            "throttle_rate": throttle_rate,
            "queue_length": 1,
        })

    def advertise(self, topic: str, msg_type: str):
        self._send({
            "op": "advertise",
            "topic": topic,
            "type": msg_type,
        })

    def publish(self, topic: str, msg: dict):
        self._send({
            "op": "publish",
            "topic": topic,
            "msg": msg,
        })

    def _send(self, obj: dict):
        if self._ws and self.is_connected:
            try:
                self._ws.send(json.dumps(obj))
            except Exception as e:
                print(f"[WARN] send error: {e}")

    def close(self):
        if self._ws:
            self._ws.close()


# ─── Clustering Node ──────────────────────────────────────────────────────────

class ClusteringNode:
    """
    Connects to Jetson rosbridge via WebSocket, subscribes to the foreground
    point cloud, runs Euclidean clustering, and publishes detection results.
    Runs entirely on the Legion laptop without requiring a local ROS2 install.
    """

    def __init__(self,
                 jetson_ip: str,
                 port: int,
                 input_topic: str,
                 cluster_tol: float,
                 min_points: int,
                 max_points: int,
                 max_persons: int):

        self.cluster_tol = cluster_tol
        self.min_points = min_points
        self.max_points = max_points
        self.max_persons = max_persons

        self.frame_count = 0
        self.last_detections = []

        print(f"Connecting to rosbridge at {jetson_ip}:{port} ...")
        self.client = RosBridgeClient(host=jetson_ip, port=port)
        if not self.client.connect(timeout=10.0):
            raise RuntimeError(
                f"Failed to connect to rosbridge at {jetson_ip}:{port}. "
                "Is rosbridge running? Check: "
                "ros2 launch rosbridge_server rosbridge_websocket_launch.xml")
        print(f"Connected: {self.client.is_connected}")

        # Subscribe to foreground point cloud
        # Use ROS2 fully-qualified type name (sensor_msgs/msg/PointCloud2)
        # to avoid type-inference errors on rosbridge 2.0.6.
        self.client.subscribe(
            topic=input_topic,
            msg_type="sensor_msgs/msg/PointCloud2",
            callback=self._callback,
            throttle_rate=0,
        )

        # Advertise output topics
        self.client.advertise(
            "/detection_boxes",
            "visualization_msgs/msg/MarkerArray",
        )
        self.client.advertise(
            "/detection_centers",
            "sensor_msgs/msg/PointCloud2",
        )

        self._input_topic = input_topic
        print("Clustering node ready")
        print(f"  Input:  {input_topic}")
        print(f"  Output: /detection_boxes, /detection_centers")
        print(f"  Params: tol={cluster_tol}m, "
              f"min={min_points}pts, max={max_points}pts")

    def _callback(self, msg: dict):
        t0 = time.time()

        pts = self._decode_pointcloud2(msg)
        if pts is None or len(pts) < self.min_points:
            return

        clusters = euclidean_clustering(
            pts,
            cluster_tol=self.cluster_tol,
            min_points=self.min_points,
            max_points=self.max_points,
        )
        clusters = clusters[:self.max_persons]

        detections = [cluster_to_bbox(c) for c in clusters]
        self.last_detections = detections

        header = msg.get("header", {})
        self._publish_markers(detections, header)
        self._publish_centers(detections, header)

        dt = time.time() - t0
        self.frame_count += 1
        if self.frame_count % 20 == 0:
            print(f"[frame {self.frame_count:>4}]  "
                  f"in={len(pts):>4} -> {len(detections):>2} persons | "
                  f"{dt*1000:.0f}ms")

    def _decode_pointcloud2(self, msg: dict) -> np.ndarray:
        """Extract xyz from a rosbridge-serialized PointCloud2 message."""
        try:
            data_b64 = msg.get("data", "")
            if not data_b64:
                return None
            raw = base64.b64decode(data_b64)

            fields = msg.get("fields", [])
            field_map = {f["name"]: f["offset"] for f in fields}
            point_step = msg.get("point_step", 16)
            width = msg.get("width", 0)

            if width == 0 or "x" not in field_map:
                return None

            x_off = field_map["x"]
            y_off = field_map["y"]
            z_off = field_map["z"]

            pts = []
            for i in range(width):
                base = i * point_step
                x = struct.unpack_from("<f", raw, base + x_off)[0]
                y = struct.unpack_from("<f", raw, base + y_off)[0]
                z = struct.unpack_from("<f", raw, base + z_off)[0]
                if np.isfinite(x) and np.isfinite(y) and np.isfinite(z):
                    pts.append((x, y, z))

            return np.array(pts, dtype=np.float32) if pts else None

        except Exception as e:
            print(f"[WARN] decode error: {e}")
            return None

    def _publish_markers(self, detections: List[dict], header: dict):
        """Publish bounding box markers (displayed as wireframe cubes in Foxglove)."""
        markers = []
        stamp = header.get("stamp", {"sec": 0, "nanosec": 0})
        frame_id = header.get("frame_id", "livox_frame")

        for i, det in enumerate(detections):
            c = det["center"].tolist()
            s = det["size"].tolist()
            markers.append({
                "header": {"stamp": stamp, "frame_id": frame_id},
                "ns": "persons",
                "id": i,
                "type": 1,    # CUBE
                "action": 0,  # ADD
                "pose": {
                    "position": {"x": float(c[0]),
                                 "y": float(c[1]),
                                 "z": float(c[2])},
                    "orientation": {"x": 0.0, "y": 0.0,
                                    "z": 0.0, "w": 1.0},
                },
                "scale": {"x": max(float(s[0]), 0.3),
                          "y": max(float(s[1]), 0.3),
                          "z": max(float(s[2]), 0.1)},
                "color": {"r": 1.0, "g": 0.5, "b": 0.0, "a": 0.4},
                "lifetime": {"sec": 0, "nanosec": 300000000},
            })

        # Delete markers from previous frame that are no longer needed
        for i in range(len(detections), len(detections) + 5):
            markers.append({
                "header": {"stamp": stamp, "frame_id": frame_id},
                "ns": "persons",
                "id": i,
                "type": 1,
                "action": 2,  # DELETE
                "pose": {
                    "position": {"x": 0.0, "y": 0.0, "z": 0.0},
                    "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
                },
                "scale": {"x": 0.1, "y": 0.1, "z": 0.1},
                "color": {"r": 0.0, "g": 0.0, "b": 0.0, "a": 0.0},
                "lifetime": {"sec": 0, "nanosec": 0},
            })

        self.client.publish("/detection_boxes", {"markers": markers})

    def _publish_centers(self, detections: List[dict], header: dict):
        """Publish centroid point cloud for AB3DMOT tracking input."""
        if not detections:
            return

        stamp = header.get("stamp", {"sec": 0, "nanosec": 0})
        frame_id = header.get("frame_id", "livox_frame")

        raw = b""
        for det in detections:
            c = det["center"]
            raw += struct.pack("<fff",
                               float(c[0]), float(c[1]), float(c[2]))

        msg = {
            "header": {"stamp": stamp, "frame_id": frame_id},
            "height": 1,
            "width": len(detections),
            "fields": [
                {"name": "x", "offset": 0, "datatype": 7, "count": 1},
                {"name": "y", "offset": 4, "datatype": 7, "count": 1},
                {"name": "z", "offset": 8, "datatype": 7, "count": 1},
            ],
            "is_bigendian": False,
            "point_step": 12,
            "row_step": 12 * len(detections),
            "data": base64.b64encode(raw).decode("ascii"),
            "is_dense": True,
        }
        self.client.publish("/detection_centers", msg)

    def spin(self):
        """Block until Ctrl+C."""
        print("\nRunning... Press Ctrl+C to stop\n")
        try:
            while self.client.is_connected:
                time.sleep(0.1)
        except KeyboardInterrupt:
            pass
        finally:
            self.client.close()
            print(f"\nStopped after {self.frame_count} frames")


# ─── Standalone Algorithm Test (no ROS required) ─────────────────────────────

def run_test():
    """Validate the clustering algorithm without ROS or network connectivity."""
    print("=" * 55)
    print("TEST: Euclidean Clustering Algorithm")
    print("=" * 55)
    rng = np.random.default_rng(0)

    def make_person(cx, cy, cz=-1.7, n=50):
        return np.column_stack([
            rng.uniform(cx - 0.25, cx + 0.25, n),
            rng.uniform(cy - 0.25, cy + 0.25, n),
            rng.uniform(cz - 0.3, cz, n),
        ]).astype(np.float32)

    persons = [
        make_person(0.0, 0.0),    # person 1: origin
        make_person(1.5, 0.5),    # person 2: 1.5 m away
        make_person(-1.0, 1.2),   # person 3: diagonal
    ]

    # Residual background noise that BG removal didn't fully eliminate
    noise = np.column_stack([
        rng.uniform(-3, 3, 20),
        rng.uniform(-3, 3, 20),
        rng.uniform(-2.8, -2.5, 20),
    ]).astype(np.float32)

    pts = np.vstack(persons + [noise])
    print(f"Input: {len(pts)} points "
          f"(3 persons x 50pts + 20 noise)")

    clusters = euclidean_clustering(
        pts,
        cluster_tol=0.4,
        min_points=8,
        max_points=800,
    )

    print(f"Detected: {len(clusters)} clusters")
    for i, c in enumerate(clusters):
        bbox = cluster_to_bbox(c)
        print(f"  Cluster {i+1}: {len(c):>3} pts | "
              f"center=({bbox['center'][0]:.2f}, "
              f"{bbox['center'][1]:.2f}, "
              f"{bbox['center'][2]:.2f}) | "
              f"size=({bbox['size'][0]:.2f}, "
              f"{bbox['size'][1]:.2f})")

    if len(clusters) == 3:
        print("\n✓ TEST PASSED: 3 persons correctly detected")
    else:
        print(f"\n✗ TEST FAILED: expected 3, got {len(clusters)}")

    return len(clusters) == 3


# ─── main ─────────────────────────────────────────────────────────────────────

if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Euclidean Clustering Node for Person Detection",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    parser.add_argument("--jetson_ip", type=str,
                        default="172.26.42.167",
                        help="Jetson IP address")
    parser.add_argument("--port", type=int, default=9090,
                        help="rosbridge websocket port (default 9090)")
    parser.add_argument("--topic", type=str,
                        default="/livox/lidar_foreground",
                        help="Input foreground point cloud topic")

    parser.add_argument("--cluster_tol", type=float, default=0.4,
                        help="""
Euclidean clustering distance tolerance in metres (default 0.4).
Points within this distance join the same cluster.
  Too small (<0.25): one person split into multiple clusters
  Too large (>0.6):  adjacent people merged into one cluster
Recommended for 3 m ceiling view: 0.35-0.45
""")
    parser.add_argument("--min_points", type=int, default=8,
                        help="""
Minimum cluster size (default 8). Clusters below this are discarded as noise.
At 3 m height a person yields ~30-80 points. 8 is conservative.
Raise to 15-20 if false positives are a problem.
""")
    parser.add_argument("--max_points", type=int, default=800,
                        help="""
Maximum cluster size (default 800). Clusters above this are discarded
(large objects such as carts or pillars).
A normal person at 3 m height has <150 points; 800 is permissive.
""")
    parser.add_argument("--max_persons", type=int, default=20,
                        help="Maximum detections to output (default 20)")
    parser.add_argument("--test", action="store_true",
                        help="Run algorithm self-test without ROS/network")
    args = parser.parse_args()

    if args.test:
        run_test()
    else:
        if not WEBSOCKET_AVAILABLE:
            print("ERROR: websocket-client not installed.")
            print("Install: pip install websocket-client")
            print("Or run --test to verify the algorithm works.")
            exit(1)
        node = ClusteringNode(
            jetson_ip=args.jetson_ip,
            port=args.port,
            input_topic=args.topic,
            cluster_tol=args.cluster_tol,
            min_points=args.min_points,
            max_points=args.max_points,
            max_persons=args.max_persons,
        )
        node.spin()
