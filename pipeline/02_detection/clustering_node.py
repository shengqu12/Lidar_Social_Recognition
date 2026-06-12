#!/usr/bin/env python3
"""
Euclidean Clustering Node - Person Detection
=============================================
Receives foreground point clouds from background removal,
applies ROI filtering, frame accumulation, Euclidean clustering,
cluster shape validation, and publishes per-person 3D bounding boxes
and centroid positions.

Pipeline position:
  /livox/lidar
      -> [statistical_bg_node]
      -> /livox/lidar_foreground
      -> [clustering_node]  <- this file
      -> /detection_boxes   (MarkerArray, Foxglove visualization)
      -> /detection_centers (PointCloud2, tracking input)

Importable interface (used by tracking_node.py):
  from clustering_node import detect, apply_roi, is_valid_person_cluster

Launch (Legion laptop, connects to Jetson rosbridge via websocket):
  conda activate livox
  python3 clustering_node.py --config nodes_config.yaml --node node1
"""

import argparse
import base64
import json
import struct
import threading
import time
from collections import deque
from typing import List, Optional

import numpy as np
from scipy.spatial import KDTree


# ─── ROI Filter ───────────────────────────────────────────────────────────────

def apply_roi(pts: np.ndarray, roi_cfg: dict) -> np.ndarray:
    """
    Crop points to the axis-aligned ROI box, then remove any exclusion zones.

    exclusion_zones (optional list in roi_cfg):
        [{cx, cy, radius}, ...]  — circular XY masks for known static objects
        that the background model doesn't cover (furniture, equipment).
    """
    if not roi_cfg.get("enabled", False) or len(pts) == 0:
        return pts
    mask = (
        (pts[:, 0] >= roi_cfg["x_min"]) & (pts[:, 0] <= roi_cfg["x_max"]) &
        (pts[:, 1] >= roi_cfg["y_min"]) & (pts[:, 1] <= roi_cfg["y_max"]) &
        (pts[:, 2] >= roi_cfg["z_min"]) & (pts[:, 2] <= roi_cfg["z_max"])
    )
    for zone in roi_cfg.get("exclusion_zones", []):
        cx, cy, r = float(zone["cx"]), float(zone["cy"]), float(zone["radius"])
        dist_sq = (pts[:, 0] - cx) ** 2 + (pts[:, 1] - cy) ** 2
        mask &= dist_sq > r * r
    return pts[mask]


# ─── Core Clustering Algorithm ────────────────────────────────────────────────

def euclidean_clustering(
    pts: np.ndarray,
    cluster_tol: float = 0.4,
    min_points: int = 8,
    max_points: int = 800,
) -> List[np.ndarray]:
    """
    BFS Euclidean clustering on a foreground point cloud.

    Args:
        pts:          (N, 3) foreground points
        cluster_tol:  max intra-cluster distance in metres
        min_points:   discard clusters with fewer points
        max_points:   discard clusters with more points

    Returns:
        list of (M, 3) arrays, sorted largest-first
    """
    if len(pts) < min_points:
        return []

    tree = KDTree(pts)
    visited = np.zeros(len(pts), dtype=bool)
    clusters = []

    for seed_idx in range(len(pts)):
        if visited[seed_idx]:
            continue

        cluster_indices = []
        queue = [seed_idx]
        visited[seed_idx] = True

        while queue:
            current = queue.pop(0)
            cluster_indices.append(current)
            for nb in tree.query_ball_point(pts[current], cluster_tol):
                if not visited[nb]:
                    visited[nb] = True
                    queue.append(nb)

        if min_points <= len(cluster_indices) <= max_points:
            clusters.append(pts[cluster_indices])

    clusters.sort(key=lambda c: len(c), reverse=True)
    return clusters


def cluster_to_bbox(cluster: np.ndarray) -> dict:
    """
    Compute 3D bounding box and centroid for one cluster.

    Returns:
        center, size, bbox_min, bbox_max, n_points
    """
    bbox_min = cluster.min(axis=0)
    bbox_max = cluster.max(axis=0)
    center = (bbox_min + bbox_max) / 2.0
    size = bbox_max - bbox_min
    return {
        "center":   center,
        "size":     size,
        "bbox_min": bbox_min,
        "bbox_max": bbox_max,
        "n_points": len(cluster),
    }


# ─── Cluster Shape Validation ─────────────────────────────────────────────────

def is_valid_person_cluster(bbox: dict, filter_cfg: dict) -> bool:
    """
    Reject clusters that don't match expected human geometry:
      - Both XY extents must be within [min_xy_size, max_xy_size]
      - XY aspect ratio must be <= max_aspect_ratio  (kills wall lines)
    """
    sx, sy = float(bbox["size"][0]), float(bbox["size"][1])
    min_s = float(filter_cfg.get("min_xy_size", 0.15))
    max_s = float(filter_cfg.get("max_xy_size", 1.2))
    max_ar = float(filter_cfg.get("max_aspect_ratio", 4.0))

    if sx < min_s or sy < min_s:
        return False
    if sx > max_s or sy > max_s:
        return False
    aspect = max(sx, sy) / (min(sx, sy) + 1e-6)
    if aspect > max_ar:
        return False
    return True


# ─── High-level detect() for use by tracking_node ────────────────────────────

def detect(
    pts: np.ndarray,
    cluster_tol: float = 0.4,
    min_points: int = 8,
    max_points: int = 800,
    max_persons: int = 10,
    roi_cfg: Optional[dict] = None,
    filter_cfg: Optional[dict] = None,
) -> List[dict]:
    """
    Full detection pipeline on a point array:
      ROI filter → Euclidean clustering → bbox → shape filter

    Args:
        pts:          (N, 3) foreground points (may be from multiple frames)
        roi_cfg:      dict from nodes_config roi section (or None to skip)
        filter_cfg:   dict from nodes_config cluster_filter section (or None to skip)

    Returns:
        list of bbox dicts, at most max_persons entries
    """
    if roi_cfg:
        pts = apply_roi(pts, roi_cfg)

    if len(pts) < min_points:
        return []

    clusters = euclidean_clustering(pts, cluster_tol, min_points, max_points)

    detections = []
    for c in clusters:
        bbox = cluster_to_bbox(c)
        if filter_cfg and not is_valid_person_cluster(bbox, filter_cfg):
            continue
        detections.append(bbox)
        if len(detections) >= max_persons:
            break

    return detections


# ─── RosBridge WebSocket Client ───────────────────────────────────────────────

try:
    import websocket
    WEBSOCKET_AVAILABLE = True
except ImportError:
    WEBSOCKET_AVAILABLE = False
    print("[WARN] websocket-client not installed. Run: pip install websocket-client")


class RosBridgeClient:
    """
    Minimal rosbridge v2 WebSocket client.
    Uses websocket-client directly to avoid roslibpy blocking issues.
    """

    def __init__(self, host: str, port: int):
        self.url = f"ws://{host}:{port}"
        self._topic_callbacks = {}
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
        return self._connected_event.wait(timeout=timeout)

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
        self._send({"op": "advertise", "topic": topic, "type": msg_type})

    def publish(self, topic: str, msg: dict):
        self._send({"op": "publish", "topic": topic, "msg": msg})

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
    Connects to Jetson rosbridge, subscribes to the foreground point cloud,
    runs detection (ROI → accumulation → clustering → shape filter),
    and publishes /detection_boxes and /detection_centers.
    """

    def __init__(self,
                 jetson_ip: str,
                 port: int,
                 input_topic: str,
                 cluster_tol: float,
                 min_points: int,
                 max_points: int,
                 max_persons: int,
                 accum_frames: int = 1,
                 bbox_alpha: float = 0.4,
                 bbox_color: List[float] = None,
                 roi_cfg: Optional[dict] = None,
                 filter_cfg: Optional[dict] = None):

        self.cluster_tol = cluster_tol
        self.min_points = min_points
        self.max_points = max_points
        self.max_persons = max_persons
        self.accum_frames = max(1, accum_frames)
        self.bbox_alpha = bbox_alpha
        self.bbox_color = bbox_color if bbox_color is not None else [1.0, 0.5, 0.0]
        self.roi_cfg = roi_cfg or {}
        self.filter_cfg = filter_cfg or {}

        self.frame_count = 0
        self._rejected_count = 0
        self._frame_buf: deque = deque(maxlen=self.accum_frames)

        print(f"Connecting to rosbridge at {jetson_ip}:{port} ...")
        self.client = RosBridgeClient(host=jetson_ip, port=port)
        if not self.client.connect(timeout=10.0):
            raise RuntimeError(
                f"Failed to connect to rosbridge at {jetson_ip}:{port}.")
        print(f"Connected: {self.client.is_connected}")

        self.client.subscribe(
            topic=input_topic,
            msg_type="sensor_msgs/msg/PointCloud2",
            callback=self._callback,
            throttle_rate=0,
        )
        self.client.advertise("/detection_boxes",
                              "visualization_msgs/msg/MarkerArray")
        self.client.advertise("/detection_centers",
                              "sensor_msgs/msg/PointCloud2")

        self._input_topic = input_topic
        print("Clustering node ready")
        print(f"  Input:  {input_topic}")
        print(f"  Output: /detection_boxes, /detection_centers")
        print(f"  Params: tol={cluster_tol}m  min={min_points}  "
              f"max={max_points}  accum={accum_frames}frames")
        if roi_cfg and roi_cfg.get("enabled"):
            print(f"  ROI:    x[{roi_cfg['x_min']},{roi_cfg['x_max']}]  "
                  f"y[{roi_cfg['y_min']},{roi_cfg['y_max']}]")

    # ── callback ──────────────────────────────────────────────────────────────

    def _callback(self, msg: dict):
        t0 = time.time()

        pts = self._decode_pointcloud2(msg)
        if pts is None or len(pts) == 0:
            return

        # ROI first (on raw frame, before accumulation — cheaper than filtering merged cloud)
        if self.roi_cfg.get("enabled"):
            pts = apply_roi(pts, self.roi_cfg)

        self._frame_buf.append(pts)

        # Skip early frames until buffer reaches accum_frames
        if len(self._frame_buf) < self.accum_frames:
            return

        merged = np.vstack(list(self._frame_buf))

        clusters = euclidean_clustering(
            merged,
            cluster_tol=self.cluster_tol,
            min_points=self.min_points,
            max_points=self.max_points,
        )

        detections = []
        rejected = 0
        for c in clusters:
            bbox = cluster_to_bbox(c)
            if self.filter_cfg and not is_valid_person_cluster(bbox, self.filter_cfg):
                rejected += 1
                continue
            detections.append(bbox)
            if len(detections) >= self.max_persons:
                break

        self._rejected_count += rejected

        header = msg.get("header", {})
        self._publish_markers(detections, header)
        self._publish_centers(detections, header)

        dt = time.time() - t0
        self.frame_count += 1
        if self.frame_count % 20 == 0:
            print(f"[frame {self.frame_count:>4}]  "
                  f"raw={len(pts):>4} merged={len(merged):>5} -> "
                  f"{len(detections):>2} persons "
                  f"(rejected {self._rejected_count} total) | {dt*1000:.0f}ms")
            self._rejected_count = 0

    # ── decode ────────────────────────────────────────────────────────────────

    def _decode_pointcloud2(self, msg: dict) -> Optional[np.ndarray]:
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
                base_i = i * point_step
                x = struct.unpack_from("<f", raw, base_i + x_off)[0]
                y = struct.unpack_from("<f", raw, base_i + y_off)[0]
                z = struct.unpack_from("<f", raw, base_i + z_off)[0]
                if np.isfinite(x) and np.isfinite(y) and np.isfinite(z):
                    pts.append((x, y, z))

            return np.array(pts, dtype=np.float32) if pts else None

        except Exception as e:
            print(f"[WARN] decode error: {e}")
            return None

    # ── publish ───────────────────────────────────────────────────────────────

    def _publish_markers(self, detections: List[dict], header: dict):
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
                    "position": {"x": float(c[0]), "y": float(c[1]),
                                 "z": float(c[2])},
                    "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
                },
                "scale": {"x": max(float(s[0]), 0.3),
                          "y": max(float(s[1]), 0.3),
                          "z": max(float(s[2]), 0.1)},
                "color": {"r": float(self.bbox_color[0]),
                          "g": float(self.bbox_color[1]),
                          "b": float(self.bbox_color[2]),
                          "a": float(self.bbox_alpha)},
                "lifetime": {"sec": 0, "nanosec": 300000000},
            })

        # Delete stale markers from previous frames
        for i in range(len(detections), len(detections) + 5):
            markers.append({
                "header": {"stamp": stamp, "frame_id": frame_id},
                "ns": "persons", "id": i,
                "type": 1, "action": 2,  # DELETE
                "pose": {"position": {"x": 0.0, "y": 0.0, "z": 0.0},
                         "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}},
                "scale": {"x": 0.1, "y": 0.1, "z": 0.1},
                "color": {"r": 0.0, "g": 0.0, "b": 0.0, "a": 0.0},
                "lifetime": {"sec": 0, "nanosec": 0},
            })

        self.client.publish("/detection_boxes", {"markers": markers})

    def _publish_centers(self, detections: List[dict], header: dict):
        if not detections:
            return
        stamp = header.get("stamp", {"sec": 0, "nanosec": 0})
        frame_id = header.get("frame_id", "livox_frame")

        raw = b""
        for det in detections:
            c = det["center"]
            raw += struct.pack("<fff", float(c[0]), float(c[1]), float(c[2]))

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
        print("\nRunning... Press Ctrl+C to stop\n")
        try:
            while self.client.is_connected:
                time.sleep(0.1)
        except KeyboardInterrupt:
            pass
        finally:
            self.client.close()
            print(f"\nStopped after {self.frame_count} frames")


# ─── Standalone Algorithm Test ────────────────────────────────────────────────

def run_test():
    print("=" * 55)
    print("TEST: Detection pipeline (ROI + clustering + shape filter)")
    print("=" * 55)
    rng = np.random.default_rng(0)

    def make_person(cx, cy, cz=-1.7, n=50):
        return np.column_stack([
            rng.uniform(cx - 0.25, cx + 0.25, n),
            rng.uniform(cy - 0.25, cy + 0.25, n),
            rng.uniform(cz - 0.3, cz, n),
        ]).astype(np.float32)

    def make_wall(x_range, y_range, n=100):
        return np.column_stack([
            rng.uniform(*x_range, n),
            rng.uniform(*y_range, n),
            rng.uniform(-2.0, -1.8, n),
        ]).astype(np.float32)

    persons = [make_person(1.5, -1.5), make_person(3.0, -2.5), make_person(-1.0, 1.2)]
    wall1 = make_wall((-1.5, -0.5), (-5.0, 1.0))   # thin wall strip: excluded by ROI
    wall2 = make_wall((0.5, 6.0), (-0.5, 0.5))      # elongated: excluded by aspect_ratio
    pts = np.vstack(persons + [wall1, wall2])

    roi_cfg = {"enabled": True, "x_min": 0.3, "x_max": 8.0,
               "y_min": -5.0, "y_max": -0.7, "z_min": -2.5, "z_max": -0.5}
    filter_cfg = {"min_xy_size": 0.15, "max_xy_size": 1.2, "max_aspect_ratio": 4.0}

    detections = detect(pts, cluster_tol=0.4, min_points=8,
                        roi_cfg=roi_cfg, filter_cfg=filter_cfg)

    print(f"Input: {len(pts)} pts  |  Detected: {len(detections)} persons")
    for i, d in enumerate(detections):
        c = d["center"]
        s = d["size"]
        print(f"  #{i+1}: center=({c[0]:.2f},{c[1]:.2f}) "
              f"size=({s[0]:.2f}x{s[1]:.2f})")

    passed = len(detections) == 2  # person at (-1.0,1.2) excluded by ROI; wall2 by aspect
    print(f"\n{'✓ TEST PASSED' if passed else '✗ TEST FAILED'}: "
          f"expected 2 persons within ROI, got {len(detections)}")
    return passed


# ─── Config loader ────────────────────────────────────────────────────────────

def _load_node_config(config_path: str, node_name: str) -> dict:
    import yaml
    with open(config_path) as f:
        data = yaml.safe_load(f)
    nodes = data.get("nodes", {})
    if node_name not in nodes:
        raise ValueError(
            f"Node '{node_name}' not found in {config_path}. "
            f"Available: {list(nodes.keys())}")
    return nodes[node_name]


# ─── main ─────────────────────────────────────────────────────────────────────

if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Euclidean Clustering Node for Person Detection",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    parser.add_argument("--config", type=str, default=None)
    parser.add_argument("--node",   type=str, default="node1")
    parser.add_argument("--jetson_ip",   type=str,   default=None)
    parser.add_argument("--port",        type=int,   default=None)
    parser.add_argument("--topic",       type=str,   default=None)
    parser.add_argument("--cluster_tol", type=float, default=None)
    parser.add_argument("--min_points",  type=int,   default=None)
    parser.add_argument("--max_points",  type=int,   default=None)
    parser.add_argument("--max_persons", type=int,   default=None)
    parser.add_argument("--accum_frames",type=int,   default=None)
    parser.add_argument("--bbox_alpha",  type=float, default=None)
    parser.add_argument("--bbox_color",  type=float, nargs=3, default=None,
                        metavar=("R", "G", "B"))
    parser.add_argument("--test", action="store_true",
                        help="Run algorithm self-test without ROS/network")
    args = parser.parse_args()

    if args.test:
        run_test()
    else:
        jetson_ip = args.jetson_ip
        port      = args.port
        topic     = args.topic

        cfg_clustering = {}
        roi_cfg    = {}
        filter_cfg = {}

        if args.config is not None:
            try:
                node_cfg = _load_node_config(args.config, args.node)
            except (FileNotFoundError, ValueError) as e:
                print(f"Config error: {e}")
                exit(1)
            if jetson_ip is None:
                jetson_ip = node_cfg.get("jetson_ip", "172.26.42.167")
            if port is None:
                port = int(node_cfg.get("rosbridge_port", 9090))
            if topic is None:
                topic = node_cfg.get("foreground_topic", "/livox/lidar_foreground")
            cfg_clustering = node_cfg.get("clustering", {})
            roi_cfg    = node_cfg.get("roi", {})
            filter_cfg = node_cfg.get("cluster_filter", {})

        if jetson_ip is None:
            jetson_ip = "172.26.42.167"
        if port is None:
            port = 9090
        if topic is None:
            topic = "/livox/lidar_foreground"

        cluster_tol  = args.cluster_tol   if args.cluster_tol   is not None else cfg_clustering.get("cluster_tol",   0.4)
        min_points   = args.min_points    if args.min_points    is not None else cfg_clustering.get("min_points",    8)
        max_points   = args.max_points    if args.max_points    is not None else cfg_clustering.get("max_points",    800)
        max_persons  = args.max_persons   if args.max_persons   is not None else cfg_clustering.get("max_persons",   10)
        accum_frames = args.accum_frames  if args.accum_frames  is not None else cfg_clustering.get("accum_frames",  1)
        bbox_alpha   = args.bbox_alpha    if args.bbox_alpha    is not None else cfg_clustering.get("bbox_alpha",    0.4)
        bbox_color   = args.bbox_color    if args.bbox_color    is not None else cfg_clustering.get("bbox_color",    [1.0, 0.5, 0.0])

        print(f"Params: tol={cluster_tol}m  min={min_points}  "
              f"max={max_points}  max_persons={max_persons}  accum={accum_frames}")

        if not WEBSOCKET_AVAILABLE:
            print("ERROR: pip install websocket-client")
            exit(1)

        node = ClusteringNode(
            jetson_ip=jetson_ip,
            port=port,
            input_topic=topic,
            cluster_tol=cluster_tol,
            min_points=min_points,
            max_points=max_points,
            max_persons=max_persons,
            accum_frames=accum_frames,
            bbox_alpha=bbox_alpha,
            bbox_color=bbox_color,
            roi_cfg=roi_cfg,
            filter_cfg=filter_cfg,
        )
        node.spin()
