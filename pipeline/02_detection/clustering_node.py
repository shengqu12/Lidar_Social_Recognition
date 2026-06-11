#!/usr/bin/env python3
"""
Euclidean Clustering Node — Person Detection
=============================================
接收 background removal 的前景点云，
用 Euclidean clustering 把点分成独立的人体 cluster，
输出每个人的 3D bounding box（用于 Foxglove 可视化）
和质心位置（用于 AB3DMOT tracking）。

Pipeline 位置:
  /livox/lidar
      → [statistical_bg_node]
      → /livox/lidar_foreground
      → [clustering_node]  ← 这个文件
      → /detection_boxes   (MarkerArray, Foxglove 可视化)
      → /detection_centers (PointCloud2, 给 tracking 用)

启动（Legion 本地，通过 roslibpy 连 Jetson rosbridge）:
  conda activate livox
  python3 clustering_node.py \
      --jetson_ip 172.26.42.167 \
      --topic /livox/lidar_foreground \
      --cluster_tol 0.4 \
      --min_points 8 \
      --max_points 800

参数说明见下方 argparse 的 help 字段。

算法原理:
  Euclidean clustering（也叫 Euclidean cluster extraction）:
  把空间中距离小于 cluster_tol 的点归为同一个 cluster。
  本质是 BFS（广度优先搜索）+ KDTree 近邻查找。
  对天花板俯视场景非常合适：人体从上往下看是椭圆 blob，
  cluster_tol 设 0.4m 能把一个人身上的点连起来，
  同时不会把相邻两人（距离通常 >0.5m）合并。

参考:
  Yamaguchi et al. 2024 — Euclidean clustering for indoor LiDAR tracking
  Gómez et al. 2023 — Euclidean distance-based segmentation
  PCL EuclideanClusterExtraction (同原理的 C++ 实现)
"""

import argparse
import time
import threading
from typing import List, Tuple
import numpy as np
from scipy.spatial import KDTree


# ─── 核心算法（不依赖 ROS，可独立测试）──────────────────────────────────────

def euclidean_clustering(
    pts: np.ndarray,
    cluster_tol: float = 0.4,
    min_points: int = 8,
    max_points: int = 800,
) -> List[np.ndarray]:
    """
    对前景点云做 Euclidean clustering，返回每个 cluster 的点云。

    Args:
        pts:          np.ndarray (N, 3)，前景点云（已做 BG removal）
        cluster_tol:  两点距离 <= 此值则归为同一 cluster（单位：米）
                      天花板3m俯视场景推荐 0.35~0.45m
        min_points:   cluster 最少点数，过滤噪声点
                      3m高度下一个人约 30~80 点，min 设 8 偏保守
        max_points:   cluster 最多点数，过滤大面积误检（如移动推车）

    Returns:
        clusters: list of np.ndarray，每个元素是一个 cluster 的点 (M, 3)
                  按点数从多到少排序
    """
    if len(pts) < min_points:
        return []

    tree = KDTree(pts)
    visited = np.zeros(len(pts), dtype=bool)
    clusters = []

    for seed_idx in range(len(pts)):
        if visited[seed_idx]:
            continue

        # BFS 扩展这个 cluster
        cluster_indices = []
        queue = [seed_idx]
        visited[seed_idx] = True

        while queue:
            current = queue.pop(0)
            cluster_indices.append(current)
            # 找半径内所有邻居
            neighbors = tree.query_ball_point(
                pts[current], cluster_tol)
            for nb in neighbors:
                if not visited[nb]:
                    visited[nb] = True
                    queue.append(nb)

        # 大小过滤
        if min_points <= len(cluster_indices) <= max_points:
            clusters.append(pts[cluster_indices])

    # 按点数从多到少排序（点多的 cluster 更可能是人）
    clusters.sort(key=lambda c: len(c), reverse=True)
    return clusters


def cluster_to_bbox(cluster: np.ndarray) -> dict:
    """
    计算一个 cluster 的 3D bounding box 和质心。

    Returns dict:
        center:   np.ndarray (3,) — 质心 xyz
        size:     np.ndarray (3,) — 长宽高
        bbox_min: np.ndarray (3,) — bbox 最小角
        bbox_max: np.ndarray (3,) — bbox 最大角
        n_points: int             — 点数
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


# ─── roslibpy 连接（Legion 本地 → Jetson rosbridge）────────────────────────

try:
    import roslibpy
    ROSLIBPY_AVAILABLE = True
except ImportError:
    ROSLIBPY_AVAILABLE = False
    print("[WARN] roslibpy not installed. "
          "Run: pip install roslibpy")


class ClusteringNode:
    """
    通过 roslibpy 连接 Jetson rosbridge，
    订阅前景点云，做 clustering，发布检测结果。
    在 Legion 本地运行，不需要 ROS2 环境。
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
        self.last_detections = []  # 供外部读取

        # 连接 rosbridge
        print(f"Connecting to rosbridge at {jetson_ip}:{port} ...")
        self.client = roslibpy.Ros(host=jetson_ip, port=port)
        self.client.run()
        print(f"Connected: {self.client.is_connected}")

        # 订阅前景点云
        self.sub = roslibpy.Topic(
            self.client, input_topic, 'sensor_msgs/PointCloud2')
        self.sub.subscribe(self._callback)

        # 发布检测结果（MarkerArray 用于 Foxglove 可视化）
        self.pub_markers = roslibpy.Topic(
            self.client,
            '/detection_boxes',
            'visualization_msgs/MarkerArray'
        )

        # 发布质心点云（给后续 tracking 用）
        self.pub_centers = roslibpy.Topic(
            self.client,
            '/detection_centers',
            'sensor_msgs/PointCloud2'
        )

        print(f"Clustering node ready")
        print(f"  Input:  {input_topic}")
        print(f"  Output: /detection_boxes, /detection_centers")
        print(f"  Params: tol={cluster_tol}m, "
              f"min={min_points}pts, max={max_points}pts")

    def _callback(self, msg):
        t0 = time.time()

        # 解码 PointCloud2
        pts = self._decode_pointcloud2(msg)
        if pts is None or len(pts) < self.min_points:
            return

        # Euclidean clustering
        clusters = euclidean_clustering(
            pts,
            cluster_tol=self.cluster_tol,
            min_points=self.min_points,
            max_points=self.max_points,
        )

        # 限制最大人数（避免噪声爆炸）
        clusters = clusters[:self.max_persons]

        # 计算每个 cluster 的 bbox
        detections = [cluster_to_bbox(c) for c in clusters]
        self.last_detections = detections

        # 发布
        header = msg.get('header', {})
        self._publish_markers(detections, header)
        self._publish_centers(detections, header)

        dt = time.time() - t0
        self.frame_count += 1
        if self.frame_count % 20 == 0:
            print(f"[frame {self.frame_count:>4}] "
                  f"in={len(pts):>4}pts → "
                  f"{len(detections):>2} persons | "
                  f"{dt*1000:.0f}ms")

    def _decode_pointcloud2(self, msg) -> np.ndarray:
        """从 roslibpy PointCloud2 msg 提取 xyz"""
        try:
            import base64, struct
            data_b64 = msg.get('data', '')
            if not data_b64:
                return None
            raw = base64.b64decode(data_b64)

            # 读取 field offsets
            fields = msg.get('fields', [])
            field_map = {f['name']: f['offset'] for f in fields}
            point_step = msg.get('point_step', 16)
            width = msg.get('width', 0)

            if width == 0 or 'x' not in field_map:
                return None

            x_off = field_map['x']
            y_off = field_map['y']
            z_off = field_map['z']

            pts = []
            for i in range(width):
                base = i * point_step
                x = struct.unpack_from('<f', raw, base + x_off)[0]
                y = struct.unpack_from('<f', raw, base + y_off)[0]
                z = struct.unpack_from('<f', raw, base + z_off)[0]
                if np.isfinite(x) and np.isfinite(y) and np.isfinite(z):
                    pts.append((x, y, z))

            return np.array(pts, dtype=np.float32) if pts else None

        except Exception as e:
            print(f"[WARN] decode error: {e}")
            return None

    def _publish_markers(self, detections: List[dict], header: dict):
        """发布 bounding box markers（Foxglove 里显示为线框方块）"""
        markers = []
        stamp = header.get('stamp', {'secs': 0, 'nsecs': 0})
        frame_id = header.get('frame_id', 'livox_frame')

        for i, det in enumerate(detections):
            c = det['center'].tolist()
            s = det['size'].tolist()

            marker = {
                'header': {
                    'stamp': stamp,
                    'frame_id': frame_id,
                },
                'ns': 'persons',
                'id': i,
                'type': 1,        # CUBE
                'action': 0,      # ADD
                'pose': {
                    'position': {'x': float(c[0]),
                                 'y': float(c[1]),
                                 'z': float(c[2])},
                    'orientation': {'x': 0.0, 'y': 0.0,
                                    'z': 0.0, 'w': 1.0},
                },
                'scale': {'x': max(float(s[0]), 0.3),
                          'y': max(float(s[1]), 0.3),
                          'z': max(float(s[2]), 0.1)},
                'color': {'r': 1.0, 'g': 0.5, 'b': 0.0, 'a': 0.4},
                'lifetime': {'secs': 0, 'nsecs': 300000000},  # 0.3s
            }
            markers.append(marker)

        # 删除上一帧多余的 marker
        for i in range(len(detections), len(detections) + 5):
            markers.append({
                'header': {'stamp': stamp, 'frame_id': frame_id},
                'ns': 'persons', 'id': i,
                'type': 1, 'action': 2,  # DELETE
                'pose': {'position': {'x': 0.0, 'y': 0.0, 'z': 0.0},
                         'orientation': {'x': 0.0, 'y': 0.0,
                                         'z': 0.0, 'w': 1.0}},
                'scale': {'x': 0.1, 'y': 0.1, 'z': 0.1},
                'color': {'r': 0.0, 'g': 0.0, 'b': 0.0, 'a': 0.0},
                'lifetime': {'secs': 0, 'nsecs': 0},
            })

        self.pub_markers.publish(
            roslibpy.Message({'markers': markers}))

    def _publish_centers(self, detections: List[dict], header: dict):
        """发布质心点云（给 AB3DMOT tracking 用）"""
        import base64, struct
        if not detections:
            return

        stamp = header.get('stamp', {'secs': 0, 'nsecs': 0})
        frame_id = header.get('frame_id', 'livox_frame')

        # 每个质心打包成 xyz float32
        raw = b''
        for det in detections:
            c = det['center']
            raw += struct.pack('<fff', float(c[0]),
                               float(c[1]), float(c[2]))

        msg = {
            'header': {'stamp': stamp, 'frame_id': frame_id},
            'height': 1,
            'width': len(detections),
            'fields': [
                {'name': 'x', 'offset': 0,
                 'datatype': 7, 'count': 1},
                {'name': 'y', 'offset': 4,
                 'datatype': 7, 'count': 1},
                {'name': 'z', 'offset': 8,
                 'datatype': 7, 'count': 1},
            ],
            'is_bigendian': False,
            'point_step': 12,
            'row_step': 12 * len(detections),
            'data': base64.b64encode(raw).decode('ascii'),
            'is_dense': True,
        }
        self.pub_centers.publish(roslibpy.Message(msg))

    def spin(self):
        """阻塞运行直到 Ctrl+C"""
        print("\nRunning... Press Ctrl+C to stop\n")
        try:
            while self.client.is_connected:
                time.sleep(0.1)
        except KeyboardInterrupt:
            pass
        finally:
            self.sub.unsubscribe()
            self.client.terminate()
            print(f"\nStopped after {self.frame_count} frames")


# ─── 独立测试（不需要 ROS）──────────────────────────────────────────────────

def run_test():
    """验证 clustering 算法本身，不需要 ROS 或 roslibpy"""
    print("=" * 55)
    print("TEST: Euclidean Clustering Algorithm")
    print("=" * 55)
    rng = np.random.default_rng(0)

    # 模拟3m天花板俯视，3个人站在不同位置
    def make_person(cx, cy, cz=-1.7, n=50):
        return np.column_stack([
            rng.uniform(cx - 0.25, cx + 0.25, n),
            rng.uniform(cy - 0.25, cy + 0.25, n),
            rng.uniform(cz - 0.3, cz, n),
        ]).astype(np.float32)

    persons = [
        make_person(0.0, 0.0),   # 人1：原点
        make_person(1.5, 0.5),   # 人2：1.5m外
        make_person(-1.0, 1.2),  # 人3：斜后方
    ]

    # 加一些残余背景噪声（BG removal 没过滤干净的点）
    noise = np.column_stack([
        rng.uniform(-3, 3, 20),
        rng.uniform(-3, 3, 20),
        rng.uniform(-2.8, -2.5, 20),
    ]).astype(np.float32)

    pts = np.vstack(persons + [noise])
    print(f"Input: {len(pts)} points "
          f"(3 persons × 50pts + 20 noise)")

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

if __name__ == '__main__':
    parser = argparse.ArgumentParser(
        description='Euclidean Clustering Node for Person Detection',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__
    )
    parser.add_argument('--jetson_ip', type=str,
                        default='172.26.42.167',
                        help='Jetson Node 1 IP address')
    parser.add_argument('--port', type=int, default=9090,
                        help='rosbridge websocket port (default 9090)')
    parser.add_argument('--topic', type=str,
                        default='/livox/lidar_foreground',
                        help='Input foreground point cloud topic')

    # Clustering 参数
    parser.add_argument('--cluster_tol', type=float, default=0.4,
                        help='''
Euclidean clustering 距离容差，单位：米（默认 0.4）。
含义：两点距离 <= 此值则归为同一 cluster。
  太小（<0.25）→ 一个人被分成多个 cluster
  太大（>0.6）  → 相邻两人被合并成一个 cluster
天花板3m俯视推荐范围：0.35 ~ 0.45
Hunt Library 安装后无需修改此参数。
''')
    parser.add_argument('--min_points', type=int, default=8,
                        help='''
cluster 最少点数（默认 8）。
含义：点数少于此值的 cluster 被丢弃（认为是噪声）。
  3m高度下一个人约 30~80 点。
  设 8 比较保守，能保留稀疏的人体点云。
  如果误检多，可以调高到 15~20。
''')
    parser.add_argument('--max_points', type=int, default=800,
                        help='''
cluster 最多点数（默认 800）。
含义：点数多于此值的 cluster 被丢弃（认为是大型物体，如推车/柱子）。
  正常人体在3m高度下 < 150 点，设 800 很宽松。
  如果有大型移动物体干扰，可以调低到 300~500。
''')
    parser.add_argument('--max_persons', type=int, default=20,
                        help='最多输出多少个检测结果（默认 20，防止噪声爆炸）')
    parser.add_argument('--test', action='store_true',
                        help='Run algorithm test without ROS/roslibpy')
    args = parser.parse_args()

    if args.test:
        run_test()
    else:
        if not ROSLIBPY_AVAILABLE:
            print("ERROR: roslibpy not installed.")
            print("Install: pip install roslibpy")
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
