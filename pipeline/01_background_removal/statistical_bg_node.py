#!/usr/bin/env python3
"""
Statistical Background Removal Node
=====================================
替换 Jetson 上的 background_removal_node.py。
接口完全兼容：/livox/lidar → /livox/lidar_foreground

原理:
  每个 voxel 记录背景点的 xyz 均值和标准差。
  新帧中，若某点偏离所在 voxel 的背景均值超过 sigma*std，
  则认为是前景（有人），保留；否则过滤。

  优势：把 LiDAR ±2-3cm 的噪声抖动学进 std，
  不会因为背景点轻微抖动就误判为前景。
  对静止的人同样有效（静止人体在背景模型里从未出现过）。

安装到 Jetson:
  scp statistical_bg_build.py statistical_bg_node.py \
      kelrod@172.26.42.167:~/ros2_ws/src/lidar_filtering/lidar_filtering/

启动方式（和原来完全一样）:
  # Step 1: 先建模型（Legion 上跑，用空场景 rosbag）
  python3 statistical_bg_build.py --bag ./empty_scene.bag --output ~/background_statistical.npz

  # Step 2: 把模型传到 Jetson
  scp ~/background_statistical.npz kelrod@172.26.42.167:~/background_statistical.npz

  # Step 3: 在 Jetson 上启动（替换原来的 background_removal_node）
  python3 statistical_bg_node.py \
      --model ~/background_statistical.npz \
      --sigma 2.0 \
      --input_topic /livox/lidar \
      --output_topic /livox/lidar_foreground

  # 或者用已有的 background_model.npz 临时转换
  python3 statistical_bg_build.py \
      --convert ~/background_model.npz \
      --output ~/background_statistical.npz
  python3 statistical_bg_node.py --model ~/background_statistical.npz

参考:
  PALMAR (Ul Alam et al. 2021) - voxelized feature representation
  Brščić et al. 2013 - background subtraction for indoor LiDAR
"""

import argparse
import sys
import time
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs_py.point_cloud2 as pc2
from std_msgs.msg import Header
import struct


# ─── 核心过滤逻辑（与 ROS 解耦，方便单元测试）──────────────────────────────

class StatisticalBGFilter:
    """
    可独立测试的背景过滤器核心。
    不依赖 ROS，可以直接在 Legion 上测试逻辑。
    """

    def __init__(self, model_path: str, sigma: float = 2.0):
        self.sigma = sigma
        self._load_model(model_path)
        print(f"[StatBGFilter] Loaded {len(self.lookup)} background voxels, "
              f"voxel_size={self.voxel_size:.3f}m, sigma={self.sigma}")

    def _load_model(self, model_path: str):
        data = np.load(str(model_path))
        keys = data['keys'].astype(np.int32)       # (N, 3)
        means = data['means'].astype(np.float32)   # (N, 3)
        stds = data['stds'].astype(np.float32)     # (N, 3)
        self.voxel_size = float(
            np.asarray(data['voxel_size']).flat[0])

        # 建 lookup dict: tuple(vx,vy,vz) -> (mean_xyz, std_xyz)
        self.lookup = {}
        for i in range(len(keys)):
            k = (int(keys[i, 0]), int(keys[i, 1]), int(keys[i, 2]))
            self.lookup[k] = (means[i], stds[i])

    def filter(self, pts: np.ndarray) -> np.ndarray:
        """
        Args:
            pts: np.ndarray (N, 3), float32

        Returns:
            foreground: np.ndarray (M, 3), M <= N
        """
        if len(pts) == 0:
            return pts

        vs = self.voxel_size
        sigma = self.sigma

        # 向量化计算 voxel key
        voxel_keys = (pts / vs).astype(np.int32)  # (N, 3)

        # 逐点判断（Python loop，但 N 通常 < 10000，速度可接受）
        foreground_mask = np.ones(len(pts), dtype=bool)

        for i in range(len(pts)):
            k = (int(voxel_keys[i, 0]),
                 int(voxel_keys[i, 1]),
                 int(voxel_keys[i, 2]))

            if k not in self.lookup:
                # 背景模型里从未见过这个位置 → 前景
                continue

            mean, std = self.lookup[k]
            dist = float(np.linalg.norm(pts[i] - mean))
            # 容许半径 = sigma * max(各轴std)
            # 用 max 而不是 norm，对各向异性噪声更鲁棒
            tolerance = sigma * float(np.max(std))

            if dist <= tolerance:
                foreground_mask[i] = False  # 在背景范围内 → 过滤

        return pts[foreground_mask]


# ─── ROS2 Node ────────────────────────────────────────────────────────────────

class StatisticalBGRemovalNode(Node):

    def __init__(self,
                 model_path: str,
                 sigma: float,
                 input_topic: str,
                 output_topic: str,
                 z_min: float,
                 z_max: float):
        super().__init__('statistical_bg_removal_node')

        self.z_min = z_min
        self.z_max = z_max

        # 初始化过滤器
        self.filter = StatisticalBGFilter(model_path, sigma)

        # QoS：Best Effort，和 Livox driver 匹配
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.sub = self.create_subscription(
            PointCloud2, input_topic, self.callback, qos)
        self.pub = self.create_publisher(
            PointCloud2, output_topic, 10)

        # 统计信息
        self.frame_count = 0
        self.t_last_log = time.time()
        self.residual_history = []  # 记录空场景残余点数，用于后续评估

        self.get_logger().info(
            f'Statistical BG Removal Node started\n'
            f'  Input:  {input_topic}\n'
            f'  Output: {output_topic}\n'
            f'  Model:  {len(self.filter.lookup)} background voxels\n'
            f'  Params: sigma={sigma}, '
            f'z_min={z_min}, z_max={z_max}'
        )

    def callback(self, msg: PointCloud2):
        t0 = time.time()

        # 读点云
        raw = list(pc2.read_points(
            msg, field_names=('x', 'y', 'z'), skip_nans=True))
        if not raw:
            return

        pts = np.array([(p[0], p[1], p[2]) for p in raw],
                       dtype=np.float32)

        # Step 1: z 范围裁剪（和原来的 background_removal_node 保持一致）
        # 倒置安装：z 是负值，z_min=-2.8, z_max=-0.5
        if self.z_min is not None and self.z_max is not None:
            z_mask = (pts[:, 2] >= self.z_min) & (pts[:, 2] <= self.z_max)
            pts = pts[z_mask]

        if len(pts) == 0:
            return

        # Step 2: 统计背景过滤
        foreground = self.filter.filter(pts)

        # Step 3: 发布前景点云
        fg_msg = self._make_pointcloud2(foreground, msg.header)
        self.pub.publish(fg_msg)

        # 日志（每 30 帧打印一次）
        self.frame_count += 1
        self.residual_history.append(len(foreground))

        dt = time.time() - t0
        if self.frame_count % 30 == 0:
            recent = self.residual_history[-30:]
            self.get_logger().info(
                f'[frame {self.frame_count:>5}] '
                f'in={len(pts):>5} → fg={len(foreground):>4} | '
                f'avg_fg(30f)={np.mean(recent):.0f} | '
                f'latency={dt*1000:.1f}ms'
            )

    @staticmethod
    def _make_pointcloud2(pts: np.ndarray, header: Header) -> PointCloud2:
        """numpy (N,3) → PointCloud2 msg"""
        msg = PointCloud2()
        msg.header = header
        msg.height = 1
        msg.width = len(pts)
        msg.is_dense = False
        msg.is_bigendian = False
        msg.fields = [
            PointField(name='x', offset=0,
                       datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4,
                       datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8,
                       datatype=PointField.FLOAT32, count=1),
        ]
        msg.point_step = 12  # 3 * 4 bytes
        msg.row_step = msg.point_step * len(pts)
        msg.data = pts.astype(np.float32).tobytes()
        return msg


# ─── main ─────────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(
        description='Statistical Background Removal Node',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__
    )
    parser.add_argument('--model', type=str,
                        default='~/background_statistical.npz',
                        help='Path to statistical background model')
    parser.add_argument('--sigma', type=float, default=2.0,
                        help='Sigma threshold (default 2.0). '
                             'Higher = less aggressive filtering.')
    parser.add_argument('--input_topic', type=str,
                        default='/livox/lidar')
    parser.add_argument('--output_topic', type=str,
                        default='/livox/lidar_foreground')
    parser.add_argument('--z_min', type=float, default=-2.8,
                        help='Min z after inversion (default -2.8m)')
    parser.add_argument('--z_max', type=float, default=-0.5,
                        help='Max z after inversion (default -0.5m). '
                             'Cuts off the ceiling plane.')

    args, ros_args = parser.parse_known_args()
    model_path = str(__import__('pathlib').Path(args.model).expanduser())

    rclpy.init(args=ros_args if ros_args else None)

    node = StatisticalBGRemovalNode(
        model_path=model_path,
        sigma=args.sigma,
        input_topic=args.input_topic,
        output_topic=args.output_topic,
        z_min=args.z_min,
        z_max=args.z_max,
    )

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        n = len(node.residual_history)
        if n > 0:
            node.get_logger().info(
                f'Session summary: {n} frames, '
                f'avg foreground={np.mean(node.residual_history):.0f} pts, '
                f'max={max(node.residual_history)} pts'
            )
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
