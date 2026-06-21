#!/usr/bin/env python3
"""
Statistical Background Model Builder
=====================================
从空场景 rosbag 建立统计背景模型，供 statistical_bg_node.py 使用。

原理:
  对空场景每个 voxel，记录所有落入其中的点的 xyz 均值和标准差。
  LiDAR ±2-3cm 的噪声抖动会被学进 std 里，之后过滤时不会误判为前景。

用法:
  # 正常模式（从 rosbag 建模）
  python3 statistical_bg_build.py \
      --bag /path/to/empty_scene.bag \
      --output ~/background_statistical.npz \
      --voxel_size 0.10 \
      --topic /livox/lidar

  # 快速测试模式（不需要 rosbag，验证代码能跑）
  python3 statistical_bg_build.py --test

  # 从已有的 voxel occupancy 模型迁移（兼容你 Jetson 上的 background_model.npz）
  python3 statistical_bg_build.py --convert ~/background_model.npz --output ~/background_statistical.npz

参考: PALMAR (Ul Alam et al. 2021), Brščić et al. 2013
"""

import argparse
import sys
import time
import numpy as np
from pathlib import Path
from collections import defaultdict
try:
    from livox_ros_driver2.msg import CustomMsg
except ImportError:
    CustomMsg = None

# ─── 核心建模逻辑 ─────────────────────────────────────────────────────────────

def build_statistical_model(points_iterator, voxel_size: float = 0.10,
                            min_points_per_voxel: int = 3) -> dict:
    """
    从点云帧序列建立统计背景模型。

    Args:
        points_iterator: 可迭代对象，每次返回一帧的 np.ndarray (N, 3)
        voxel_size: voxel 边长，单位米。越大越粗糙但越鲁棒
        min_points_per_voxel: 至少见过多少次才算背景（过滤偶发噪声）

    Returns:
        model dict: {
            'keys': np.ndarray (M, 3) int32,   # voxel 坐标
            'means': np.ndarray (M, 3) float32, # 每个 voxel 的 xyz 均值
            'stds': np.ndarray (M, 3) float32,  # 每个 voxel 的 xyz 标准差
            'voxel_size': float
        }
    """
    voxel_acc = defaultdict(list)  # key -> list of (x, y, z)

    frame_count = 0
    total_points = 0

    for pts in points_iterator:
        if pts is None or len(pts) == 0:
            continue

        pts = np.asarray(pts, dtype=np.float32)
        if pts.ndim != 2 or pts.shape[1] < 3:
            continue
        pts = pts[:, :3]

        # 过滤 NaN/Inf
        valid = np.isfinite(pts).all(axis=1)
        pts = pts[valid]
        if len(pts) == 0:
            continue

        # 计算 voxel key
        voxel_keys = (pts / voxel_size).astype(np.int32)

        for i in range(len(pts)):
            k = (int(voxel_keys[i, 0]),
                 int(voxel_keys[i, 1]),
                 int(voxel_keys[i, 2]))
            voxel_acc[k].append(pts[i])

        frame_count += 1
        total_points += len(pts)

        if frame_count % 50 == 0:
            print(f"  [{frame_count} frames] {len(voxel_acc)} voxels, "
                  f"{total_points} total points")

    print(f"\nBuild complete: {frame_count} frames, "
          f"{len(voxel_acc)} voxels occupied")

    # 计算每个 voxel 的统计量
    valid_keys = []
    means_list = []
    stds_list = []

    for k, pts_list in voxel_acc.items():
        if len(pts_list) < min_points_per_voxel:
            continue  # 太少点，可能是偶发噪声，不算背景
        arr = np.array(pts_list, dtype=np.float32)
        valid_keys.append(k)
        means_list.append(arr.mean(axis=0))
        # std + 1e-3：保证最小容许范围是 1mm，避免除零
        stds_list.append(arr.std(axis=0) + 1e-3)

    print(f"Background voxels (>= {min_points_per_voxel} points): {len(valid_keys)}")

    model = {
        'keys': np.array(valid_keys, dtype=np.int32),
        'means': np.array(means_list, dtype=np.float32),
        'stds': np.array(stds_list, dtype=np.float32),
        'voxel_size': float(voxel_size),
    }
    return model


def save_model(model: dict, output_path: str):
    output_path = str(output_path)
    if not output_path.endswith('.npz'):
        output_path += '.npz'
    np.savez(output_path,
             keys=model['keys'],
             means=model['means'],
             stds=model['stds'],
             voxel_size=np.float32(model['voxel_size']))
    print(f"Model saved → {output_path} "
          f"({len(model['keys'])} background voxels)")
    return output_path


def load_model(model_path: str) -> dict:
    data = np.load(model_path)
    return {
        'keys': data['keys'],
        'means': data['means'],
        'stds': data['stds'],
        'voxel_size': float(np.asarray(data['voxel_size']).flat[0]),
    }


# ─── 从 rosbag 读取 ───────────────────────────────────────────────────────────

def iter_bag_frames(bag_path: str, topic: str, max_frames: int = 500):
    """从 rosbag2 逐帧读取点云，yield np.ndarray (N, 3)"""
    try:
        import rosbag2_py
        from rclpy.serialization import deserialize_message
        from sensor_msgs.msg import PointCloud2
        import sensor_msgs_py.point_cloud2 as pc2
    except ImportError as e:
        raise RuntimeError(
            f"ROS2 packages not available: {e}\n"
            "Run with --test flag to test without ROS."
        )

    storage_options = rosbag2_py.StorageOptions(
        uri=str(bag_path), storage_id='sqlite3')
    converter_options = rosbag2_py.ConverterOptions('', '')
    reader = rosbag2_py.SequentialReader()
    reader.open(storage_options, converter_options)

    frame_count = 0
    while reader.has_next() and frame_count < max_frames:
        topic_name, data, _ = reader.read_next()
        if topic_name != topic:
            continue
        msg = deserialize_message(data, PointCloud2)
        pts = list(pc2.read_points(
            msg, field_names=('x', 'y', 'z'), skip_nans=True))
        if pts:
            yield np.array([(p[0], p[1], p[2]) for p in pts],
                           dtype=np.float32)
        frame_count += 1


# ─── 从已有 voxel occupancy 模型迁移 ────────────────────────────────────────

def convert_from_voxel_occupancy(old_model_path: str,
                                 output_path: str,
                                 voxel_size: float = 0.10):
    """
    把 Jetson 上已有的 background_model.npz（voxel occupancy 格式）
    转换为 statistical 格式。

    因为旧模型没有 mean/std 信息，std 用 voxel_size/4 近似
    （相当于假设背景点在 voxel 内均匀分布）。
    这只是临时方案，有空场景 rosbag 之后应重新建模。
    """
    data = np.load(old_model_path, allow_pickle=True)

    # 兼容多种旧格式
    if 'keys' in data:
        keys = data['keys'].astype(np.int32)
        old_vs = float(np.asarray(data.get('voxel_size',
                                           np.float32(voxel_size))).flat[0])
    elif 'voxel_indices' in data:
        keys = data['voxel_indices'].astype(np.int32)
        old_vs = voxel_size
    elif 'occupied_voxels' in data:
        keys = data['occupied_voxels'].astype(np.int32)
        old_vs = voxel_size
    else:
        raise ValueError(f"Unknown format in {old_model_path}. "
                         f"Keys: {list(data.keys())}")

    print(f"Converting {len(keys)} voxels from {old_model_path}")
    print(f"  Old voxel_size={old_vs}, new voxel_size={voxel_size}")

    # 用 voxel 中心作为 mean，用 voxel_size/4 作为 std 近似
    means = (keys.astype(np.float32) + 0.5) * old_vs
    stds = np.full_like(means, fill_value=old_vs / 4.0)

    model = {
        'keys': keys,
        'means': means,
        'stds': stds,
        'voxel_size': voxel_size,
    }
    saved = save_model(model, output_path)
    print(f"Converted model saved → {saved}")
    print("NOTE: This is an approximation. Rebuild from empty rosbag when possible.")
    return model


# ─── 测试模式 ─────────────────────────────────────────────────────────────────

def run_test(output_path: str = '/tmp/bg_statistical_test.npz'):
    """不需要 rosbag，生成假数据验证整个流程"""
    print("=" * 55)
    print("TEST MODE: Generating synthetic empty-scene data")
    print("=" * 55)
    rng = np.random.default_rng(42)

    def fake_frames(n_frames=100):
        """
        模拟天花板3m倒置的空场景点云。
        用结构化表面点（地板/桌子/椅子）而非随机均匀分布，
        确保人体区域 (x∈[0.7,1.3], y∈[0.2,0.8]) 在空场景下没有点。
        """
        for _ in range(n_frames):
            frame_pts = []
            # 地板 z≈-2.8m
            n_floor = rng.integers(1500, 2000)
            floor = np.column_stack([
                rng.uniform(-3.0, 3.0, n_floor),
                rng.uniform(-3.0, 3.0, n_floor),
                np.full(n_floor, -2.8),
            ]).astype(np.float32)
            floor += rng.normal(0, 0.02, floor.shape).astype(np.float32)
            frame_pts.append(floor)
            # 桌子 z≈-1.9m，在人体区域之外
            n_desk = rng.integers(200, 400)
            desk = np.column_stack([
                rng.uniform(-2.5, -0.5, n_desk),
                rng.uniform(-2.5, 2.5, n_desk),
                np.full(n_desk, -1.9),
            ]).astype(np.float32)
            desk += rng.normal(0, 0.02, desk.shape).astype(np.float32)
            frame_pts.append(desk)
            yield np.vstack(frame_pts)

    model = build_statistical_model(fake_frames(), voxel_size=0.10)
    saved = save_model(model, output_path)

    # 验证：模拟一帧有人的场景
    print("\nValidation: simulating frame with 1 person...")
    n_bg = 4000
    bg_pts = np.column_stack([
        rng.uniform(-3.0, 3.0, n_bg),
        rng.uniform(-3.0, 3.0, n_bg),
        rng.uniform(-3.0, -0.3, n_bg),
    ]).astype(np.float32)
    bg_pts += rng.normal(0, 0.02, bg_pts.shape).astype(np.float32)

    # 人站在 (1.0, 0.5)，从3m天花板往下看
    n_person = 60
    person_pts = np.column_stack([
        rng.uniform(0.7, 1.3, n_person),
        rng.uniform(0.2, 0.8, n_person),
        rng.uniform(-1.8, -1.2, n_person),
    ]).astype(np.float32)

    frame = np.vstack([bg_pts, person_pts])

    # 应用过滤
    loaded = load_model(saved)
    keys_arr = loaded['keys']
    means_arr = loaded['means']
    stds_arr = loaded['stds']
    vs = loaded['voxel_size']
    sigma = 2.0

    # 建快速查找 dict
    lookup = {}
    for i in range(len(keys_arr)):
        k = (int(keys_arr[i, 0]), int(keys_arr[i, 1]), int(keys_arr[i, 2]))
        lookup[k] = (means_arr[i], stds_arr[i])

    foreground = []
    for pt in frame:
        k = tuple((pt / vs).astype(np.int32))
        if k not in lookup:
            foreground.append(pt)
            continue
        mean, std = lookup[k]
        dist = float(np.linalg.norm(pt - mean))
        tol = sigma * float(np.max(std))
        if dist > tol:
            foreground.append(pt)

    foreground = np.array(foreground)
    person_retained = np.sum(
        (foreground[:, 0] >= 0.7) & (foreground[:, 0] <= 1.3) &
        (foreground[:, 1] >= 0.2) & (foreground[:, 1] <= 0.8)
    )

    print(f"  Input:             {len(frame):>5} points "
          f"({n_bg} bg + {n_person} person)")
    print(f"  Foreground output: {len(foreground):>5} points")
    print(f"  BG leakage:        {len(foreground) - person_retained:>5} points "
          f"(target < 50)")
    print(f"  Person retained:   {person_retained:>5}/{n_person} points")

    if person_retained >= n_person * 0.8:
        print("\n✓ TEST PASSED")
    else:
        print("\n✗ TEST FAILED: too many person points filtered")

    return saved


# ─── main ─────────────────────────────────────────────────────────────────────

if __name__ == '__main__':
    parser = argparse.ArgumentParser(
        description='Build statistical background model for LiDAR BG removal',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__
    )
    parser.add_argument('--bag', type=str, default=None,
                        help='Path to empty-scene rosbag2 directory')
    parser.add_argument('--output', type=str,
                        default='~/background_statistical.npz',
                        help='Output model path')
    parser.add_argument('--voxel_size', type=float, default=0.10,
                        help='Voxel size in meters (default 0.10)')
    parser.add_argument('--topic', type=str, default='/livox/lidar',
                        help='LiDAR topic name')
    parser.add_argument('--max_frames', type=int, default=500,
                        help='Max frames to process from bag')
    parser.add_argument('--min_points', type=int, default=3,
                        help='Min points per voxel to count as background')
    parser.add_argument('--test', action='store_true',
                        help='Run with synthetic data (no rosbag needed)')
    parser.add_argument('--convert', type=str, default=None,
                        help='Convert existing voxel occupancy .npz to statistical format')
    args = parser.parse_args()

    output = str(Path(args.output).expanduser())

    if args.test:
        run_test(output)

    elif args.convert:
        convert_from_voxel_occupancy(
            args.convert, output, args.voxel_size)

    elif args.bag:
        print(f"Building from rosbag: {args.bag}")
        print(f"  topic={args.topic}, voxel_size={args.voxel_size}, "
              f"max_frames={args.max_frames}")
        t0 = time.time()
        model = build_statistical_model(
            iter_bag_frames(args.bag, args.topic, args.max_frames),
            voxel_size=args.voxel_size,
            min_points_per_voxel=args.min_points
        )
        save_model(model, output)
        print(f"Done in {time.time() - t0:.1f}s")

    else:
        print("No input specified. Use --bag, --test, or --convert.")
        print("Run with --help for usage.")
        sys.exit(1)
