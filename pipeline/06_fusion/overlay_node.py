#!/usr/bin/env python3
"""
Dual-LiDAR Overlay Node — Calibration Diagnostic
==================================================
Connects to TWO Jetson rosbridge servers (node1 + node3), subscribes to each
node's point-cloud topic, colors the clouds distinctly
(node1 = orange, node3 = blue), and republishes them combined into a single
PointCloud2 in a common frame.

No spatial transform is applied to node3 by default (identity).
The transform_to_common matrix in the fusion: section of nodes_config.yaml
is the single calibration knob — see that block for instructions.

Usage:
    conda activate livox
    cd /path/to/lidar_social_recognition

    # Normal mode — sparse foreground clouds (background removed):
    python3 pipeline/06_fusion/overlay_node.py --config config/nodes_config.yaml

    # Calibration mode — dense raw clouds (walls/floor/structure visible):
    python3 pipeline/06_fusion/overlay_node.py --config config/nodes_config.yaml --raw

    # Calibration mode with custom downsampling and publish rate:
    python3 pipeline/06_fusion/overlay_node.py --config config/nodes_config.yaml --raw \
        --voxel 0.15 --raw-hz 2.0

    # Enable per-frame WebSocket debug logging:
    python3 pipeline/06_fusion/overlay_node.py ... --debug

Foxglove:
    Connect to:  ws://<node1-ip>:9090    (exact URL printed at startup)
    3D panel  -> subscribe to /fused/foreground  (or /fused/raw in --raw mode)
    Point Cloud settings -> Color by = rgb
    Orange = node1  |  Blue = node3

--raw mode downsampling:
    Raw clouds (~20 k pts/frame) are voxel-grid downsampled before republishing
    to prevent rosbridge ping/pong timeouts over WebSocket.
    --voxel METERS   voxel size for downsampling (default 0.10 m → ~2–4 k pts)
    --raw-hz HZ      max fused-cloud publish rate in raw mode (default 3.0 Hz)
    Set --voxel 0 to disable downsampling (may cause WebSocket drops at 10 Hz).
"""

import argparse
import base64
import struct
import sys
import threading
import time
from pathlib import Path
from typing import Optional

import numpy as np
import yaml

# ── import RosBridgeClient from clustering_node ───────────────────────────────
_PIPELINE_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(_PIPELINE_ROOT / "02_detection"))
from clustering_node import RosBridgeClient  # noqa: E402


# ─── PointCloud2 decode ───────────────────────────────────────────────────────

# Per-source drop counters and one-time metadata log guard.
_decode_drop_counts: dict = {}
_decode_meta_logged: set  = set()


def _drop(source: str, reason: str) -> None:
    """Increment drop counter and log every drop (throttled after first 3)."""
    cnt = _decode_drop_counts.get(source, 0) + 1
    _decode_drop_counts[source] = cnt
    if cnt <= 3 or cnt % 50 == 0:
        print(f"[WARN][{source}] decode drop #{cnt}: {reason}")


def decode_pointcloud2(msg: dict, source: str = "") -> Optional[np.ndarray]:
    """Decode a rosbridge PointCloud2 message → (N, 3) float32 xyz array."""
    try:
        raw_data  = msg.get("data", "")
        fields    = msg.get("fields", [])
        field_map = {f["name"]: f["offset"] for f in fields}
        step      = int(msg.get("point_step", 16))
        width     = int(msg.get("width", 0))
        height    = int(msg.get("height", 1))
        bigendian = bool(msg.get("is_bigendian", False))
        field_names = [f["name"] for f in fields]
        field_offsets   = [f.get("offset") for f in fields]
        field_datatypes = [f.get("datatype") for f in fields]

        # One-time metadata dump per source — reveals the exact drop branch
        # when comparing node1 vs node3 on first received frame.
        if source not in _decode_meta_logged:
            _decode_meta_logged.add(source)
            data_type = type(raw_data).__name__
            data_len  = len(raw_data) if raw_data else 0
            print(
                f"[DBG][{source}] first PointCloud2 frame metadata:\n"
                f"  width={width}  height={height}  point_step={step}"
                f"  is_bigendian={bigendian}\n"
                f"  data type={data_type}  data len={data_len}\n"
                f"  fields={field_names}\n"
                f"  offsets={field_offsets}  datatypes={field_datatypes}"
            )

        # ── Branch 1: empty data ──────────────────────────────────────────────
        if not raw_data:
            _drop(source, f"empty data field (type={type(raw_data).__name__} "
                          f"width={width} fields={field_names})")
            return None

        # ── Decode data: base64 string, raw bytes, or JSON int-array ─────────
        if isinstance(raw_data, list):
            # Rosbridge may serialize uint8[] as a JSON int-array instead of
            # base64 on some ROS2 / rosbridge_suite versions.
            raw = bytes(raw_data)
        elif isinstance(raw_data, (bytes, bytearray)):
            raw = bytes(raw_data)
        else:
            raw = base64.b64decode(raw_data)

        # ── Branch 2: zero width or missing xyz fields ────────────────────────
        if width == 0:
            _drop(source, f"width=0 (height={height} fields={field_names})")
            return None
        if "x" not in field_map:
            _drop(source, f"no 'x' field — got {field_names} "
                          f"(point_step={step} width={width})")
            return None

        # ── Decode points ─────────────────────────────────────────────────────
        endian = ">" if bigendian else "<"
        fmt_f  = f"{endian}f"
        xo, yo, zo = field_map["x"], field_map["y"], field_map["z"]
        # Some conversions produce organized clouds (height > 1); iterate all.
        n_pts = width * height
        pts = []
        for i in range(n_pts):
            b = i * step
            x = struct.unpack_from(fmt_f, raw, b + xo)[0]
            y = struct.unpack_from(fmt_f, raw, b + yo)[0]
            z = struct.unpack_from(fmt_f, raw, b + zo)[0]
            if np.isfinite(x) and np.isfinite(y) and np.isfinite(z):
                pts.append((x, y, z))

        # ── Branch 3: all points non-finite ──────────────────────────────────
        if not pts:
            _drop(source, f"no finite points from {n_pts} pts "
                          f"(point_step={step} width={width} height={height})")
            return None

        return np.array(pts, dtype=np.float32)

    except Exception as e:
        cnt = _decode_drop_counts.get(source, 0) + 1
        _decode_drop_counts[source] = cnt
        if cnt <= 3 or cnt % 50 == 0:
            print(f"[WARN][{source}] decode_pointcloud2 exception #{cnt}: {e} "
                  f"(data type={type(msg.get('data', '')).__name__})")
        return None


# ─── PointCloud2 encode (XYZRGB, 16 bytes/point) ─────────────────────────────

def encode_fused_cloud(
    entries: list,   # [(pts: ndarray Nx3 float32, color: (r,g,b) 0-1), ...]
    frame_id: str,
    stamp: dict,
) -> dict:
    """
    Interleave multiple colored point arrays into one XYZRGB PointCloud2.

    Layout per point: x(f32) y(f32) z(f32) rgb(u32 packed, stored as f32 bytes)
    rgb packing (standard ROS XYZRGB): (R<<16)|(G<<8)|B, values 0-255.
    Foxglove reads the 'rgb' field and renders per-point color automatically.
    """
    chunks = []
    for pts, (r, g, b) in entries:
        if pts is None or len(pts) == 0:
            continue
        ri = int(r * 255) & 0xFF
        gi = int(g * 255) & 0xFF
        bi = int(b * 255) & 0xFF
        rgb_int = (ri << 16) | (gi << 8) | bi
        n = len(pts)

        # xyz: (N, 3) float32 → reshape to (N, 12) bytes
        xyz_bytes = pts.astype(np.float32).tobytes()
        xyz_view  = np.frombuffer(xyz_bytes, dtype=np.uint8).reshape(n, 12)

        # rgb: N copies of the same uint32, packed as (N, 4) bytes
        rgb_arr  = np.full(n, rgb_int, dtype=np.uint32)
        rgb_view = rgb_arr.view(np.uint8).reshape(n, 4)

        # interleave to (N, 16)
        chunks.append(np.hstack([xyz_view, rgb_view]))

    if not chunks:
        buf = bytes()
        n_total = 0
    else:
        buf = np.vstack(chunks).tobytes()
        n_total = len(buf) // 16

    return {
        "header": {"stamp": stamp, "frame_id": frame_id},
        "height": 1,
        "width":  n_total,
        "fields": [
            {"name": "x",   "offset": 0,  "datatype": 7, "count": 1},
            {"name": "y",   "offset": 4,  "datatype": 7, "count": 1},
            {"name": "z",   "offset": 8,  "datatype": 7, "count": 1},
            {"name": "rgb", "offset": 12, "datatype": 7, "count": 1},
        ],
        "is_bigendian": False,
        "point_step": 16,
        "row_step":   16 * n_total,
        "data": base64.b64encode(buf).decode("ascii"),
        "is_dense": True,
    }


# ─── Transform ────────────────────────────────────────────────────────────────

def apply_transform(pts: np.ndarray, T: np.ndarray) -> np.ndarray:
    """Apply a 4×4 homogeneous transform T to an (N, 3) float32 point array."""
    if pts is None or len(pts) == 0:
        return pts
    if np.allclose(T, np.eye(4)):
        return pts  # identity — skip math
    ones  = np.ones((len(pts), 1), dtype=np.float32)
    pts_h = np.hstack([pts, ones])           # (N, 4)
    return (T @ pts_h.T).T[:, :3].astype(np.float32)


# ─── Voxel-grid downsampling (pure numpy) ────────────────────────────────────

def downsample_voxel(pts: np.ndarray, voxel_size: float) -> np.ndarray:
    """
    Voxel-grid downsample an (N, 3) float32 array.

    Each occupied voxel contributes exactly one point (the first that falls in
    it).  Uses a structured-array void view so np.unique operates on the full
    3-int32 key without hashing collisions.

    Returns the downsampled (M, 3) array, M <= N.
    """
    if pts is None or len(pts) == 0 or voxel_size <= 0:
        return pts
    min_pt = pts.min(axis=0)
    idx = np.floor((pts - min_pt) / voxel_size).astype(np.int32)
    # View each (ix, iy, iz) triplet as a single 12-byte opaque key.
    keys = np.ascontiguousarray(idx).view(np.dtype((np.void, 12)))
    _, first = np.unique(keys, return_index=True)
    return pts[np.sort(first)]


# ─── Rate tracker ─────────────────────────────────────────────────────────────

class RateTracker:
    def __init__(self, window_sec: float = 5.0):
        self._window = window_sec
        self._times: list = []
        self._lock = threading.Lock()

    def tick(self):
        now = time.time()
        with self._lock:
            self._times.append(now)
            cutoff      = now - self._window
            self._times = [t for t in self._times if t >= cutoff]

    @property
    def hz(self) -> float:
        now = time.time()
        with self._lock:
            recent = [t for t in self._times if t >= now - self._window]
        if len(recent) < 2:
            return 0.0
        return (len(recent) - 1) / (recent[-1] - recent[0])


# ─── Overlay Node ─────────────────────────────────────────────────────────────

class OverlayNode:
    """
    Subscribes to a point-cloud topic on two Jetsons, applies per-source color
    and an optional rigid transform, then republishes the combined cloud as a
    single PointCloud2.

    calibration_mode=True  → subscribes to lidar_topic (/livox/lidar, dense raw
    clouds) and publishes into livox_frame so Foxglove shows it next to node1's
    raw cloud without a missing-TF error.

    Latest buffers: each callback stores the most recent transformed cloud for
    its source.  On every incoming frame from either source, ALL buffered clouds
    are concatenated and republished.  This gives a continuously-updating fused
    view without requiring time synchronization.
    """

    def __init__(self, config_path: str, calibration_mode: bool = False,
                 voxel_size: float = 0.10, raw_hz: float = 3.0):
        with open(config_path) as f:
            cfg = yaml.safe_load(f)

        node_cfgs = cfg.get("nodes", {})
        fusion    = cfg.get("fusion")
        if not fusion:
            raise ValueError(
                "Config missing top-level 'fusion:' block — "
                "add it to nodes_config.yaml (see template in overlay_node.py docstring)")

        # --raw CLI flag or fusion.calibration_mode: true in config
        calibration_mode = calibration_mode or bool(fusion.get("calibration_mode", False))

        sources     = fusion["sources"]
        publish_via = fusion.get("publish_via", sources[0]["node"])

        if calibration_mode:
            self._output_topic = fusion.get("output_topic_raw", "/fused/raw")
            # Publish into livox_frame so Foxglove can render alongside node1's
            # raw cloud without a missing-transform error.
            self._output_frame = "livox_frame"
        else:
            self._output_topic = fusion.get("output_topic", "/fused/foreground")
            self._output_frame = fusion.get("output_frame", "fused")

        self._calibration_mode: bool                    = calibration_mode
        self._voxel_size:       float                   = voxel_size if calibration_mode else 0.0
        self._raw_period:       float                   = (1.0 / raw_hz) if (calibration_mode and raw_hz > 0) else 0.0
        self._last_pub_time:    float                   = 0.0
        self._lock:    threading.Lock                   = threading.Lock()
        self._latest:  dict                             = {}   # name → (pts, color, stamp) | None
        self._rates:   dict                             = {}   # name → RateTracker
        self._sources: dict                             = {}   # name → {client, ok, color, T}
        self._publish_client: Optional[RosBridgeClient] = None

        mode_label = "CALIBRATION (raw clouds)" if calibration_mode else "normal (foreground clouds)"
        print("\n" + "=" * 62)
        print(f"  Dual-LiDAR Overlay Node — {mode_label}")
        print("=" * 62)
        if calibration_mode:
            ds_str = f"voxel {voxel_size:.3f} m" if voxel_size > 0 else "DISABLED (--voxel 0)"
            print(f"  Downsampling  : {ds_str}  |  publish rate {raw_hz:.1f} Hz")

        for src in sources:
            name  = src["node"]
            ncfg  = node_cfgs.get(name)
            if ncfg is None:
                raise ValueError(
                    f"fusion source '{name}' not found under nodes: in config")
            ip    = ncfg["jetson_ip"]
            port  = int(ncfg.get("rosbridge_port", 9090))
            if calibration_mode:
                topic = ncfg.get("lidar_topic", "/livox/lidar")
            else:
                topic = ncfg.get("foreground_topic", "/livox/lidar_foreground")
            color = tuple(float(c) for c in src.get("color", [1.0, 1.0, 1.0]))

            # node1 has no transform_to_common — it IS the reference frame.
            # node3 defaults to identity; fill in after calibration.
            raw_T = src.get("transform_to_common")
            T = np.eye(4, dtype=np.float64) if raw_T is None \
                else np.array(raw_T, dtype=np.float64)
            if T.shape != (4, 4):
                raise ValueError(
                    f"transform_to_common for {name} must be 4×4, got {T.shape}")

            is_id = np.allclose(T, np.eye(4))
            print(f"\n  [{name}]  {ip}:{port}")
            print(f"    topic     : {topic}")
            print(f"    color     : rgb({color[0]:.2f}, {color[1]:.2f}, {color[2]:.2f})")
            xform_str = 'IDENTITY — calibration pending (see TODO in config)' if is_id else 'CUSTOM'
            print(f"    transform : {xform_str}")
            print(f"    connecting ...", end="", flush=True)

            client = RosBridgeClient(host=ip, port=port)
            ok = client.connect(timeout=10.0)
            status = "OK" if ok else "FAILED — node offline, will skip"
            print(f" {status}")

            if ok:
                client.subscribe(
                    topic=topic,
                    msg_type="sensor_msgs/msg/PointCloud2",
                    callback=self._make_callback(name, T, color),
                    throttle_rate=0,
                )
                print(f"    subscribed -> {topic}")
            else:
                print(f"    [WARN] {name} offline — fused output will include remaining sources only")

            self._sources[name] = {"client": client, "ok": ok, "color": color, "T": T}
            self._rates[name]   = RateTracker()
            self._latest[name]  = None

            if name == publish_via and ok:
                self._publish_client = client

        # Fall back to first live connection if publish_via node is offline
        if self._publish_client is None:
            for s in self._sources.values():
                if s["ok"]:
                    self._publish_client = s["client"]
                    break
        if self._publish_client is None:
            raise RuntimeError(
                "All rosbridge connections failed. Check IPs and network.")

        pub_ncfg = node_cfgs[publish_via]
        pub_ip   = pub_ncfg["jetson_ip"]
        pub_port = pub_ncfg.get("rosbridge_port", 9090)
        self._publish_client.advertise(
            self._output_topic, "sensor_msgs/msg/PointCloud2")

        print(f"\n  Output topic  : {self._output_topic}  (frame_id: {self._output_frame})")
        print(f"  Published via : {publish_via} rosbridge  "
              f"ws://{pub_ip}:{pub_port}")
        print("\n" + "-" * 62)
        print(f"  FOXGLOVE — Connect to:  ws://{pub_ip}:{pub_port}")
        print(f"  FOXGLOVE — 3D panel  -> {self._output_topic}")
        print(f"  FOXGLOVE — Point Cloud settings -> Color by = rgb")
        print(f"  Color key: ORANGE = node1  |  BLUE = node3")
        print("-" * 62 + "\n")

        self._stop        = threading.Event()
        self._rate_thread = threading.Thread(target=self._rate_loop, daemon=True)
        self._rate_thread.start()

    # ── per-source callback factory ───────────────────────────────────────────

    def _make_callback(self, name: str, T: np.ndarray, color: tuple):
        def _cb(msg: dict):
            pts = decode_pointcloud2(msg, source=name)
            if pts is None or len(pts) == 0:
                return
            pts_tf = apply_transform(pts, T)
            if self._calibration_mode and self._voxel_size > 0:
                pts_tf = downsample_voxel(pts_tf, self._voxel_size)
            stamp  = msg.get("header", {}).get("stamp", {"sec": 0, "nanosec": 0})
            with self._lock:
                self._latest[name] = (pts_tf, color, stamp)
            self._rates[name].tick()
            self._republish(stamp)
        return _cb

    # ── republish ─────────────────────────────────────────────────────────────

    def _republish(self, stamp: dict):
        with self._lock:
            if self._raw_period > 0:
                now = time.time()
                if now - self._last_pub_time < self._raw_period:
                    return
                self._last_pub_time = now
            entries = [
                (v[0], v[1])
                for v in self._latest.values()
                if v is not None
            ]
        if not entries:
            return
        msg = encode_fused_cloud(entries, self._output_frame, stamp)
        self._publish_client.publish(self._output_topic, msg)

    # ── periodic rate log ─────────────────────────────────────────────────────

    def _rate_loop(self):
        while not self._stop.wait(timeout=5.0):
            parts = [f"{n}: {r.hz:.1f} Hz" for n, r in self._rates.items()]
            print(f"[rate]  {'  |  '.join(parts)}")

    # ── spin ──────────────────────────────────────────────────────────────────

    def spin(self):
        print("Running — press Ctrl+C to stop\n")
        try:
            while True:
                live = [s for s in self._sources.values() if s["ok"]]
                if live and all(not s["client"].is_connected for s in live):
                    print("[WARN] All connections lost — exiting")
                    break
                time.sleep(0.5)
        except KeyboardInterrupt:
            pass
        finally:
            self._stop.set()
            for s in self._sources.values():
                s["client"].close()
            print("Stopped.")


# ─── main ─────────────────────────────────────────────────────────────────────

if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Dual-LiDAR Overlay Node — calibration diagnostic",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    parser.add_argument(
        "--config",
        default="config/nodes_config.yaml",
        help="Path to nodes_config.yaml (default: config/nodes_config.yaml)",
    )
    parser.add_argument(
        "--raw",
        action="store_true",
        default=False,
        help=(
            "Calibration mode: subscribe to /livox/lidar (dense raw clouds with "
            "walls/floor/structure) instead of /livox/lidar_foreground, and publish "
            "into livox_frame so Foxglove renders it without a missing-TF error."
        ),
    )
    parser.add_argument(
        "--voxel",
        type=float,
        default=0.10,
        metavar="METERS",
        help=(
            "Voxel size (metres) for downsampling in --raw mode.  "
            "Smaller → more points; larger → fewer.  "
            "0.10 m typically yields 2–4 k pts/cloud.  "
            "Set to 0 to disable downsampling (may cause WebSocket drops).  "
            "Has no effect in normal (foreground) mode.  Default: 0.10"
        ),
    )
    parser.add_argument(
        "--raw-hz",
        type=float,
        default=3.0,
        metavar="HZ",
        help=(
            "Maximum fused-cloud publish rate (Hz) in --raw / calibration mode.  "
            "Calibration targets static structure so 2–3 Hz is more than enough.  "
            "Has no effect in normal (foreground) mode.  Default: 3.0"
        ),
    )
    parser.add_argument(
        "--debug",
        action="store_true",
        default=False,
        help="Enable per-frame [DBG _on_message] WebSocket logging (very verbose).",
    )
    args = parser.parse_args()

    try:
        import websocket  # noqa: F401
    except ImportError:
        print("ERROR: websocket-client not installed.  Run: pip install websocket-client")
        sys.exit(1)

    if args.debug:
        RosBridgeClient.debug = True

    node = OverlayNode(
        config_path=args.config,
        calibration_mode=args.raw,
        voxel_size=args.voxel,
        raw_hz=args.raw_hz,
    )
    node.spin()
