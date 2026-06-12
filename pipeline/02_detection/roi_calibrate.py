#!/usr/bin/env python3
"""
ROI Calibration Tool
====================
Connects to rosbridge, collects 10 s of /livox/lidar_foreground points,
then prints the xyz range and a 2D text histogram of point density in
the XY plane so you can identify wall locations and propose ROI bounds.

Usage:
    python3 roi_calibrate.py [--jetson_ip 172.26.42.167] [--duration 10]
"""

import argparse
import base64
import json
import struct
import threading
import time

import numpy as np

try:
    import websocket
except ImportError:
    print("ERROR: pip install websocket-client")
    raise


# ─── Minimal WebSocket subscriber ────────────────────────────────────────────

class _WS:
    def __init__(self, url, topic, msg_type, on_pts):
        self._url = url
        self._topic = topic
        self._msg_type = msg_type
        self._on_pts = on_pts
        self._ready = threading.Event()

    def run(self):
        app = websocket.WebSocketApp(
            self._url,
            on_open=self._open,
            on_message=self._msg,
            on_error=lambda ws, e: print(f"[WS err] {e}"),
        )
        app.run_forever(ping_interval=20, ping_timeout=10)

    def _open(self, ws):
        ws.send(json.dumps({
            "op": "subscribe",
            "topic": self._topic,
            "type": self._msg_type,
            "throttle_rate": 0,
            "queue_length": 1,
        }))
        self._ready.set()

    def _msg(self, ws, raw):
        try:
            data = json.loads(raw)
        except Exception:
            return
        if data.get("op") == "publish" and data.get("topic") == self._topic:
            pts = _decode_pc2(data.get("msg", {}))
            if pts is not None and len(pts) > 0:
                self._on_pts(pts)


def _decode_pc2(msg):
    try:
        raw = base64.b64decode(msg.get("data", ""))
        fields = {f["name"]: f["offset"] for f in msg.get("fields", [])}
        step = msg.get("point_step", 16)
        width = msg.get("width", 0)
        if width == 0 or "x" not in fields:
            return None
        xo, yo, zo = fields["x"], fields["y"], fields["z"]
        pts = []
        for i in range(width):
            b = i * step
            x = struct.unpack_from("<f", raw, b + xo)[0]
            y = struct.unpack_from("<f", raw, b + yo)[0]
            z = struct.unpack_from("<f", raw, b + zo)[0]
            if np.isfinite(x) and np.isfinite(y) and np.isfinite(z):
                pts.append((x, y, z))
        return np.array(pts, dtype=np.float32) if pts else None
    except Exception as e:
        print(f"[decode err] {e}")
        return None


# ─── Histogram ────────────────────────────────────────────────────────────────

def text_histogram_2d(all_pts, grid=20):
    """
    Print a 20x20 ASCII density map in the XY plane.
    Rows = Y axis (top = max Y), Cols = X axis (left = min X).
    """
    xs = all_pts[:, 0]
    ys = all_pts[:, 1]

    x_min, x_max = xs.min(), xs.max()
    y_min, y_max = ys.min(), ys.max()

    # guard against degenerate range
    if x_max - x_min < 0.01:
        x_max = x_min + 0.01
    if y_max - y_min < 0.01:
        y_max = y_min + 0.01

    hist, y_edges, x_edges = np.histogram2d(
        ys, xs,
        bins=grid,
        range=[[y_min, y_max], [x_min, x_max]],
    )

    max_count = hist.max()
    if max_count == 0:
        print("  (no points)")
        return

    # Map count -> character: space, ·, :, !, #, █
    chars = " .·:!#█"
    def ch(count):
        if count == 0:
            return " "
        level = int((count / max_count) * (len(chars) - 1))
        return chars[level]

    print()
    print("  XY density map  (rows=Y top→bottom, cols=X left→right)")
    print(f"  X: {x_min:.2f} → {x_max:.2f} m     "
          f"Y: {y_min:.2f} → {y_max:.2f} m")
    print()

    # Column header: X labels
    col_labels = [f"{x_edges[i]:.1f}" for i in range(0, grid + 1, grid // 4)]
    print("  X:  " + "  ".join(f"{l:>6}" for l in col_labels))
    print("      " + "-" * (grid + 2))

    for row in range(grid - 1, -1, -1):  # top = max Y
        y_label = f"{y_edges[row]:.2f}"
        row_str = "".join(ch(hist[row, col]) for col in range(grid))
        print(f"  Y={y_label:>6} | {row_str} |")

    print("      " + "-" * (grid + 2))
    print(f"  max count per cell: {int(max_count)}")


# ─── Main ─────────────────────────────────────────────────────────────────────

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--jetson_ip", default="172.26.42.167")
    ap.add_argument("--port", type=int, default=9090)
    ap.add_argument("--topic", default="/livox/lidar_foreground")
    ap.add_argument("--duration", type=float, default=10.0,
                    help="Collection time in seconds (default 10)")
    args = ap.parse_args()

    url = f"ws://{args.jetson_ip}:{args.port}"
    all_pts = []
    frame_count = [0]
    lock = threading.Lock()

    def on_pts(pts):
        with lock:
            all_pts.append(pts)
            frame_count[0] += 1

    ws = _WS(url, args.topic, "sensor_msgs/msg/PointCloud2", on_pts)
    t = threading.Thread(target=ws.run, daemon=True)
    t.start()

    print(f"Connecting to {url} ...")
    time.sleep(1.5)
    print(f"Collecting {args.duration:.0f} s of '{args.topic}'")

    deadline = time.time() + args.duration
    while time.time() < deadline:
        remaining = deadline - time.time()
        frames = frame_count[0]
        print(f"\r  {frames:>4} frames | {remaining:.1f}s remaining  ", end="", flush=True)
        time.sleep(0.25)

    print()

    with lock:
        if not all_pts:
            print("ERROR: No points received. Is the Jetson pipeline running?")
            print(f"  Try: ssh kelrod@{args.jetson_ip} 'ros2 topic list'")
            return
        combined = np.vstack(all_pts)

    frames = frame_count[0]
    print(f"\nCollected {len(combined):,} points across {frames} frames\n")

    # ── XYZ ranges ──────────────────────────────────────────────────────────
    xs = combined[:, 0]
    ys = combined[:, 1]
    zs = combined[:, 2]

    print("=" * 62)
    print("POINT CLOUD EXTENT")
    print("=" * 62)
    print(f"  X:  {xs.min():+.3f} → {xs.max():+.3f} m   (range {xs.max()-xs.min():.3f} m)")
    print(f"  Y:  {ys.min():+.3f} → {ys.max():+.3f} m   (range {ys.max()-ys.min():.3f} m)")
    print(f"  Z:  {zs.min():+.3f} → {zs.max():+.3f} m   (range {zs.max()-zs.min():.3f} m)")

    p5x, p95x = np.percentile(xs, 5), np.percentile(xs, 95)
    p5y, p95y = np.percentile(ys, 5), np.percentile(ys, 95)
    p5z, p95z = np.percentile(zs, 5), np.percentile(zs, 95)
    print()
    print("  5th–95th percentile (ignores sparse edge noise):")
    print(f"  X:  {p5x:+.3f} → {p95x:+.3f} m")
    print(f"  Y:  {p5y:+.3f} → {p95y:+.3f} m")
    print(f"  Z:  {p5z:+.3f} → {p95z:+.3f} m")

    # ── 2-D histogram ────────────────────────────────────────────────────────
    print()
    print("=" * 62)
    print("XY DENSITY MAP  (all frames combined)")
    print("=" * 62)
    text_histogram_2d(combined, grid=20)

    # ── High-density edge bands (wall candidates) ────────────────────────────
    print()
    print("=" * 62)
    print("HIGH-DENSITY EDGE ANALYSIS (wall candidate identification)")
    print("=" * 62)

    def edge_density(axis_vals, label, bins=20):
        counts, edges = np.histogram(axis_vals, bins=bins)
        max_c = counts.max()
        threshold = max_c * 0.5
        hot = [(edges[i], edges[i+1], counts[i])
               for i in range(bins) if counts[i] >= threshold]
        print(f"\n  {label} bins with count >= 50% of peak (peak={int(max_c)}):")
        for lo, hi, c in hot:
            bar = "█" * int(20 * c / max_c)
            print(f"    {lo:+6.2f} → {hi:+6.2f} m : {c:>6}  {bar}")

    edge_density(xs, "X")
    edge_density(ys, "Y")

    print()
    print("=" * 62)
    print("SUGGESTED NEXT STEP")
    print("=" * 62)
    print("  Look at the XY map for dense bands along edges — those are walls.")
    print("  Set ROI x_min/x_max and y_min/y_max to exclude those bands.")
    print("  For z: typically z_min=-2.5, z_max=-0.5 keeps floor-to-head height")
    print("  relative to ceiling mount (foreground cloud is already z-filtered).")
    print()
    print("  Provide the confirmed values and I will write them to nodes_config.yaml.")


if __name__ == "__main__":
    main()
