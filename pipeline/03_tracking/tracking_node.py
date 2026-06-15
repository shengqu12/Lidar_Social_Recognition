#!/usr/bin/env python3
"""
Tracking Node — AB3DMOT-style Multi-Object Tracking
=====================================================
Integrates detection and tracking in a single process:
  /livox/lidar_foreground
      -> ROI filter + frame accumulation + Euclidean clustering + shape filter
      -> AB3DMOT tracker (Kalman + Hungarian)
      -> /tracked_boxes   (MarkerArray: smoothed boxes + track ID labels)
      -> /tracked_centers (PointCloud2)
      -> data/tracklets/session_<starttime>.csv  (ATC format for Flack)

Detection is handled by importing from clustering_node.py (no round-trip
through rosbridge).  Tracking uses Kalman3D / Track / MultiObjectTracker3D
from the third-party AB3DMOT implementation.

Usage:
    conda activate livox
    python3 pipeline/03_tracking/tracking_node.py \\
        --config config/nodes_config.yaml \\
        --node node1
"""

import argparse
import base64
import colorsys
import csv
import json
import math
import os
import struct
import sys
import threading
import time
from collections import deque
from datetime import datetime
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import numpy as np

# ─── path setup ───────────────────────────────────────────────────────────────
# Allow importing clustering_node from the detection directory
_PIPELINE_ROOT = Path(__file__).resolve().parent.parent
_DETECTION_DIR = _PIPELINE_ROOT / "02_detection"
sys.path.insert(0, str(_DETECTION_DIR))

# Import detection primitives
from clustering_node import (
    apply_roi,
    euclidean_clustering,
    cluster_to_bbox,
    is_valid_person_cluster,
    check_vertical_extent,
    detect,
    RosBridgeClient,
)

# Import AB3DMOT tracker classes from third-party (read-only, never modified)
_THIRD_PARTY = (
    _PIPELINE_ROOT.parent
    / "third_party"
    / "O-LiPeDeT-Overhead-LiDAR-Person-Detection-and-Tracking"
    / "lidar-human-tracking"
    / "AB3DMOT"
)
sys.path.insert(0, str(_THIRD_PARTY))
from ab3dmot_tracking import Kalman3D, Track, MultiObjectTracker3D  # noqa: E402


# ─── Static zone filter (detection-level) ────────────────────────────────────

class StaticZoneFilter:
    """
    Filters out detections that appear consistently at the same location.

    Algorithm: maintain a rolling window of the last `history_frames` detection
    center lists.  A new detection is considered static if, in at least
    `density_threshold` fraction of the history frames, at least one past
    detection center falls within `radius` metres of it.

    This catches fixed furniture/equipment that the background model misses,
    without relying on track IDs (so track flickering doesn't reset state).
    """

    def __init__(self,
                 history_frames: int = 30,
                 radius: float = 0.5,
                 density_threshold: float = 0.75,
                 min_history: int = 15):
        self._history: deque = deque(maxlen=history_frames)
        self._radius    = float(radius)
        self._threshold = float(density_threshold)
        self._min_history = int(min_history)

    def filter(self, detections: List[dict]) -> List[dict]:
        """Return only detections that are NOT in a persistent static zone."""
        centers_now = [
            (float(d["center"][0]), float(d["center"][1]))
            for d in detections
        ]

        n_history = len(self._history)
        filtered = []

        for i, (cx, cy) in enumerate(centers_now):
            if n_history < self._min_history:
                filtered.append(detections[i])
            else:
                frames_with_nearby = 0
                for past_centers in self._history:
                    for px, py in past_centers:
                        if np.hypot(cx - px, cy - py) < self._radius:
                            frames_with_nearby += 1
                            break  # one match per historical frame is enough
                density = frames_with_nearby / n_history
                if density < self._threshold:
                    filtered.append(detections[i])

        # Record current centers AFTER filtering (only non-static detections
        # should seed future history — avoids reinforcing stale zones if the
        # room layout changes)
        self._history.append(centers_now)
        return filtered


# ─── Color helper ─────────────────────────────────────────────────────────────

_GOLDEN = 0.618033988749895

def track_color(track_id: int):
    """Deterministic, perceptually distinct RGB color for a track ID."""
    h = (track_id * _GOLDEN) % 1.0
    r, g, b = colorsys.hsv_to_rgb(h, 0.85, 0.95)
    return r, g, b


# ─── Direction-aware tracker ──────────────────────────────────────────────────

class DirectionAwareTracker(MultiObjectTracker3D):
    """
    Extends MultiObjectTracker3D with a motion-direction consistency penalty.

    When a track is moving faster than `direction_min_speed`, matches where
    the implied motion direction (track → detection) disagrees with the
    track's Kalman velocity are penalized.  This helps disambiguate two
    people crossing paths who are close in position but moving oppositely.

    Added cost term: direction_weight * (1 - cos_similarity(v_kalman, v_implied))
    The term is in [0, 2*direction_weight] — 0 for same direction, max for opposite.
    """

    def __init__(self,
                 direction_weight: float = 0.5,
                 direction_min_speed: float = 0.3,
                 **kwargs):
        super().__init__(**kwargs)
        self._dir_weight   = float(direction_weight)
        self._dir_min_spd  = float(direction_min_speed)

    def _build_cost(self, det_boxes, det_scores):
        cost = super()._build_cost(det_boxes, det_scores)

        if self._dir_weight == 0.0:
            return cost

        T = len(self.tracks)
        D = len(det_boxes)
        if T == 0 or D == 0:
            return cost

        for ti, trk in enumerate(self.tracks):
            vx    = float(trk.kf.x[3, 0])
            vy    = float(trk.kf.x[4, 0])
            speed = math.sqrt(vx * vx + vy * vy)
            if speed < self._dir_min_spd:
                continue  # no reliable direction for near-stationary tracks

            trk_x = float(trk.kf.x[0, 0])
            trk_y = float(trk.kf.x[1, 0])

            for dj in range(D):
                if cost[ti, dj] >= 1e5:
                    continue  # already gated out by Mahalanobis

                dx = float(det_boxes[dj][0]) - trk_x
                dy = float(det_boxes[dj][1]) - trk_y
                impl_norm = math.sqrt(dx * dx + dy * dy)
                if impl_norm < 1e-6:
                    continue

                cos_sim = (vx * dx + vy * dy) / (speed * impl_norm)
                # (1 - cos_sim) in [0, 2]: 0 = same direction, 2 = opposite
                cost[ti, dj] += self._dir_weight * (1.0 - float(cos_sim))

        return cost


# ─── Tracker wrapper ──────────────────────────────────────────────────────────

class LiveTracker:
    """
    Wraps DirectionAwareTracker with parameters tuned for overhead ceiling LiDAR:
    - Measurement variance set to reflect ~0.3 m bbox-center noise
    - Association gate corresponding to max_association_dist metres
    - IoU disabled (unreliable for sparse overhead blobs)
    - min_hits / max_age lifecycle from config
    - Static suppression: tracks that don't displace >= static_suppress_dist metres
      over static_suppress_frames frames are classified as furniture and removed
    - Direction-aware association: penalises cross-path mismatches
    """

    def __init__(self,
                 max_age: int = 5,
                 min_hits: int = 3,
                 max_association_dist: float = 1.0,
                 fps: float = 10.0,
                 static_suppress_frames: int = 20,
                 static_suppress_dist: float = 0.30,
                 direction_weight: float = 0.5,
                 direction_min_speed: float = 0.3,
                 coast_velocity_decay: float = 1.0):
        # Kalman dt from observed frame rate
        dt = 1.0 / max(1.0, fps)

        # Measurement variance: σ ≈ 0.35 m for bbox center from overhead LiDAR
        meas_var = 0.12

        # Mahalanobis gate: at convergence S ≈ R, so d² ≈ dist²/meas_var per axis.
        # For euclidean gate G metres in 2D: d² ≈ G²/meas_var (conservative).
        maha_gate = (max_association_dist ** 2) / meas_var * 1.5

        self._min_hits = int(min_hits)
        self._suppress_frames = int(static_suppress_frames)
        self._suppress_dist   = float(static_suppress_dist)
        self._coast_decay     = float(coast_velocity_decay)
        # position history per track: track_id -> deque of (x, y) arrays
        self._pos_history: dict = {}

        self._tracker = DirectionAwareTracker(
            direction_weight=direction_weight,
            direction_min_speed=direction_min_speed,
            dt=dt,
            max_age=max_age,
            min_hits=min_hits,
            init_delay=0,
            maha_gate=maha_gate,
            w_maha=1.0,
            w_iou=0.0,      # pure position-based — IoU unreliable from overhead
        )

        # Patch each future Track to use appropriate noise levels.
        # We monkey-patch the Kalman3D constructor via Track.__init__ overrides
        # by pre-setting the class-level defaults we need.
        self._dt = dt
        self._meas_var = meas_var

        # Override Kalman3D defaults for this session so newly created tracks
        # use correct noise; Track.__init__ calls Kalman3D(dt=self.dt) with no
        # extra kwargs, so we patch the class defaults.
        Kalman3D.__init__.__defaults__  # just verifying it exists
        self._orig_init = Kalman3D.__init__

        meas_var_ref = meas_var
        dt_ref = dt

        def _patched_init(self_kf,
                          dt=dt_ref,
                          process_var_pos=0.05,
                          process_var_vel=0.20,
                          meas_var_pos=meas_var_ref):
            self._orig_init(self_kf, dt, process_var_pos,
                            process_var_vel, meas_var_pos)

        Kalman3D.__init__ = _patched_init

    def step(self, detections: List[dict]) -> List[dict]:
        """
        Run one tracker step.

        Args:
            detections: list of bbox dicts from detect()

        Returns:
            list of track dicts: {id, center, size, vx, vy, age,
                                  hits, time_since_update}
        """
        if not detections:
            det_boxes  = np.zeros((0, 7), np.float32)
            det_scores = np.zeros((0,),   np.float32)
        else:
            det_boxes  = np.array([
                [d["center"][0], d["center"][1], d["center"][2],
                 d["size"][0],   d["size"][1],   d["size"][2],   0.0]
                for d in detections], dtype=np.float32)
            det_scores = np.ones(len(detections), dtype=np.float32)

        outputs = self._tracker.step(det_boxes, det_scores)

        # Dampen velocity of coasting (unmatched) tracks to prevent forward-drift
        # overshoot when a person stops. Kalman state: [x,y,z,vx,vy,vz] — vx
        # at index 3, vy at index 4. After step(), matched tracks have
        # time_since_update=0; coasting tracks have time_since_update>=1.
        if self._coast_decay < 1.0:
            for trk in self._tracker.tracks:
                if trk.time_since_update > 0:
                    trk.kf.x[3, 0] *= self._coast_decay
                    trk.kf.x[4, 0] *= self._coast_decay

        # Only output tracks that have been confirmed (hits >= min_hits).
        # The third-party tracker outputs all freshly-associated tracks regardless
        # of min_hits (via its init_delay=0 OR branch); we enforce min_hits here
        # so users never see flicker from single-frame ghost detections.
        outputs = [o for o in outputs if o["hits"] >= self._min_hits]

        raw_tracks = []
        for o in outputs:
            box   = o["box"]   # [x,y,z,dx,dy,dz,yaw]
            trk   = self._get_track(o["id"])
            vx    = float(trk.kf.x[3, 0]) if trk else 0.0
            vy    = float(trk.kf.x[4, 0]) if trk else 0.0
            raw_tracks.append({
                "id":                o["id"],
                "center":            np.array(box[:3], dtype=np.float32),
                "size":              np.array(box[3:6], dtype=np.float32),
                "vx":                vx,
                "vy":                vy,
                "age":               o["age"],
                "hits":              o["hits"],
                "time_since_update": o["time_since_update"],
            })

        # Update position history and apply static suppression.
        # A track whose maximum displacement over the last N positions is less
        # than the threshold is classified as a stationary object (furniture)
        # and excluded from output.
        active_ids = {t["id"] for t in raw_tracks}
        dead_ids   = set(self._pos_history.keys()) - active_ids
        for dead in dead_ids:
            del self._pos_history[dead]

        tracks = []
        for t in raw_tracks:
            tid = t["id"]
            xy  = t["center"][:2].copy()
            if tid not in self._pos_history:
                self._pos_history[tid] = deque(maxlen=self._suppress_frames)
            self._pos_history[tid].append(xy)

            hist = self._pos_history[tid]
            if len(hist) >= self._suppress_frames:
                positions    = np.array(list(hist))
                displacement = float(np.linalg.norm(positions - positions[0], axis=1).max())
                if displacement < self._suppress_dist:
                    continue  # static object — suppress

            tracks.append(t)

        return tracks

    def _get_track(self, track_id: int) -> Optional[object]:
        for trk in self._tracker.tracks:
            if trk.id == track_id:
                return trk
        return None


# ─── CSV logger ───────────────────────────────────────────────────────────────

class ATCLogger:
    """
    Appends trajectory rows to a session CSV in ATC format compatible with
    pipeline/04_encounter_detection/collision_detection.py:

    Columns 1-8 (ATC-compatible, read positionally by existing tools):
      timestamp, person_id, x, y, z, velocity, angle1, angle2
    Column 9 (new, ignored by legacy readers):
      behavior

      - timestamp : Unix milliseconds
      - x, y, z  : position in millimetres
      - velocity  : speed in mm/s
      - angle1    : movement direction in radians (atan2(vy, vx))
      - angle2    : same as angle1 (body facing unknown from overhead)
      - behavior  : one of walking/stationary/talking/unknown
    """

    def __init__(self, output_dir: Path):
        output_dir.mkdir(parents=True, exist_ok=True)
        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.path = output_dir / f"session_{ts}.csv"
        self._f = open(self.path, "w", newline="")
        self._w = csv.writer(self._f)
        print(f"[CSV] logging to {self.path}")

    def write(self, tracks: List[dict], wall_time: float,
              behaviors: Optional[Dict[int, str]] = None):
        ts_ms = int(wall_time * 1000)
        for t in tracks:
            vx, vy = t["vx"], t["vy"]
            speed  = float(np.hypot(vx, vy))
            angle  = float(np.arctan2(vy, vx))
            cx, cy, cz = t["center"]
            tid = int(t["id"])
            behavior = (behaviors or {}).get(tid, "unknown")
            self._w.writerow([
                ts_ms,
                tid,
                int(float(cx) * 1000),   # m → mm
                int(float(cy) * 1000),
                int(float(cz) * 1000),
                int(speed * 1000),        # m/s → mm/s
                round(angle, 4),
                round(angle, 4),
                behavior,                 # 9th column — safe for legacy readers
            ])
        self._f.flush()

    def close(self):
        self._f.close()


# ─── Behavior classifier ──────────────────────────────────────────────────────

# Behavior-dependent marker colors: (r, g, b)
_BEHAVIOR_COLORS = {
    "walking":    (1.0,  0.55, 0.0),   # orange
    "stationary": (0.2,  0.4,  1.0),   # blue
    "talking":    (1.0,  0.0,  0.0),   # red
    "unknown":    (0.65, 0.65, 0.65),  # gray
}


class BehaviorClassifier:
    """
    Per-track behavior classification using a sliding window of recent trajectory
    features.  Pure trajectory/geometry — no neural network, no training data.

    States:
      unknown    — warm-up period; fewer than min_window_frames of history
      walking    — mean speed over window > walk_speed_threshold (m/s)
      stationary — speed <= walk_speed_threshold (replaces sitting/standing;
                   z-extent is unreliable for far-away sparse point clouds)
      talking    — two stationary tracks within talk_distance_threshold (m)
                   for >= talk_duration_sec seconds (overrides stationary)

    Priority: talking > walking > stationary > unknown

    Hysteresis: a new label must persist for behavior_hold_frames consecutive
    frames before the displayed label switches.
    """

    def __init__(self, cfg: dict):
        self._enabled     = bool(cfg.get("enabled", True))
        self._window      = int(cfg.get("window_frames", 30))
        self._min_win     = int(cfg.get("min_window_frames", 15))
        self._walk_spd    = float(cfg.get("walk_speed_threshold", 0.5))
        self._talk_d      = float(cfg.get("talk_distance_threshold", 1.5))
        self._talk_dur    = float(cfg.get("talk_duration_sec", 30.0))
        self._hold_frames = int(cfg.get("behavior_hold_frames", 5))

        # per-track: deque of {x, y, z, z_extent, t}
        self._windows:   Dict[int, deque] = {}
        # per-track: (candidate_label, consecutive_count) for hysteresis
        self._candidate: Dict[int, Tuple[str, int]] = {}
        # per-track: currently displayed label (after hysteresis)
        self._label:     Dict[int, str] = {}
        # pair proximity tracking: frozenset({id1, id2}) -> wall_time first seen close
        self._close_since: Dict[frozenset, float] = {}
        # pairs currently classified as "talking"
        self._talking_pairs: set = set()
        # incremental path-length cache: avoids O(window) recompute each frame
        self._path_segs: Dict[int, deque] = {}   # segment lengths, maxlen=window-1
        self._path_lens: Dict[int, float] = {}   # running sum of current window segments

    def update(self, tracks: List[dict], wall_time: float) -> Dict[int, str]:
        """
        Update the classifier with this frame's confirmed tracks.

        Returns: dict mapping track_id -> behavior label string
        """
        if not self._enabled:
            return {t["id"]: "unknown" for t in tracks}

        active_ids = {t["id"] for t in tracks}

        # Clean up state for tracks that have disappeared
        for dead in set(self._windows) - active_ids:
            del self._windows[dead]
            self._candidate.pop(dead, None)
            self._label.pop(dead, None)
            self._path_segs.pop(dead, None)
            self._path_lens.pop(dead, None)

        dead_pairs = [p for p in self._close_since if not p.issubset(active_ids)]
        for p in dead_pairs:
            del self._close_since[p]
            self._talking_pairs.discard(p)

        # Update sliding windows with incremental path-length maintenance
        for t in tracks:
            tid = t["id"]
            if tid not in self._windows:
                self._windows[tid] = deque(maxlen=self._window)
                self._path_segs[tid] = deque(maxlen=max(1, self._window - 1))
                self._path_lens[tid] = 0.0
            win = self._windows[tid]
            if len(win) > 0:
                prev = win[-1]
                new_seg = math.hypot(float(t["center"][0]) - prev["x"],
                                     float(t["center"][1]) - prev["y"])
                segs = self._path_segs[tid]
                if len(segs) == segs.maxlen:
                    # oldest segment is about to be auto-dropped — subtract it first
                    self._path_lens[tid] -= segs[0]
                segs.append(new_seg)
                self._path_lens[tid] += new_seg
            self._windows[tid].append({
                "x":        float(t["center"][0]),
                "y":        float(t["center"][1]),
                "z":        float(t["center"][2]),
                "z_extent": float(t["size"][2]),
                "t":        wall_time,
            })

        # Compute raw label for each track
        raw: Dict[int, str] = {}
        for t in tracks:
            tid = t["id"]
            win = self._windows[tid]
            if len(win) < self._min_win:
                raw[tid] = "unknown"
                continue

            dt_total = win[-1]["t"] - win[0]["t"]
            if dt_total <= 0.0:
                raw[tid] = "unknown"
                continue

            # Mean speed using incremental path-length cache (O(1) per track)
            mean_speed = self._path_lens[tid] / dt_total

            if mean_speed > self._walk_spd:
                raw[tid] = "walking"
            else:
                raw[tid] = "stationary"

        # Pairwise talking detection — only among confirmed slow tracks
        track_map  = {t["id"]: t for t in tracks}
        slow_ids   = [tid for tid, lbl in raw.items()
                      if lbl == "stationary"]
        active_pairs: set = set()

        for i in range(len(slow_ids)):
            for j in range(i + 1, len(slow_ids)):
                id1, id2 = slow_ids[i], slow_ids[j]
                c1 = track_map[id1]["center"]
                c2 = track_map[id2]["center"]
                horiz_dist = math.hypot(float(c1[0]) - float(c2[0]),
                                        float(c1[1]) - float(c2[1]))
                if horiz_dist < self._talk_d:
                    pair = frozenset({id1, id2})
                    active_pairs.add(pair)
                    if pair not in self._close_since:
                        self._close_since[pair] = wall_time
                    elif wall_time - self._close_since[pair] >= self._talk_dur:
                        self._talking_pairs.add(pair)

        # Pairs that broke proximity → stop talking
        for pair in list(self._close_since.keys()):
            if pair not in active_pairs:
                del self._close_since[pair]
                self._talking_pairs.discard(pair)

        # Override raw labels: talking > everything else
        for pair in self._talking_pairs:
            for tid in pair:
                if tid in raw:
                    raw[tid] = "talking"

        # Apply hysteresis: label only switches after hold_frames consecutive frames
        result: Dict[int, str] = {}
        for t in tracks:
            tid = t["id"]
            new_raw = raw.get(tid, "unknown")

            if tid not in self._label:
                # First frame for this track — display immediately, no hysteresis
                self._label[tid]     = new_raw
                self._candidate[tid] = (new_raw, 1)
                result[tid] = new_raw
                continue

            cand_lbl, cand_cnt = self._candidate.get(tid, (new_raw, 0))
            if new_raw == cand_lbl:
                cand_cnt += 1
            else:
                cand_lbl = new_raw
                cand_cnt = 1
            self._candidate[tid] = (cand_lbl, cand_cnt)

            if cand_cnt >= self._hold_frames:
                self._label[tid] = cand_lbl

            result[tid] = self._label[tid]

        return result

    @staticmethod
    def color(behavior: str) -> Tuple[float, float, float]:
        return _BEHAVIOR_COLORS.get(behavior, _BEHAVIOR_COLORS["unknown"])

    def count_summary(self, behaviors: Dict[int, str]) -> str:
        """Short summary string for the periodic log line, e.g. 'walk=1 sit=2 talk=2'."""
        counts: Dict[str, int] = {}
        for lbl in behaviors.values():
            counts[lbl] = counts.get(lbl, 0) + 1
        parts = []
        for lbl in ("walking", "stationary", "talking", "unknown"):
            if counts.get(lbl, 0) > 0:
                parts.append(f"{lbl[:4]}={counts[lbl]}")
        return " ".join(parts)


# ─── Tracking Node ────────────────────────────────────────────────────────────

class TrackingNode:
    """
    Connects to Jetson rosbridge, subscribes to /livox/lidar_foreground,
    runs detection + AB3DMOT tracking, and publishes /tracked_boxes,
    /tracked_centers, and a CSV trajectory log.
    """

    def __init__(self,
                 jetson_ip: str,
                 port: int,
                 input_topic: str,
                 cluster_tol: float,
                 min_points: int,
                 max_points: int,
                 max_persons: int,
                 accum_frames: int,
                 roi_cfg: dict,
                 filter_cfg: dict,
                 tracking_cfg: dict,
                 behavior_cfg: dict,
                 csv_dir: Path):

        self.cluster_tol  = cluster_tol
        self.min_points   = min_points
        self.max_points   = max_points
        self.max_persons  = max_persons
        self.accum_frames = max(1, accum_frames)
        self.roi_cfg      = roi_cfg
        self.filter_cfg   = filter_cfg

        max_age         = int(tracking_cfg.get("max_age",   5))
        min_hits        = int(tracking_cfg.get("min_hits",  3))
        max_dist        = float(tracking_cfg.get("max_association_dist", 1.0))
        suppress_frames = int(tracking_cfg.get("static_suppress_frames", 20))
        suppress_dist   = float(tracking_cfg.get("static_suppress_dist",  0.30))
        do_csv          = bool(tracking_cfg.get("csv_logging", True))
        direction_w     = float(tracking_cfg.get("direction_weight", 0.5))
        direction_spd   = float(tracking_cfg.get("direction_min_speed", 0.3))
        coast_decay     = float(tracking_cfg.get("coast_velocity_decay", 1.0))

        self._frame_buf: deque = deque(maxlen=self.accum_frames)
        self._frame_count = 0
        self._rejected_count = 0
        self._vert_rejected_buf: list = []  # z-extents of furniture-rejected clusters

        # Estimate fps from first few frames
        self._fps_times: deque = deque(maxlen=10)
        self._fps = 10.0

        self.tracker = LiveTracker(
            max_age=max_age,
            min_hits=min_hits,
            max_association_dist=max_dist,
            fps=self._fps,
            static_suppress_frames=suppress_frames,
            static_suppress_dist=suppress_dist,
            direction_weight=direction_w,
            direction_min_speed=direction_spd,
            coast_velocity_decay=coast_decay,
        )

        self.static_filter = StaticZoneFilter(
            history_frames=int(tracking_cfg.get("static_zone_history", 30)),
            radius=float(tracking_cfg.get("static_zone_radius", 0.5)),
            density_threshold=float(tracking_cfg.get("static_zone_density", 0.75)),
            min_history=int(tracking_cfg.get("static_zone_min_history", 15)),
        )

        self.behavior_clf = BehaviorClassifier(behavior_cfg)

        self.csv_logger = ATCLogger(csv_dir) if do_csv else None

        # Keep a rolling set of track IDs we've output so we can clean up markers
        self._prev_track_ids: set = set()

        print(f"Connecting to rosbridge at {jetson_ip}:{port} ...")
        self.client = RosBridgeClient(host=jetson_ip, port=port)
        if not self.client.connect(timeout=10.0):
            raise RuntimeError(f"Failed to connect to rosbridge at {jetson_ip}:{port}.")
        print(f"Connected: {self.client.is_connected}")

        self.client.subscribe(
            topic=input_topic,
            msg_type="sensor_msgs/msg/PointCloud2",
            callback=self._callback,
            throttle_rate=0,
        )
        self.client.advertise("/tracked_boxes",   "visualization_msgs/msg/MarkerArray")
        self.client.advertise("/tracked_centers", "sensor_msgs/msg/PointCloud2")

        print("Tracking node ready")
        print(f"  Input:  {input_topic}")
        print(f"  Output: /tracked_boxes, /tracked_centers")
        print(f"  Tracker: max_age={max_age}  min_hits={min_hits}  "
              f"max_dist={max_dist}m  accum={self.accum_frames}frames")
        print(f"  Direction: weight={direction_w}  min_speed={direction_spd}m/s")
        if roi_cfg.get("enabled"):
            print(f"  ROI:    x[{roi_cfg['x_min']},{roi_cfg['x_max']}]  "
                  f"y[{roi_cfg['y_min']},{roi_cfg['y_max']}]")
        print(f"  Behavior: {'enabled' if behavior_cfg.get('enabled', True) else 'disabled'}")

    # ── callback ──────────────────────────────────────────────────────────────

    def _callback(self, msg: dict):
        t0 = time.time()

        pts = self._decode_pointcloud2(msg)
        if pts is None or len(pts) == 0:
            return

        # Track fps for logging
        self._fps_times.append(t0)
        if len(self._fps_times) >= 2:
            span = self._fps_times[-1] - self._fps_times[0]
            if span > 0:
                self._fps = (len(self._fps_times) - 1) / span

        # ROI crop before accumulation
        if self.roi_cfg.get("enabled"):
            pts = apply_roi(pts, self.roi_cfg)

        self._frame_buf.append(pts)
        if len(self._frame_buf) < self.accum_frames:
            return

        merged = np.vstack(list(self._frame_buf))

        # Detect (vertical rejections are accumulated in _vert_rejected_buf
        # and logged every 20 frames for furniture threshold tuning)
        detections = detect(
            merged,
            cluster_tol=self.cluster_tol,
            min_points=self.min_points,
            max_points=self.max_points,
            max_persons=self.max_persons,
            roi_cfg=None,       # already applied above
            filter_cfg=self.filter_cfg,
            _vert_rejected=self._vert_rejected_buf,
        )

        # Remove detections that are persistently at a static location (furniture)
        detections = self.static_filter.filter(detections)

        # Track
        tracks = self.tracker.step(detections)

        # Classify behaviors
        behaviors = self.behavior_clf.update(tracks, t0)

        header = msg.get("header", {})
        self._publish_tracked_boxes(tracks, behaviors, header)
        self._publish_tracked_centers(tracks, header)

        if self.csv_logger and tracks:
            self.csv_logger.write(tracks, t0, behaviors)

        dt = time.time() - t0
        self._frame_count += 1
        if self._frame_count % 20 == 0:
            active = len(tracks)
            bhv_summary = self.behavior_clf.count_summary(behaviors)
            vert_info = ""
            if self._vert_rejected_buf:
                sample = ", ".join(f"{z:.2f}" for z in self._vert_rejected_buf[-5:])
                vert_info = (f"  | vert_reject={len(self._vert_rejected_buf)} "
                             f"z=[{sample}]m")
            print(f"[frame {self._frame_count:>4}]  "
                  f"pts={len(merged):>5}  det={len(detections):>2}  "
                  f"tracks={active:>2}  fps={self._fps:.1f}  {dt*1000:.0f}ms"
                  + (f"  | {bhv_summary}" if bhv_summary else "")
                  + vert_info)
            self._vert_rejected_buf.clear()

    # ── decode ────────────────────────────────────────────────────────────────

    def _decode_pointcloud2(self, msg: dict) -> Optional[np.ndarray]:
        try:
            raw = base64.b64decode(msg.get("data", ""))
            fields = {f["name"]: f["offset"] for f in msg.get("fields", [])}
            step  = msg.get("point_step", 16)
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
            print(f"[WARN] decode: {e}")
            return None

    # ── publish markers ───────────────────────────────────────────────────────

    def _publish_tracked_boxes(self, tracks: List[dict],
                               behaviors: Dict[int, str], header: dict):
        stamp    = header.get("stamp",    {"sec": 0, "nanosec": 0})
        frame_id = header.get("frame_id", "livox_frame")
        markers  = []
        cur_ids  = set()

        for t in tracks:
            tid = t["id"]
            cur_ids.add(tid)
            c  = t["center"].tolist()
            s  = t["size"].tolist()

            behavior = behaviors.get(tid, "unknown")
            if behavior != "unknown":
                r, g, b = BehaviorClassifier.color(behavior)
            else:
                # warm-up or disabled — use per-ID color so boxes stay distinct
                r, g, b = track_color(tid)

            # Box marker — color driven by behavior
            markers.append({
                "header":    {"stamp": stamp, "frame_id": frame_id},
                "ns":        "tracked_persons",
                "id":        tid * 2,        # even ids = boxes
                "type":      1,              # CUBE
                "action":    0,              # ADD
                "pose": {
                    "position":    {"x": float(c[0]), "y": float(c[1]),
                                    "z": float(c[2])},
                    "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
                },
                "scale": {
                    "x": max(float(s[0]), 0.3),
                    "y": max(float(s[1]), 0.3),
                    "z": max(float(s[2]), 0.1),
                },
                "color":    {"r": r, "g": g, "b": b, "a": 0.45},
                "lifetime": {"sec": 0, "nanosec": 400000000},
            })

            # Text marker — ID + behavior label, same color as box
            label_z  = float(c[2]) + max(float(s[2]) / 2.0, 0.05) + 0.2
            coasting = t["time_since_update"] > 0
            label    = f"#{tid}{'*' if coasting else ''} {behavior}"
            markers.append({
                "header":    {"stamp": stamp, "frame_id": frame_id},
                "ns":        "tracked_persons",
                "id":        tid * 2 + 1,   # odd ids = text
                "type":      9,             # TEXT_VIEW_FACING
                "action":    0,
                "pose": {
                    "position":    {"x": float(c[0]), "y": float(c[1]),
                                    "z": label_z},
                    "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
                },
                "scale":    {"x": 0.0, "y": 0.0, "z": 0.25},
                "color":    {"r": r, "g": g, "b": b, "a": 0.95},
                "text":     label,
                "lifetime": {"sec": 0, "nanosec": 400000000},
            })

        # Delete markers for tracks that just died
        for old_id in self._prev_track_ids - cur_ids:
            for marker_id in (old_id * 2, old_id * 2 + 1):
                markers.append({
                    "header":    {"stamp": stamp, "frame_id": frame_id},
                    "ns":        "tracked_persons",
                    "id":        marker_id,
                    "type":      1, "action": 2,  # DELETE
                    "pose": {
                        "position":    {"x": 0.0, "y": 0.0, "z": 0.0},
                        "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
                    },
                    "scale": {"x": 0.1, "y": 0.1, "z": 0.1},
                    "color": {"r": 0.0, "g": 0.0, "b": 0.0, "a": 0.0},
                    "lifetime": {"sec": 0, "nanosec": 0},
                })

        self._prev_track_ids = cur_ids
        self.client.publish("/tracked_boxes", {"markers": markers})

    def _publish_tracked_centers(self, tracks: List[dict], header: dict):
        if not tracks:
            return
        stamp    = header.get("stamp",    {"sec": 0, "nanosec": 0})
        frame_id = header.get("frame_id", "livox_frame")

        raw = b""
        for t in tracks:
            c = t["center"]
            raw += struct.pack("<fff", float(c[0]), float(c[1]), float(c[2]))

        msg = {
            "header":      {"stamp": stamp, "frame_id": frame_id},
            "height":      1,
            "width":       len(tracks),
            "fields": [
                {"name": "x", "offset": 0,  "datatype": 7, "count": 1},
                {"name": "y", "offset": 4,  "datatype": 7, "count": 1},
                {"name": "z", "offset": 8,  "datatype": 7, "count": 1},
            ],
            "is_bigendian": False,
            "point_step":  12,
            "row_step":    12 * len(tracks),
            "data":        base64.b64encode(raw).decode("ascii"),
            "is_dense":    True,
        }
        self.client.publish("/tracked_centers", msg)

    # ── lifecycle ─────────────────────────────────────────────────────────────

    def spin(self):
        print("\nRunning... Press Ctrl+C to stop\n")
        try:
            while self.client.is_connected:
                time.sleep(0.1)
        except KeyboardInterrupt:
            pass
        finally:
            self.client.close()
            if self.csv_logger:
                self.csv_logger.close()
            print(f"\nStopped after {self._frame_count} frames")


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
        description="AB3DMOT Tracking Node",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    parser.add_argument("--config", type=str, default=None)
    parser.add_argument("--node",   type=str, default="node1")
    parser.add_argument("--jetson_ip",   type=str,   default=None)
    parser.add_argument("--port",        type=int,   default=None)
    parser.add_argument("--topic",       type=str,   default=None)
    parser.add_argument("--max_age",     type=int,   default=None)
    parser.add_argument("--min_hits",    type=int,   default=None)
    parser.add_argument("--max_assoc_dist", type=float, default=None)
    parser.add_argument("--coast_velocity_decay", type=float, default=None)
    parser.add_argument("--no_csv", action="store_true",
                        help="Disable CSV trajectory logging")
    args = parser.parse_args()

    jetson_ip = args.jetson_ip
    port      = args.port
    topic     = args.topic

    cfg_clustering = {}
    roi_cfg        = {}
    filter_cfg     = {}
    tracking_cfg   = {}
    behavior_cfg   = {}

    if args.config is not None:
        try:
            node_cfg = _load_node_config(args.config, args.node)
        except (FileNotFoundError, ValueError) as e:
            print(f"Config error: {e}")
            sys.exit(1)
        if jetson_ip is None:
            jetson_ip = node_cfg.get("jetson_ip", "172.26.42.167")
        if port is None:
            port = int(node_cfg.get("rosbridge_port", 9090))
        if topic is None:
            topic = node_cfg.get("foreground_topic", "/livox/lidar_foreground")
        cfg_clustering = node_cfg.get("clustering", {})
        roi_cfg        = node_cfg.get("roi",            {})
        filter_cfg     = node_cfg.get("cluster_filter", {})
        tracking_cfg   = node_cfg.get("tracking",       {})
        behavior_cfg   = node_cfg.get("behavior",       {})

    if jetson_ip is None:
        jetson_ip = "172.26.42.167"
    if port is None:
        port = 9090
    if topic is None:
        topic = "/livox/lidar_foreground"

    # CLI overrides for tracking params
    if args.max_age is not None:
        tracking_cfg["max_age"] = args.max_age
    if args.min_hits is not None:
        tracking_cfg["min_hits"] = args.min_hits
    if args.max_assoc_dist is not None:
        tracking_cfg["max_association_dist"] = args.max_assoc_dist
    if args.coast_velocity_decay is not None:
        tracking_cfg["coast_velocity_decay"] = args.coast_velocity_decay
    if args.no_csv:
        tracking_cfg["csv_logging"] = False

    cluster_tol  = float(cfg_clustering.get("cluster_tol",   0.4))
    min_points   = int(cfg_clustering.get("min_points",    8))
    max_points   = int(cfg_clustering.get("max_points",    800))
    max_persons  = int(cfg_clustering.get("max_persons",   10))
    accum_frames = int(cfg_clustering.get("accum_frames",  1))

    # CSV goes in project_root/data/tracklets/
    project_root = Path(__file__).resolve().parent.parent.parent
    csv_dir      = project_root / "data" / "tracklets"

    try:
        import websocket  # noqa: F401
    except ImportError:
        print("ERROR: pip install websocket-client")
        sys.exit(1)

    node = TrackingNode(
        jetson_ip=jetson_ip,
        port=port,
        input_topic=topic,
        cluster_tol=cluster_tol,
        min_points=min_points,
        max_points=max_points,
        max_persons=max_persons,
        accum_frames=accum_frames,
        roi_cfg=roi_cfg,
        filter_cfg=filter_cfg,
        tracking_cfg=tracking_cfg,
        behavior_cfg=behavior_cfg,
        csv_dir=csv_dir,
    )
    node.spin()
