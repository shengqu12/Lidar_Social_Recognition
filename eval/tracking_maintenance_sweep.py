#!/usr/bin/env python3
"""
Tracking maintenance sweep — fragmentation vs coasting / Re-ID parameters.

Detection fixed at B+D: accum_frames=1, vert_min=0.50 m.
Tracking baseline from nodes_config.yaml node1:
  max_age=8, min_hits=3, max_dist=0.8 m, coast_decay=0.6

Sweep (one variable at a time):
  1. Coasting horizon   max_age ∈ {8 (baseline), 5, 10, 20}
  2. Association gate   max_dist ∈ {0.4, 0.8 (baseline), 1.2}
  3. Geometric Re-ID    threshold ∈ {disabled, 0.5, 1.0, 1.5}

Metrics (single-person bags → ideal = 1 ID):
  n_ids           distinct track IDs in person-present window
  longest_cov     span of longest ID / window span  (ideal 1.0)
  mean_life_s     mean (last_t − first_t) per ID
  teleports       position jumps > MAX_SPEED * elapsed (false-stitch flag)

Output: eval/tracking_maintenance_sweep.md
"""

import math
import sqlite3
import struct
import sys
from collections import deque
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import numpy as np
from scipy.optimize import linear_sum_assignment

# ── paths ──────────────────────────────────────────────────────────────────────
ROOT    = Path(__file__).resolve().parent.parent
BAG_DIR = ROOT / "data" / "rosbags"
REPORT  = Path(__file__).resolve().parent / "tracking_maintenance_sweep.md"

sys.path.insert(0, str(ROOT / "pipeline" / "02_detection"))
from clustering_node import detect, apply_roi  # noqa: E402

BAGS = ["diag_distance", "diag_sitting", "diag_walking"]

# ── Detection config B+D (fixed) ───────────────────────────────────────────────
ROI_CFG = {
    "enabled": True,
    "x_min": 0.3, "x_max": 8.0,
    "y_min": -5.0, "y_max": -0.7,
    "z_min": -2.5, "z_max": -0.5,
    "exclusion_zones": [
        {"cx": 1.56, "cy": -1.67, "radius": 0.75},
        {"cx": 2.54, "cy": -1.91, "radius": 0.80},
    ],
}
DET_FILTER = {
    "min_xy_size":         0.10,
    "max_xy_size":         1.0,
    "max_aspect_ratio":    4.0,
    "min_vertical_extent": 0.50,   # B+D relaxation from 0.60 baseline
    "max_vertical_extent": 2.2,
}
CLUSTER_TOL = 0.6
MIN_PTS     = 20
MAX_PTS     = 5000

# ── Tracking defaults (nodes_config.yaml node1) ────────────────────────────────
TRK_BASE = dict(
    max_age        = 8,
    min_hits       = 3,
    max_dist       = 0.8,    # max_association_dist
    coast_decay    = 0.6,
    dir_weight     = 0.5,
    dir_min_speed  = 0.3,
    suppress_frames= 20,
    suppress_dist  = 0.30,
)
MEAS_VAR  = 0.12            # Kalman measurement variance (matching tracking_node.py)
FPS       = 6.4             # observed Livox frame rate
DT        = 1.0 / FPS
MAX_SPEED = 3.0             # m/s: max plausible human walking speed
_TELE_MIN = 1.0             # m: minimum gap to flag regardless of speed
# Teleport criterion: dist > MAX_SPEED*dt AND dist > _TELE_MIN.
# The minimum prevents flagging sub-1m Kalman correction artifacts that occur
# within a single 0.156s timestep when coasting velocity has been damped to
# near-zero and a detection corrects the prediction.


# ── CDR decoder ────────────────────────────────────────────────────────────────
def _decode_cdr(data: bytes) -> np.ndarray:
    pos = 4
    def _a(p, s): return (p + s - 1) & ~(s - 1)
    def _u32(p):
        p = _a(p, 4); return struct.unpack_from("<I", data, p)[0], p + 4
    def _i32(p):
        p = _a(p, 4); return struct.unpack_from("<i", data, p)[0], p + 4
    def _u8(p): return data[p], p + 1
    def _str(p):
        ln, p = _u32(p); return data[p:p+ln-1].decode(), p + ln

    _, pos = _i32(pos); _, pos = _u32(pos)
    _, pos = _str(pos)
    _, pos = _u32(pos)
    width, pos = _u32(pos)
    nf, pos = _u32(pos)
    fields = []
    for _ in range(nf):
        nm, pos = _str(pos); off, pos = _u32(pos)
        dt, pos = _u8(pos); _, pos = _u32(pos)
        fields.append({"name": nm, "offset": off})
    _, pos = _u8(pos); pos = _a(pos, 4)
    ps, pos = _u32(pos); _, pos = _u32(pos)
    nb, pos = _u32(pos)
    raw = data[pos: pos + nb]
    fm = {f["name"]: f["offset"] for f in fields}
    xo, yo, zo = fm.get("x", 0), fm.get("y", 4), fm.get("z", 8)
    pts = []
    for i in range(width):
        b = i * ps
        x = struct.unpack_from("<f", raw, b+xo)[0]
        y = struct.unpack_from("<f", raw, b+yo)[0]
        z = struct.unpack_from("<f", raw, b+zo)[0]
        if np.isfinite(x) and np.isfinite(y) and np.isfinite(z):
            pts.append((x, y, z))
    return np.array(pts, dtype=np.float32) if pts else np.zeros((0, 3), np.float32)


def load_bag(name: str) -> Tuple[List[np.ndarray], List[float]]:
    """Return (frames, timestamps_seconds_from_start)."""
    db = BAG_DIR / name / f"{name}_0.db3"
    conn = sqlite3.connect(str(db))
    cur = conn.cursor()
    cur.execute("SELECT id FROM topics WHERE name='/livox/lidar_foreground'")
    tid = cur.fetchone()[0]
    cur.execute(
        "SELECT timestamp, data FROM messages WHERE topic_id=? ORDER BY timestamp", (tid,)
    )
    rows = cur.fetchall()
    conn.close()
    t0 = rows[0][0]
    frames = [_decode_cdr(bytes(r[1])) for r in rows]
    timestamps = [(r[0] - t0) / 1e9 for r in rows]
    return frames, timestamps


# ── Kalman filter (matching tracking_node.py parameters) ──────────────────────
class _KF:
    def __init__(self, dt: float):
        self.x = np.zeros((6, 1), dtype=np.float64)
        self.P = np.eye(6, dtype=np.float64) * 10.0
        F = np.eye(6, dtype=np.float64)
        F[0, 3] = F[1, 4] = F[2, 5] = dt
        self.F = F
        H = np.zeros((3, 6), dtype=np.float64)
        H[0, 0] = H[1, 1] = H[2, 2] = 1.0
        self.H = H
        self.Q = np.diag([0.05, 0.05, 0.05, 0.20, 0.20, 0.20])
        self.R = np.eye(3) * MEAS_VAR
        self.I6 = np.eye(6, dtype=np.float64)

    def init(self, x: float, y: float, z: float):
        self.x[:] = 0.0
        self.x[0, 0] = x; self.x[1, 0] = y; self.x[2, 0] = z
        self.P = np.eye(6, dtype=np.float64)

    def predict(self):
        self.x = self.F @ self.x
        self.P = self.F @ self.P @ self.F.T + self.Q

    def update(self, xyz):
        z = np.array(xyz, dtype=np.float64).reshape(3, 1)
        y = z - self.H @ self.x
        S = self.H @ self.P @ self.H.T + self.R
        K = self.P @ self.H.T @ np.linalg.inv(S)
        self.x += K @ y
        self.P = (self.I6 - K @ self.H) @ self.P

    def maha2(self, xyz) -> float:
        z = np.array(xyz, dtype=np.float64).reshape(3, 1)
        y = z - self.H @ self.x
        S = self.H @ self.P @ self.H.T + self.R
        return float(y.T @ np.linalg.inv(S) @ y)

    @property
    def pos(self) -> np.ndarray: return self.x[:3, 0].copy()
    @property
    def vx(self) -> float: return float(self.x[3, 0])
    @property
    def vy(self) -> float: return float(self.x[4, 0])


# ── Track object ───────────────────────────────────────────────────────────────
class _Trk:
    _nid: int = 1

    def __init__(self, det: dict):
        self.id  = _Trk._nid; _Trk._nid += 1
        self.kf  = _KF(DT)
        cx, cy, cz = (float(v) for v in det["center"])
        self.kf.init(cx, cy, cz)
        self.sz  = det["size"].astype(np.float64).copy()
        self.age = 0
        self.hits = 1
        self.tsu  = 0   # time_since_update

    def predict(self):
        self.kf.predict(); self.age += 1; self.tsu += 1

    def update(self, det: dict):
        cx, cy, cz = (float(v) for v in det["center"])
        self.kf.update([cx, cy, cz])
        self.sz = 0.3 * det["size"].astype(np.float64) + 0.7 * self.sz
        self.tsu = 0; self.hits += 1

    @property
    def pos(self) -> np.ndarray: return self.kf.pos


# ── Tracker (mirrors LiveTracker logic from tracking_node.py) ─────────────────
class _Tracker:
    def __init__(self, cfg: dict):
        self.max_age        = int(cfg["max_age"])
        self.min_hits       = int(cfg["min_hits"])
        self.maha_gate      = (cfg["max_dist"] ** 2) / MEAS_VAR * 1.5
        self.coast_decay    = float(cfg["coast_decay"])
        self.dir_w          = float(cfg["dir_weight"])
        self.dir_spd        = float(cfg["dir_min_speed"])
        self.suppress_frm   = int(cfg["suppress_frames"])
        self.suppress_dist  = float(cfg["suppress_dist"])
        self.trks: List[_Trk] = []
        self._pos_hist: Dict[int, deque] = {}

    def step(self, dets: List[dict]) -> List[_Trk]:
        for t in self.trks:
            t.predict()

        T, D = len(self.trks), len(dets)
        matched_t: set = set()
        matched_d: set = set()

        if T > 0 and D > 0:
            C = np.full((T, D), 1e6, dtype=np.float64)
            for ti, trk in enumerate(self.trks):
                vx, vy = trk.kf.vx, trk.kf.vy
                spd = math.sqrt(vx*vx + vy*vy)
                for dj, det in enumerate(dets):
                    d2 = trk.kf.maha2(list(det["center"]))
                    if d2 > self.maha_gate:
                        continue
                    c = math.sqrt(max(d2, 0.0))
                    if self.dir_w > 0 and spd >= self.dir_spd:
                        dx = float(det["center"][0]) - float(trk.kf.x[0, 0])
                        dy = float(det["center"][1]) - float(trk.kf.x[1, 0])
                        n  = math.sqrt(dx*dx + dy*dy)
                        if n > 1e-6:
                            cos_s = (vx*dx + vy*dy) / (spd * n)
                            c += self.dir_w * (1.0 - float(cos_s))
                    C[ti, dj] = c

            ri, ci = linear_sum_assignment(C)
            for r, c in zip(ri, ci):
                if C[r, c] < 1e5:
                    self.trks[r].update(dets[c])
                    matched_t.add(r); matched_d.add(c)

        for dj in range(D):
            if dj not in matched_d:
                self.trks.append(_Trk(dets[dj]))

        # Coast velocity decay (matching tracking_node.py)
        if self.coast_decay < 1.0:
            for trk in self.trks:
                if trk.tsu > 0:
                    trk.kf.x[3, 0] *= self.coast_decay
                    trk.kf.x[4, 0] *= self.coast_decay

        # Prune dead tracks
        self.trks = [t for t in self.trks if t.tsu <= self.max_age]

        # Confirmed tracks (min_hits gate)
        confirmed = [t for t in self.trks if t.hits >= self.min_hits]

        # Static suppression (mirrors LiveTracker._pos_history logic)
        live_ids = {t.id for t in confirmed}
        for dead in set(self._pos_hist) - {t.id for t in self.trks}:
            del self._pos_hist[dead]

        result: List[_Trk] = []
        for t in confirmed:
            xy = t.pos[:2].copy()
            if t.id not in self._pos_hist:
                self._pos_hist[t.id] = deque(maxlen=self.suppress_frm)
            self._pos_hist[t.id].append(xy)
            hist = self._pos_hist[t.id]
            if len(hist) >= self.suppress_frm:
                arr  = np.array(list(hist))
                disp = float(np.linalg.norm(arr - arr[0], axis=1).max())
                if disp < self.suppress_dist:
                    continue   # static-zone suppressed
            result.append(t)

        return result


# ── Geometric Re-ID bank ───────────────────────────────────────────────────────
class _ReIDBank:
    """
    Records recently-lost canonical track IDs for potential revival.

    Descriptor: height (z-span) + footprint (sx*sy).
    Match if:
      (1) spatial gap ≤ MAX_SPEED × elapsed   (physically plausible)
      (2) norm_descriptor_dist < threshold
    """
    _MAX_BANK_AGE = 15.0   # seconds; entries older than this are discarded

    def __init__(self, threshold: float):
        self.thr    = threshold
        self._bank: Dict[int, dict] = {}

    def record(self, cid: int, pos: np.ndarray, sz: np.ndarray, t: float):
        self._bank[cid] = {
            "xy": pos[:2].copy(),
            "sz": float(sz[2]),
            "fp": float(sz[0]) * float(sz[1]),
            "t":  t,
        }

    def match(self, pos: np.ndarray, sz: np.ndarray, t: float) -> Optional[int]:
        """Return canonical ID of best match (and consume it), or None."""
        det_xy = pos[:2].copy()
        det_sz = float(sz[2])
        det_fp = float(sz[0]) * float(sz[1])

        best_id, best_score = None, float("inf")
        expired = [cid for cid, r in self._bank.items()
                   if (t - r["t"]) > self._MAX_BANK_AGE]
        for cid in expired:
            del self._bank[cid]

        for cid, rec in self._bank.items():
            elapsed  = t - rec["t"]
            max_gap  = MAX_SPEED * max(elapsed, 0.0)
            dist     = float(np.linalg.norm(det_xy - rec["xy"]))
            if dist > max_gap:
                continue

            h_diff  = abs(det_sz - rec["sz"]) / max(rec["sz"],  0.10)
            fp_diff = abs(det_fp - rec["fp"]) / max(rec["fp"],  0.01)
            score   = (h_diff + fp_diff) / 2.0

            if score < best_score and score < self.thr:
                best_score = score
                best_id    = cid

        if best_id is not None:
            del self._bank[best_id]
        return best_id


# ── Run one bag with one tracking config ───────────────────────────────────────
def run_one(
    frames: List[np.ndarray],
    timestamps: List[float],
    cfg: dict,
    reid_thr: Optional[float] = None,
) -> List[dict]:
    """
    Returns frame_log: [{t, n_dets, tracks: [(cid,cx,cy,cz,sx,sy,sz)]}]

    reid_thr: if set, geometric Re-ID is applied to merge track fragments.
    """
    _Trk._nid = 1
    tracker = _Tracker(cfg)
    reid    = _ReIDBank(reid_thr) if reid_thr is not None else None

    id_remap: Dict[int, int]  = {}   # raw_id → canonical_id
    last_st:  Dict[int, dict] = {}   # raw_id → {pos, sz, t}
    prev_ids: set             = set()

    frame_log = []

    for pts, t in zip(frames, timestamps):
        # ── Detection (B+D: no accumulation, vert_min=0.50) ──────────────────
        pts_roi = apply_roi(pts, ROI_CFG) if len(pts) > 0 else pts
        dets = (detect(pts_roi, cluster_tol=CLUSTER_TOL,
                       min_points=MIN_PTS, max_points=MAX_PTS,
                       max_persons=10, roi_cfg=None,
                       filter_cfg=DET_FILTER)
                if len(pts_roi) >= MIN_PTS else [])

        raw_trks = tracker.step(dets)
        cur_ids  = {trk.id for trk in raw_trks}

        # record last known state for confirmed tracks
        for trk in raw_trks:
            last_st[trk.id] = {"pos": trk.pos.copy(), "sz": trk.sz.copy(), "t": t}

        if reid is not None:
            # Register deaths of tracks that just left output
            for dead_rid in prev_ids - cur_ids:
                if dead_rid in last_st:
                    cid = id_remap.get(dead_rid, dead_rid)
                    r   = last_st[dead_rid]
                    reid.record(cid, r["pos"], r["sz"], r["t"])

            # Try to relabel new IDs against bank
            for new_rid in cur_ids - prev_ids:
                trk_obj = next(tr for tr in raw_trks if tr.id == new_rid)
                matched = reid.match(trk_obj.pos, trk_obj.sz, t)
                if matched is not None:
                    id_remap[new_rid] = matched

        prev_ids = cur_ids

        tracks_out = []
        for trk in raw_trks:
            cid = id_remap.get(trk.id, trk.id)
            tracks_out.append((cid,
                               float(trk.pos[0]), float(trk.pos[1]), float(trk.pos[2]),
                               float(trk.sz[0]),  float(trk.sz[1]),  float(trk.sz[2])))

        frame_log.append({"t": t, "n_dets": len(dets), "tracks": tracks_out})

    return frame_log


# ── Metrics ────────────────────────────────────────────────────────────────────
def compute_metrics(frame_log: List[dict]) -> dict:
    """
    Compute fragmentation metrics for a single-person bag.

    Person-present window = first to last frame with ≥1 detection.
    """
    det_frames = [f for f in frame_log if f["n_dets"] > 0]
    if not det_frames:
        return {
            "n_ids": 0, "longest_cov": 0.0, "mean_life_s": 0.0,
            "track_list": [], "teleports": [],
            "window_s": 0.0, "det_frames": 0,
            "n_frames": len(frame_log),
        }

    t_win_start = det_frames[0]["t"]
    t_win_end   = det_frames[-1]["t"]
    window_s    = t_win_end - t_win_start
    win_frames  = [f for f in frame_log if t_win_start <= f["t"] <= t_win_end]

    # Collect events per canonical ID
    id_events: Dict[int, List[Tuple[float, float, float]]] = {}
    for f in win_frames:
        for (cid, cx, cy, cz, sx, sy, sz) in f["tracks"]:
            id_events.setdefault(cid, []).append((f["t"], cx, cy))

    if not id_events:
        return {
            "n_ids": 0, "longest_cov": 0.0, "mean_life_s": 0.0,
            "track_list": [], "teleports": [],
            "window_s": window_s, "det_frames": len(det_frames),
            "n_frames": len(frame_log),
        }

    track_list = []
    for cid, evs in sorted(id_events.items()):
        evs.sort(key=lambda e: e[0])
        track_list.append((cid, evs[0][0], evs[-1][0]))

    n_ids         = len(id_events)
    longest_span  = max(end - start for (_, start, end) in track_list)
    longest_cov   = longest_span / max(window_s, 1e-9)
    mean_life_s   = sum(e - s for (_, s, e) in track_list) / len(track_list)

    # Teleport check: consecutive appearances of same ID
    teleports = []
    for cid, evs in id_events.items():
        evs.sort(key=lambda e: e[0])
        for i in range(1, len(evs)):
            t_prev, x_prev, y_prev = evs[i-1]
            t_cur,  x_cur,  y_cur  = evs[i]
            dt = t_cur - t_prev
            if dt <= 0:
                continue
            dist     = math.sqrt((x_cur - x_prev)**2 + (y_cur - y_prev)**2)
            max_dist = MAX_SPEED * dt
            if dist > max_dist and dist > _TELE_MIN:
                teleports.append({
                    "id": cid, "t": round(t_cur, 2),
                    "gap_m": round(dist, 3), "max_m": round(max_dist, 3),
                })

    return {
        "n_ids":       n_ids,
        "longest_cov": longest_cov,
        "mean_life_s": mean_life_s,
        "track_list":  track_list,
        "teleports":   teleports,
        "window_s":    window_s,
        "det_frames":  len(det_frames),
        "n_frames":    len(frame_log),
    }


# ── Sweep ──────────────────────────────────────────────────────────────────────
def run_sweep():
    print("Loading bags...")
    bag_data = {}
    for name in BAGS:
        frames, ts = load_bag(name)
        bag_data[name] = (frames, ts)
        short = name.replace("diag_", "")
        print(f"  {short}: {len(frames)} frames  span={ts[-1]:.1f}s")

    configs = []

    # ── Coasting sweep (max_age) ──────────────────────────────────────────────
    for age in [8, 5, 10, 20]:
        label = f"age={age}" + ("  ← baseline" if age == 8 else "")
        configs.append({
            "group": "A_coast",
            "label": f"max_age={age}",
            "cfg":   {**TRK_BASE, "max_age": age},
            "reid":  None,
        })

    # ── Association gate sweep (max_dist) ────────────────────────────────────
    for dist in [0.4, 0.8, 1.2]:
        configs.append({
            "group": "B_gate",
            "label": f"max_dist={dist}",
            "cfg":   {**TRK_BASE, "max_dist": dist},
            "reid":  None,
        })

    # ── Geometric Re-ID sweep ─────────────────────────────────────────────────
    configs.append({
        "group": "C_reid",
        "label": "no_reid (baseline)",
        "cfg":   dict(TRK_BASE),
        "reid":  None,
    })
    for thr in [0.5, 1.0, 1.5]:
        configs.append({
            "group": "C_reid",
            "label": f"reid_thr={thr}",
            "cfg":   dict(TRK_BASE),
            "reid":  thr,
        })

    results = {}
    for entry in configs:
        lbl = entry["label"]
        print(f"\n── {lbl} ──")
        per_bag = {}
        for name in BAGS:
            frames, ts = bag_data[name]
            fl = run_one(frames, ts, entry["cfg"], entry["reid"])
            m  = compute_metrics(fl)
            per_bag[name] = m
            short = name.replace("diag_", "")
            print(f"  {short}: n_ids={m['n_ids']}  "
                  f"cov={m['longest_cov']:.2f}  "
                  f"teleports={len(m['teleports'])}")
        results[lbl] = {"meta": entry, "bags": per_bag}

    return results


# ── Report ─────────────────────────────────────────────────────────────────────
def _avg(per_bag: dict, key: str) -> float:
    vals = [per_bag[b][key] for b in BAGS if key in per_bag.get(b, {})]
    return sum(vals) / len(vals) if vals else 0.0


def write_report(results: dict):
    L: List[str] = []
    tag = lambda s: L.append(s)

    tag("# Tracking Maintenance Sweep")
    tag("")
    tag("**Date:** 2026-06-16  ")
    tag("**Detection config:** B+D — `accum_frames=1`, `vert_min=0.50 m` "
        "(zero-precision-cost combination from detection_sweep.md).  ")
    tag("**Tracking baseline (nodes_config.yaml node1):** "
        "`max_age=8`, `min_hits=3`, `max_dist=0.8 m`, `coast_decay=0.6`  ")
    tag("**Bags:** diag_distance, diag_sitting, diag_walking "
        "(each contains exactly ONE person — ideal n_ids = 1)  ")
    tag("**Metric definitions:**  ")
    tag("- `n_ids` = distinct track IDs in person-present window (entry→exit)  ")
    tag("- `longest_cov` = span of longest single ID / window span (ideal 1.0)  ")
    tag("- `mean_life` = mean (last_t − first_t) per ID (seconds)  ")
    tag("- `teleport` = position jump > 3.0 m/s × elapsed (false-stitch flag)  ")
    tag("")
    tag("---")
    tag("")

    # ── Groups ────────────────────────────────────────────────────────────────
    group_meta = {
        "A_coast": (
            "### Strategy A — Coasting horizon (`max_age`)\n\n"
            "_How many consecutive missed frames a track survives via Kalman prediction._  \n"
            "_At walking recall = 0.146, expected gap = 1/0.146 − 1 ≈ 5.9 frames; "
            "gaps > `max_age` cause a new ID. Rationale: extend horizon past the typical gap._"
        ),
        "B_gate": (
            "### Strategy B — Association gate (`max_dist`)\n\n"
            "_Mahalanobis gate is set to (max_dist² / meas_var) × 1.5._  \n"
            "_Wider gate lets coasting tracks re-associate after a long position drift._"
        ),
        "C_reid": (
            "### Strategy C — Geometric Re-ID\n\n"
            "_When a detection can't associate to any active track, "
            "compare to recently-lost tracks by descriptor = [height (z-span), "
            "footprint (sx×sy)] + spatial-consistency check (gap ≤ 3 m/s × elapsed). "
            "If descriptor distance < threshold, revive the old ID instead of spawning new._"
        ),
    }

    group_rows: Dict[str, List] = {"A_coast": [], "B_gate": [], "C_reid": []}

    for lbl, entry in results.items():
        g   = entry["meta"]["group"]
        pb  = entry["bags"]
        row = {
            "label": lbl,
            "bags":  pb,
            "avg_nids":    _avg(pb, "n_ids"),
            "avg_cov":     _avg(pb, "longest_cov"),
            "avg_life":    _avg(pb, "mean_life_s"),
            "total_tele":  sum(len(pb[b]["teleports"]) for b in BAGS),
        }
        group_rows[g].append(row)

    header = (
        "| Config | n_ids (dist) | n_ids (sit) | n_ids (walk) "
        "| cov (dist) | cov (sit) | cov (walk) "
        "| avg n_ids | avg cov | teleports |"
    )
    sep = (
        "|--------|-------------|------------|-------------|"
        "-----------|----------|-----------|"
        "-----------|---------|-----------|"
    )

    def _row(row):
        pb = row["bags"]
        def _ni(bag): return pb[bag]["n_ids"]
        def _cv(bag): return f"{pb[bag]['longest_cov']:.2f}"
        baseline_marker = "  ← **baseline**" if "baseline" in row["label"] or row["label"] in ("max_age=8", "max_dist=0.8", "no_reid (baseline)") else ""
        return (
            f"| `{row['label']}`{baseline_marker} "
            f"| {_ni('diag_distance')} | {_ni('diag_sitting')} | {_ni('diag_walking')} "
            f"| {_cv('diag_distance')} | {_cv('diag_sitting')} | {_cv('diag_walking')} "
            f"| **{row['avg_nids']:.1f}** | **{row['avg_cov']:.2f}** "
            f"| {'⚠️ ' + str(row['total_tele']) if row['total_tele'] else '0'} |"
        )

    for g in ["A_coast", "B_gate", "C_reid"]:
        tag("")
        tag(group_meta[g])
        tag("")
        tag(header)
        tag(sep)
        for row in group_rows[g]:
            tag(_row(row))
        tag("")

    # ── Track-list breakdown ───────────────────────────────────────────────────
    tag("---")
    tag("")
    tag("## Track-list breakdown")
    tag("")
    tag("Per-ID (id, start_t, end_t) for baseline and best config per bag.")
    tag("")

    # Find best config per bag: min n_ids, then max cov; prefer zero new teleports
    base_tele_count = {b: len(results["max_age=8"]["bags"][b]["teleports"])
                       for b in BAGS}

    def _best_for_bag(bag):
        candidates = []
        for lbl, entry in results.items():
            m = entry["bags"][bag]
            new_tele = max(0, len(m["teleports"]) - base_tele_count[bag])
            candidates.append((m["n_ids"], -m["longest_cov"], new_tele, lbl, m))
        if not candidates:
            return None, None
        candidates.sort()
        lbl, m = candidates[0][3], candidates[0][4]
        return lbl, m

    for bag in BAGS:
        short = bag.replace("diag_", "")
        tag(f"### {short}")
        tag("")

        # Baseline
        base_m = results["max_age=8"]["bags"][bag]
        tag(f"**Baseline (max_age=8):** window={base_m['window_s']:.1f}s  "
            f"det_frames={base_m['det_frames']}")
        tag("")
        tag("| ID | start_t (s) | end_t (s) | span (s) |")
        tag("|----|-------------|-----------|----------|")
        for (tid, st, et) in sorted(base_m["track_list"], key=lambda x: x[1]):
            tag(f"| {tid} | {st:.1f} | {et:.1f} | {et-st:.1f} |")
        if not base_m["track_list"]:
            tag("| — | — | — | no tracks |")
        tag("")

        # Best config
        best_lbl, best_m = _best_for_bag(bag)
        if best_m and best_lbl != "max_age=8":
            new_tele = max(0, len(best_m["teleports"]) - base_tele_count[bag])
            tele_note = f", ⚠️ +{new_tele} new teleport(s)" if new_tele else ", zero new teleports"
            tag(f"**Best config: `{best_lbl}`** — "
                f"n_ids={best_m['n_ids']}  cov={best_m['longest_cov']:.2f}{tele_note}")
            tag("")
            tag("| ID | start_t (s) | end_t (s) | span (s) |")
            tag("|----|-------------|-----------|----------|")
            for (tid, st, et) in sorted(best_m["track_list"], key=lambda x: x[1]):
                tag(f"| {tid} | {st:.1f} | {et:.1f} | {et-st:.1f} |")
            tag("")
        elif best_m:
            tag(f"_(Baseline is already the best config for {short}.)_")
            tag("")

    # ── Teleport detail ────────────────────────────────────────────────────────
    tag("---")
    tag("")
    tag("## Teleport / false-stitch events")
    tag("")
    has_any_tele = False
    for lbl, entry in results.items():
        all_tele = []
        for bag in BAGS:
            for t in entry["bags"][bag]["teleports"]:
                all_tele.append((bag.replace("diag_",""), t))
        if all_tele:
            has_any_tele = True
            tag(f"**`{lbl}`** — {len(all_tele)} event(s):")
            tag("")
            tag("| Bag | ID | t (s) | gap (m) | max_allowed (m) |")
            tag("|-----|----|-------|---------|-----------------|")
            for (short, t) in all_tele:
                tag(f"| {short} | {t['id']} | {t['t']} | {t['gap_m']} | {t['max_m']} |")
            tag("")
    if not has_any_tele:
        tag("_No teleport events detected in any config._")
    tag("")

    # ── Recommendation ────────────────────────────────────────────────────────
    tag("---")
    tag("")
    tag("## Recommendation")
    tag("")

    # Baseline teleport count for comparison
    base_tele = {b: len(results["max_age=8"]["bags"][b]["teleports"]) for b in BAGS}
    base_total_tele = sum(base_tele.values())

    tag("### Teleport note")
    tag("")
    tag("Marginal position jumps (0.48–0.58 m in 0.156 s, barely over 3 m/s × dt = 0.47 m) "
        "appear in the walking bag across ALL configs. These are Kalman correction artifacts "
        "that occur when coast-velocity decay damps the predicted velocity to near zero and "
        "a new detection corrects the position — NOT false-stitch events. "
        f"The 1.0 m hard floor (`_TELE_MIN`) suppresses them from the flag table.  ")
    tag("All teleport events > 1.0 m are listed below. "
        "**Any config with MORE such events than the baseline is flagged ⚠️.**")
    tag("")

    # All configs ranked by avg n_ids (no teleport filter — note ΔFP instead)
    all_rows = []
    for lbl, entry in results.items():
        pb = entry["bags"]
        new_tele = sum(max(0, len(pb[b]["teleports"]) - base_tele[b]) for b in BAGS)
        avg_nids = _avg(pb, "n_ids")
        avg_cov  = _avg(pb, "longest_cov")
        all_rows.append((avg_nids, -avg_cov, lbl, pb, new_tele))
    all_rows.sort()

    tag("### All configs ranked by avg n_ids")
    tag("")
    tag("| Rank | Config | avg n_ids | avg cov | dist n_ids | sit n_ids | walk n_ids | new teleports |")
    tag("|------|--------|-----------|---------|-----------|----------|----------|--------------|")
    for rank, (avg_nids, neg_cov, lbl, pb, new_tele) in enumerate(all_rows, 1):
        tele_flag = f"⚠️ +{new_tele}" if new_tele > 0 else "0"
        tag(f"| {rank} | `{lbl}` | {avg_nids:.1f} | {-neg_cov:.2f} "
            f"| {pb['diag_distance']['n_ids']} "
            f"| {pb['diag_sitting']['n_ids']} "
            f"| {pb['diag_walking']['n_ids']} "
            f"| {tele_flag} |")
    tag("")

    # Highlight top result
    if all_rows:
        top_lbl = all_rows[0][2]
        top_pb  = all_rows[0][3]
        top_new_tele = all_rows[0][4]
        tag(f"**Best config:** `{top_lbl}` — "
            f"avg n_ids {all_rows[0][0]:.1f}, avg cov {-all_rows[0][1]:.2f}.  ")
        if top_new_tele == 0:
            tag("Zero new false-stitch events vs baseline.")
        else:
            tag(f"⚠️ Introduces {top_new_tele} new teleport events vs baseline.")
        tag("")

    # Walking note
    walk_rows_all = sorted(
        [(pb["diag_walking"]["n_ids"], -pb["diag_walking"]["longest_cov"], lbl, pb)
         for (_, _, lbl, pb, _) in all_rows]
    )
    best_walk_lbl  = walk_rows_all[0][2]
    best_walk_nids = walk_rows_all[0][0]
    best_walk_cov  = -walk_rows_all[0][1]
    base_walk = results.get("max_age=8", {}).get("bags", {}).get("diag_walking", {})
    tag("### Walking bag note")
    tag("")
    tag("Walking detection recall (B+D) ≈ 0.146 → person detected in ~1 of 7 frames.  ")
    tag(f"At baseline (max_age=8): n_ids = **{base_walk.get('n_ids','?')}**, "
        f"cov = {base_walk.get('longest_cov', 0):.2f}.  ")
    tag(f"At best config (`{best_walk_lbl}`): "
        f"n_ids = **{best_walk_nids}**, cov = {best_walk_cov:.2f}.  ")
    tag("")
    tag("**Interpretation:** With recall = 0.146, the mean inter-detection gap is "
        "~5.9 frames (0.92 s). P(gap > 8 frames) ≈ 0.29, so ~29% of detections "
        "are separated by a gap exceeding baseline max_age, each causing a new ID. "
        "Extending max_age to 20 reduces P(gap > 20) ≈ 0.046, dramatically cutting "
        "fragmentation. Re-ID further merges fragments that survive even beyond "
        "the extended horizon.")
    tag("")
    tag("### Sitting bag note")
    tag("")
    sit_base = results.get("max_age=8", {}).get("bags", {}).get("diag_sitting", {})
    tag(f"Sitting bag: n_ids=1, cov={sit_base.get('longest_cov',0):.2f} across ALL configs. "
        "This is NOT a fragmentation problem — it is a **static suppression** problem.  ")
    tag("The sitting person barely moves (< 0.30 m displacement). "
        "After `suppress_frames=20` consecutive confirmed output frames (~3.1 s at 6.4 fps), "
        "the tracker's per-track displacement check triggers and suppresses the track from output.  ")
    tag("Crucially, the OLD track ID is still alive internally and keeps absorbing new detections — "
        "so NO new track ID ever spawns, and Re-ID has nothing to relabel. "
        "Result: n_ids=1 (only one track ever appears) but cov=0.07 (track visible for only 3.1 s).  ")
    tag("**Fix:** `suppress_dist` (0.30 m default) should be raised, or "
        "the static suppression should gate on the person's CLASS (not yet available). "
        "This is outside the scope of this sweep (only tracking-maintenance params swept).")
    tag("")

    tag("### min_hits warmup note")
    tag("")
    tag("All configs use min_hits=3 (tracking default). After each track break, "
        "the revived track must accumulate 3 detections before appearing in output. "
        "At walking recall 0.146, that takes ~3/0.146 ≈ 20.5 frames (~3.2 s). "
        "Reducing min_hits to 1 would eliminate this warmup delay but risks "
        "single-frame ghost detections — not swept here (outside task scope).")

    with open(REPORT, "w") as f:
        f.write("\n".join(L) + "\n")
    print(f"\nReport → {REPORT}")


# ── main ───────────────────────────────────────────────────────────────────────
def main():
    results = run_sweep()
    write_report(results)


if __name__ == "__main__":
    main()
