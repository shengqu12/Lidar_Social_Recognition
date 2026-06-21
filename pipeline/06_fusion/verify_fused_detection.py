#!/usr/bin/env python3
"""
Offline synthetic test — fused two-LiDAR detection
====================================================
No network, no ROS, no Jetsons required.

TEST 1 (good calibration): union of node1+node3 foreground points for a person
in the overlap region produces exactly ONE cluster, plus a node3-only decoy
produces one more — total 2 detections.

TEST 2 (bad calibration counter-test): same point counts but node3's
contribution is shifted 0.70 m edge-to-edge from node1's (> cluster_tol 0.60 m).
The overlap person splits into 2 separate clusters. This confirms the main test
is actually checking the merge behaviour.

Run:
    python3 pipeline/06_fusion/verify_fused_detection.py
"""

import sys
import pathlib
import numpy as np

# ── locate clustering_node ────────────────────────────────────────────────────
_PIPELINE_ROOT = pathlib.Path(__file__).resolve().parent.parent
sys.path.insert(0, str(_PIPELINE_ROOT / "02_detection"))
from clustering_node import detect  # noqa: E402

# ── config values from nodes_config.yaml (fused node) ────────────────────────
ROI_CFG = {
    "enabled": True,
    "x_min": 0.3,  "x_max": 8.0,
    "y_min": -5.0, "y_max": -0.7,
    "z_min": -2.5, "z_max": -0.5,
    # exclusion_zones omitted — test points are 3+ m from those zones
}
FILTER_CFG = {
    "min_xy_size":         0.10,
    "max_xy_size":         1.0,
    "max_aspect_ratio":    4.0,
    "min_vertical_extent": 0.50,
    "max_vertical_extent": 2.2,
}
CLUSTER_TOL = 0.6
MIN_POINTS   = 20
MAX_POINTS   = 5000
MAX_PERSONS  = 10


# ── helpers ───────────────────────────────────────────────────────────────────

def make_points(cx, cy, cz, n, xy_spread=0.2, z_spread=0.4, rng=None):
    """Return (n, 3) float32 array uniformly scattered around (cx, cy, cz)."""
    if rng is None:
        rng = np.random.default_rng(0)
    x = rng.uniform(cx - xy_spread, cx + xy_spread, n)
    y = rng.uniform(cy - xy_spread, cy + xy_spread, n)
    z = rng.uniform(cz - z_spread,  cz + z_spread,  n)
    return np.column_stack([x, y, z]).astype(np.float32)


def run_detect(pts):
    vert_rej = []
    dets = detect(
        pts,
        cluster_tol=CLUSTER_TOL,
        min_points=MIN_POINTS,
        max_points=MAX_POINTS,
        max_persons=MAX_PERSONS,
        roi_cfg=ROI_CFG,
        filter_cfg=FILTER_CFG,
        _vert_rejected=vert_rej,
    )
    return dets, vert_rej


def near(det, cx, cy, tol=1.0):
    c = det["center"]
    return abs(c[0] - cx) < tol and abs(c[1] - cy) < tol


def check_shape(det):
    s = det["size"]
    sx, sy, sz = float(s[0]), float(s[1]), float(s[2])
    return (
        FILTER_CFG["min_xy_size"] <= sx <= FILTER_CFG["max_xy_size"] and
        FILTER_CFG["min_xy_size"] <= sy <= FILTER_CFG["max_xy_size"] and
        FILTER_CFG["min_vertical_extent"] <= sz <= FILTER_CFG["max_vertical_extent"]
    )


# ── TEST 1: well-calibrated fusion ────────────────────────────────────────────

def test_good_calibration():
    print("=" * 62)
    print("TEST 1: Well-calibrated fusion  (overlap person → 1 cluster)")
    print("=" * 62)
    rng = np.random.default_rng(42)

    OX, OY, OZ = 4.5, -2.5, -1.6   # overlap person centre (node1 frame)

    # node1 sees the person: 40 points ± 0.20 m XY, ± 0.40 m Z
    n1_pts = make_points(OX, OY, OZ, 40, xy_spread=0.20, z_spread=0.40, rng=rng)
    # node3 sees the same person (already transformed): 30 pts ± 0.25 m XY
    n3_pts = make_points(OX, OY, OZ, 30, xy_spread=0.25, z_spread=0.40, rng=rng)
    # decoy: second person seen only by node3
    dc_pts = make_points(6.0, -3.5, OZ, 30, xy_spread=0.20, z_spread=0.40, rng=rng)

    all_pts = np.vstack([n1_pts, n3_pts, dc_pts])
    dets, vert_rej = run_detect(all_pts)

    print(f"  Input:  {len(all_pts)} pts  "
          f"(n1={len(n1_pts)}, n3={len(n3_pts)}, decoy={len(dc_pts)})")
    print(f"  Output: {len(dets)} detection(s)")
    if vert_rej:
        print(f"  Vert-rejected z-extents (m): {[round(v, 3) for v in vert_rej]}")
    for i, d in enumerate(dets):
        c, s = d["center"], d["size"]
        region = "OVERLAP" if near(d, OX, OY) else "decoy"
        print(f"    #{i+1}: center=({c[0]:.2f}, {c[1]:.2f})  "
              f"size=({s[0]:.2f} x {s[1]:.2f} x {s[2]:.2f})  "
              f"n_pts={d['n_points']}  [{region}]")

    overlap_dets = [d for d in dets if near(d, OX, OY)]
    p_total   = len(dets) == 2
    p_overlap = len(overlap_dets) == 1
    p_shape   = all(check_shape(d) for d in dets)

    print()
    print(f"  [{'PASS' if p_total   else 'FAIL'}] Total detections == 2: got {len(dets)}")
    print(f"  [{'PASS' if p_overlap else 'FAIL'}] Overlap person → exactly 1 cluster: "
          f"got {len(overlap_dets)}")
    print(f"  [{'PASS' if p_shape   else 'FAIL'}] All detections pass shape filters")
    ok = p_total and p_overlap and p_shape
    print(f"\n  TEST 1 RESULT: {'PASS' if ok else 'FAIL'}")
    return ok


# ── TEST 2: misaligned / bad-calibration counter-test ─────────────────────────

def test_bad_calibration():
    """
    Same counts as test 1 but node3's contribution is shifted so that the
    edge-to-edge gap in X is 0.70 m > cluster_tol (0.60 m).

    node1 group: x ∈ [4.30, 4.70]  (centre 4.50, spread ±0.20)
    node3 group: x ∈ [5.40, 5.90]  (centre 5.65, spread ±0.25)
    edge gap = 5.40 - 4.70 = 0.70 m > cluster_tol 0.60 m

    Expected: 3 detections (overlap person splits into 2 + decoy).
    """
    print()
    print("=" * 62)
    print("TEST 2: Bad calibration  (0.70 m edge gap → 2 separate clusters)")
    print("=" * 62)
    rng = np.random.default_rng(42)

    OZ = -1.6
    N1X, N1Y = 4.50, -2.5
    N3X, N3Y = 5.65, -2.5   # edge gap = 5.65 - 0.25 - (4.50 + 0.20) = 0.70 m

    n1_pts = make_points(N1X, N1Y, OZ, 40, xy_spread=0.20, z_spread=0.40, rng=rng)
    n3_pts = make_points(N3X, N3Y, OZ, 30, xy_spread=0.25, z_spread=0.40, rng=rng)
    dc_pts = make_points(6.0, -3.5, OZ, 30, xy_spread=0.20, z_spread=0.40, rng=rng)

    all_pts = np.vstack([n1_pts, n3_pts, dc_pts])
    dets, vert_rej = run_detect(all_pts)

    print(f"  Input:  {len(all_pts)} pts  (n1 ctr x={N1X}, n3 ctr x={N3X})")
    print(f"  Edge gap = {N3X - 0.25 - (N1X + 0.20):.2f} m  "
          f"(cluster_tol = {CLUSTER_TOL} m)")
    print(f"  Output: {len(dets)} detection(s)")
    if vert_rej:
        print(f"  Vert-rejected z-extents (m): {[round(v, 3) for v in vert_rej]}")
    for i, d in enumerate(dets):
        c, s = d["center"], d["size"]
        print(f"    #{i+1}: center=({c[0]:.2f}, {c[1]:.2f})  "
              f"size=({s[0]:.2f} x {s[1]:.2f} x {s[2]:.2f})  n_pts={d['n_points']}")

    overlap_region_dets = [
        d for d in dets
        if 4.0 <= d["center"][0] <= 6.2 and -3.1 <= d["center"][1] <= -2.0
    ]
    p_split = len(overlap_region_dets) == 2

    print()
    print(f"  [{'PASS' if p_split else 'FAIL'}] Misaligned overlap → 2 separate clusters "
          f"in overlap region: got {len(overlap_region_dets)}")
    print(f"\n  TEST 2 RESULT: {'PASS' if p_split else 'FAIL'}")
    return p_split


# ── main ──────────────────────────────────────────────────────────────────────

if __name__ == "__main__":
    ok1 = test_good_calibration()
    ok2 = test_bad_calibration()

    print()
    print("=" * 62)
    final = "ALL TESTS PASS" if (ok1 and ok2) else "SOME TESTS FAILED"
    print(f"FINAL: {final}")
    print("=" * 62)
    sys.exit(0 if (ok1 and ok2) else 1)
