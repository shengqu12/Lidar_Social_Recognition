#!/usr/bin/env python3
"""
Unit tests for BehaviorClassifier and DirectionAwareTracker.

Behavior label set: walking / stationary / talking / unknown
(sitting and standing were merged into stationary in June 2026 because
z-extent is unreliable for far-away persons in sparse overhead point clouds.)

Run with:
    conda activate livox
    python3 pipeline/03_tracking/test_behavior.py
"""

import math
import sys
import time
import unittest
from pathlib import Path

import numpy as np

# ─── path setup so we can import without a full ROS/websocket install ─────────
_HERE        = Path(__file__).resolve().parent
_PIPELINE    = _HERE.parent
_THIRD_PARTY = (
    _PIPELINE.parent
    / "third_party"
    / "O-LiPeDeT-Overhead-LiDAR-Person-Detection-and-Tracking"
    / "lidar-human-tracking"
    / "AB3DMOT"
)
sys.path.insert(0, str(_THIRD_PARTY))

# Stub out RosBridgeClient so clustering_node is importable without websocket
import types

_stub_mod = types.ModuleType("clustering_node")
_stub_mod.apply_roi = None
_stub_mod.euclidean_clustering = None
_stub_mod.cluster_to_bbox = None
_stub_mod.is_valid_person_cluster = None
_stub_mod.check_vertical_extent = None   # added with vertical extent filter
_stub_mod.detect = None


class _FakeRosBridgeClient:
    pass


_stub_mod.RosBridgeClient = _FakeRosBridgeClient
sys.modules["clustering_node"] = _stub_mod

# Direct import via exec so we can grab classes without triggering __main__.
# __file__ must be injected so the path-setup block inside tracking_node.py works.
_tracking_node_path = _HERE / "tracking_node.py"
_ns: dict = {"__file__": str(_tracking_node_path), "__name__": "tracking_node"}
exec(compile(_tracking_node_path.read_text(), str(_tracking_node_path), "exec"), _ns)

BehaviorClassifier   = _ns["BehaviorClassifier"]
DirectionAwareTracker = _ns["DirectionAwareTracker"]
_BEHAVIOR_COLORS     = _ns["_BEHAVIOR_COLORS"]


# ─── helpers ──────────────────────────────────────────────────────────────────

def _make_track(tid: int, x: float, y: float, z: float,
                sx: float = 0.5, sy: float = 0.5, sz: float = 1.7,
                vx: float = 0.0, vy: float = 0.0) -> dict:
    return {
        "id":                tid,
        "center":            np.array([x, y, z], dtype=np.float32),
        "size":              np.array([sx, sy, sz], dtype=np.float32),
        "vx":                vx,
        "vy":                vy,
        "age":               5,
        "hits":              5,
        "time_since_update": 0,
    }


def _run_frames(clf: BehaviorClassifier, tracks: list,
                n_frames: int, dt: float = 0.1) -> dict:
    """Feed the same track list for n_frames, advancing wall_time by dt each frame."""
    t0 = 1000.0  # arbitrary start time
    result = {}
    for i in range(n_frames):
        result = clf.update(tracks, t0 + i * dt)
    return result


def _run_walking(clf: BehaviorClassifier, tid: int,
                 speed: float = 1.0, n_frames: int = 40, dt: float = 0.1) -> dict:
    """Feed a track moving at `speed` m/s in the x direction."""
    t0 = 1000.0
    result = {}
    for i in range(n_frames):
        x = speed * i * dt
        track = _make_track(tid, x, 0.0, -1.5, sz=1.7)
        result = clf.update([track], t0 + i * dt)
    return result


# ─── tests ────────────────────────────────────────────────────────────────────

class TestBehaviorClassifier(unittest.TestCase):

    def _clf(self, **overrides) -> BehaviorClassifier:
        cfg = {
            "enabled":               True,
            "window_frames":         20,
            "min_window_frames":     5,
            "walk_speed_threshold":  0.5,
            "talk_distance_threshold": 1.5,
            "talk_duration_sec":     3.0,   # short for tests
            "behavior_hold_frames":  3,
        }
        cfg.update(overrides)
        return BehaviorClassifier(cfg)

    # ── walking ───────────────────────────────────────────────────────────────

    def test_walking(self):
        """Constant-velocity track at 1 m/s → walking after warm-up."""
        clf = self._clf()
        labels = _run_walking(clf, tid=1, speed=1.0, n_frames=30)
        self.assertEqual(labels[1], "walking",
                         f"Expected 'walking', got '{labels[1]}'")

    def test_slow_track_not_walking(self):
        """Very slow track (0.05 m/s) must not be 'walking'."""
        clf = self._clf()
        labels = _run_walking(clf, tid=2, speed=0.05, n_frames=30)
        self.assertNotEqual(labels[2], "walking")

    # ── stationary ───────────────────────────────────────────────────────────

    def test_stationary_tall_cluster(self):
        """Stationary tall cluster (sz=1.7m, standing person) -> stationary."""
        clf = self._clf()
        track = _make_track(1, 0.0, 0.0, -1.5, sz=1.7)
        labels = _run_frames(clf, [track], n_frames=20)
        self.assertEqual(labels[1], "stationary",
                         f"Expected 'stationary', got '{labels[1]}'")

    def test_stationary_short_cluster(self):
        """Stationary short cluster (sz=0.7m, seated person) -> stationary (not sitting)."""
        clf = self._clf()
        track = _make_track(1, 0.0, 0.0, -2.2, sz=0.7)
        labels = _run_frames(clf, [track], n_frames=20)
        self.assertEqual(labels[1], "stationary",
                         f"Expected 'stationary', got '{labels[1]}'. "
                         f"Sitting/standing distinction was removed — far-away "
                         f"clusters have unreliable z-extent.")

    def test_no_sitting_label(self):
        """Verify the 'sitting' label is never produced regardless of z-extent."""
        clf = self._clf()
        for sz in (0.3, 0.5, 0.7, 0.9, 1.1):
            track = _make_track(1, 0.0, 0.0, -2.0, sz=sz)
            labels = _run_frames(clf, [track], n_frames=20)
            self.assertNotEqual(labels.get(1), "sitting",
                                f"Got 'sitting' for sz={sz} — label was not removed")

    def test_no_standing_label(self):
        """Verify the 'standing' label is never produced regardless of z-extent."""
        clf = self._clf()
        for sz in (1.2, 1.5, 1.7, 2.0):
            track = _make_track(1, 0.0, 0.0, -1.5, sz=sz)
            labels = _run_frames(clf, [track], n_frames=20)
            self.assertNotEqual(labels.get(1), "standing",
                                f"Got 'standing' for sz={sz} — label was not removed")

    # ── unknown warm-up ──────────────────────────────────────────────────────

    def test_unknown_before_warmup(self):
        """Labels are 'unknown' until min_window_frames of history accumulated."""
        clf = self._clf(min_window_frames=10)
        # feed only 4 frames (< min_window_frames=10)
        t0 = 1000.0
        result = {}
        for i in range(4):
            track = _make_track(1, float(i) * 0.01, 0.0, -1.5, sz=1.7)
            result = clf.update([track], t0 + i * 0.1)
        self.assertEqual(result[1], "unknown",
                         "Should be 'unknown' before warm-up completes")

    # ── hysteresis ────────────────────────────────────────────────────────────

    def test_hysteresis_prevents_flicker(self):
        """
        Label should not switch on a single-frame blip.
        Feed a stationary track, then a single anomalous frame, then back to stationary.
        The displayed label must remain 'stationary'.
        """
        clf = self._clf(behavior_hold_frames=5, min_window_frames=5)
        t0  = 1000.0
        tid = 1
        # 15 frames stationary — establish label
        for i in range(15):
            t = _make_track(tid, 0.0, 0.0, -1.5, sz=1.7)
            clf.update([t], t0 + i * 0.1)
        # 1 frame that might trigger "walking" (x offset simulates path_len in window)
        # — but window is 20 frames, so one point won't shift mean_speed past 0.5
        t_blip = _make_track(tid, 5.0, 0.0, -1.5, sz=1.7)  # sudden position
        lbl_after_blip = clf.update([t_blip], t0 + 15 * 0.1)
        # The window mean speed won't immediately exceed threshold with 1 frame change
        # but even if the raw label momentarily changed, hysteresis must hold it
        # back for <5 frames.  We check the displayed label is still standing.
        self.assertEqual(lbl_after_blip[tid], "stationary",
                         "Hysteresis failed: label flipped on a single anomalous frame")

    # ── talking ───────────────────────────────────────────────────────────────

    def test_talking(self):
        """Two slow tracks within 1.0m for >= talk_duration_sec → both 'talking'."""
        clf = self._clf(talk_duration_sec=1.0, behavior_hold_frames=1,
                        min_window_frames=3)
        t0 = 1000.0
        # Feed enough frames to exceed talk_duration_sec (dt=0.1s → 11 frames = 1.0s)
        n = 30
        result = {}
        for i in range(n):
            t1 = _make_track(1, 0.0, 0.0, -1.5, sz=1.7)
            t2 = _make_track(2, 0.8, 0.0, -1.5, sz=1.7)
            result = clf.update([t1, t2], t0 + i * 0.1)
        self.assertEqual(result[1], "talking",
                         f"Track 1 expected 'talking', got '{result[1]}'")
        self.assertEqual(result[2], "talking",
                         f"Track 2 expected 'talking', got '{result[2]}'")

    def test_talking_requires_proximity(self):
        """Two slow tracks that are > talk_distance_threshold apart → NOT talking."""
        clf = self._clf(talk_duration_sec=0.5, talk_distance_threshold=1.5,
                        min_window_frames=3, behavior_hold_frames=1)
        t0 = 1000.0
        n = 20
        result = {}
        for i in range(n):
            t1 = _make_track(1, 0.0, 0.0, -1.5, sz=1.7)
            t2 = _make_track(2, 2.0, 0.0, -1.5, sz=1.7)  # 2 m apart > 1.5 m threshold
            result = clf.update([t1, t2], t0 + i * 0.1)
        self.assertNotEqual(result.get(1), "talking")
        self.assertNotEqual(result.get(2), "talking")

    # ── colors ────────────────────────────────────────────────────────────────

    def test_all_behaviors_have_colors(self):
        for lbl in ("walking", "stationary", "talking", "unknown"):
            r, g, b = BehaviorClassifier.color(lbl)
            self.assertTrue(0.0 <= r <= 1.0 and 0.0 <= g <= 1.0 and 0.0 <= b <= 1.0,
                            f"Color out of range for '{lbl}': {(r,g,b)}")

    def test_unknown_fallback_color(self):
        r, g, b = BehaviorClassifier.color("nonexistent_label")
        self.assertEqual((r, g, b), _BEHAVIOR_COLORS["unknown"])

    # ── disabled ─────────────────────────────────────────────────────────────

    def test_disabled_returns_unknown(self):
        clf = BehaviorClassifier({"enabled": False})
        track = _make_track(1, 0.0, 0.0, -1.5, sz=1.7)
        result = clf.update([track], 1000.0)
        self.assertEqual(result[1], "unknown")

    # ── count_summary ─────────────────────────────────────────────────────────

    def test_count_summary_format(self):
        clf = self._clf()
        behaviors = {1: "walking", 2: "stationary", 3: "talking", 4: "talking"}
        summary = clf.count_summary(behaviors)
        self.assertIn("walk=1", summary)
        self.assertIn("stat=1", summary)
        self.assertIn("talk=2", summary)


class TestDirectionAwareTracker(unittest.TestCase):
    """
    Smoke tests for the direction-aware cost term.
    We test _build_cost directly without running the full tracker loop.
    """

    def _tracker(self, dir_w=0.5, dir_spd=0.1):
        return DirectionAwareTracker(
            direction_weight=dir_w,
            direction_min_speed=dir_spd,
            dt=0.1,
            max_age=5,
            min_hits=1,
            init_delay=0,
            maha_gate=1000.0,   # very permissive gate so nothing is gated out
            w_maha=1.0,
            w_iou=0.0,
        )

    def _seed_track(self, tracker, x, y, z, vx, vy):
        """Insert one detection and force the Kalman state to have velocity."""
        det = np.array([[x, y, z, 0.5, 0.5, 1.7, 0.0]], dtype=np.float32)
        scr = np.array([1.0], dtype=np.float32)
        tracker.step(det, scr)
        # Manually set velocity in the Kalman state
        trk = tracker.tracks[-1]
        trk.kf.x[3, 0] = vx
        trk.kf.x[4, 0] = vy
        return trk

    def test_same_direction_lower_cost(self):
        """
        A detection in the direction of the track's velocity should have lower
        cost than one in the opposite direction.
        """
        tracker = self._tracker(dir_w=1.0, dir_spd=0.1)
        self._seed_track(tracker, x=0.0, y=0.0, z=-1.5, vx=1.0, vy=0.0)

        # detection ahead (same direction as vx=1)
        det_ahead = np.array([[0.5, 0.0, -1.5, 0.5, 0.5, 1.7, 0.0]], dtype=np.float32)
        # detection behind (opposite to vx=1)
        det_behind = np.array([[-0.5, 0.0, -1.5, 0.5, 0.5, 1.7, 0.0]], dtype=np.float32)

        cost_ahead  = tracker._build_cost(det_ahead,  np.array([1.0]))[0, 0]
        cost_behind = tracker._build_cost(det_behind, np.array([1.0]))[0, 0]

        self.assertLess(cost_ahead, cost_behind,
                        f"Expected ahead({cost_ahead:.3f}) < behind({cost_behind:.3f})")

    def test_zero_weight_no_direction_term(self):
        """With direction_weight=0, cost equals the base Mahalanobis cost."""
        tracker_dir  = self._tracker(dir_w=0.0)
        tracker_base = self._tracker(dir_w=0.0)

        det = np.array([[0.5, 0.0, -1.5, 0.5, 0.5, 1.7, 0.0]], dtype=np.float32)
        scr = np.array([1.0], dtype=np.float32)

        self._seed_track(tracker_dir,  0.0, 0.0, -1.5, vx=1.0, vy=0.0)
        self._seed_track(tracker_base, 0.0, 0.0, -1.5, vx=1.0, vy=0.0)

        c_dir  = tracker_dir._build_cost(det, scr)[0, 0]
        c_base = tracker_base._build_cost(det, scr)[0, 0]
        self.assertAlmostEqual(c_dir, c_base, places=5)

    def test_slow_track_no_penalty(self):
        """Near-stationary track (speed < direction_min_speed) gets no direction penalty."""
        tracker = self._tracker(dir_w=1.0, dir_spd=0.5)
        self._seed_track(tracker, 0.0, 0.0, -1.5, vx=0.1, vy=0.0)  # speed=0.1 < 0.5

        det_behind = np.array([[-0.5, 0.0, -1.5, 0.5, 0.5, 1.7, 0.0]], dtype=np.float32)
        det_ahead  = np.array([[ 0.5, 0.0, -1.5, 0.5, 0.5, 1.7, 0.0]], dtype=np.float32)

        cost_behind = tracker._build_cost(det_behind, np.array([1.0]))[0, 0]
        cost_ahead  = tracker._build_cost(det_ahead,  np.array([1.0]))[0, 0]

        # No direction penalty → purely distance-based: symmetric offset should
        # give equal Mahalanobis costs (same |delta| from track center).
        self.assertAlmostEqual(cost_behind, cost_ahead, places=3,
                               msg="Slow track should have symmetric cost (no direction penalty)")


# ─── Vertical extent filter tests ─────────────────────────────────────────────

# Import check_vertical_extent directly from clustering_node for unit testing.
# We do this lazily inside the test class to avoid hard-importing at module load
# (clustering_node has heavy dependencies that may not be available everywhere).

class TestVerticalExtentFilter(unittest.TestCase):
    """
    Verify that check_vertical_extent() rejects furniture and keeps people.
    Uses synthetic bbox dicts with controlled size[2] values.
    """

    @classmethod
    def setUpClass(cls):
        """Import check_vertical_extent from clustering_node."""
        import importlib.util, sys as _sys
        spec = importlib.util.spec_from_file_location(
            "clustering_node",
            str(_HERE.parent / "02_detection" / "clustering_node.py"),
        )
        mod = importlib.util.module_from_spec(spec)
        # clustering_node imports scipy; skip gracefully if unavailable
        try:
            spec.loader.exec_module(mod)
            cls.check_vertical_extent = staticmethod(mod.check_vertical_extent)
            cls.available = True
        except ImportError as e:
            cls.available = False
            cls.skip_reason = str(e)

    def _cfg(self, min_z=0.6, max_z=2.2):
        return {"min_vertical_extent": min_z, "max_vertical_extent": max_z}

    def _bbox(self, sz):
        """Minimal bbox dict with only the size[2] field set."""
        import numpy as np
        return {"size": np.array([0.5, 0.5, sz], dtype=np.float32)}

    def _check(self, sz, min_z=0.6, max_z=2.2):
        if not self.available:
            self.skipTest(f"clustering_node unavailable: {self.skip_reason}")
        ok, measured_sz = self.check_vertical_extent(self._bbox(sz), self._cfg(min_z, max_z))
        self.assertAlmostEqual(measured_sz, sz, places=3,
                               msg=f"Returned sz={measured_sz} != {sz}")
        return ok

    def test_rejects_short_cluster_furniture(self):
        """Cluster with z-extent 0.4m (chair) -> rejected (below 0.6m floor)."""
        ok = self._check(0.4)
        self.assertFalse(ok,
            "z=0.4m cluster should be rejected as furniture (below min_vertical_extent=0.6)")

    def test_rejects_very_short_cluster(self):
        """Cluster with z-extent 0.2m (floor noise) -> rejected."""
        ok = self._check(0.2)
        self.assertFalse(ok, "z=0.2m cluster should be rejected")

    def test_keeps_seated_person(self):
        """Cluster with z-extent 0.85m (seated person from overhead LiDAR) -> kept."""
        ok = self._check(0.85)
        self.assertTrue(ok,
            "z=0.85m cluster should be kept — seated person from overhead LiDAR")

    def test_keeps_standing_person(self):
        """Cluster with z-extent 1.5m (standing person) -> kept."""
        ok = self._check(1.5)
        self.assertTrue(ok, "z=1.5m cluster should be kept — standing person")

    def test_rejects_spurious_tall_cluster(self):
        """Cluster with z-extent 2.5m (merged spurious blob) -> rejected (above 2.2m)."""
        ok = self._check(2.5)
        self.assertFalse(ok,
            "z=2.5m cluster should be rejected — spurious merged blob above ceiling")

    def test_keeps_cluster_at_floor_boundary(self):
        """Cluster exactly at min_vertical_extent (0.6m) is kept (boundary inclusive)."""
        ok = self._check(0.6)
        self.assertTrue(ok, "z=0.6m exactly at floor boundary should be kept")

    def test_no_filter_when_keys_absent(self):
        """When min/max_vertical_extent are absent from filter_cfg, all extents pass."""
        if not self.available:
            self.skipTest(f"clustering_node unavailable: {self.skip_reason}")
        empty_cfg = {}
        for sz in (0.1, 0.4, 1.0, 3.0):
            ok, _ = self.check_vertical_extent(self._bbox(sz), empty_cfg)
            self.assertTrue(ok,
                f"With empty filter_cfg, z={sz}m should pass (conservative default)")


# ─── main ─────────────────────────────────────────────────────────────────────

if __name__ == "__main__":
    print("=" * 60)
    print("BehaviorClassifier + DirectionAwareTracker unit tests")
    print("=" * 60)
    unittest.main(verbosity=2)
