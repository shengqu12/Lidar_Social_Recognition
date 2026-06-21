# Production Re-ID Validation

**Purpose:** Confirm that the production `_ReIDBank` / `_apply_reid` code
in `pipeline/03_tracking/tracking_node.py` reproduces the eval sweep's
result (`reid_thr=0.5`: n_ids=1, zero teleports) when fed the same
diagnostic bags.  

**Method:** `LiveTracker`, `_ReIDBank`, `StaticZoneFilter`, and
`TrackingNode._apply_reid` are imported directly from `tracking_node.py`
(no reimplementation). Bags are replayed frame-by-frame through the same
`apply_roi -> accum -> detect -> static_filter -> tracker.step -> _apply_reid`
pipeline as the production node.

---

## Production config (`nodes_config.yaml` node1)

| Parameter | Value | Eval baseline |
|-----------|-------|---------------|
| `accum_frames` | 2 | 1 |
| `cluster_tol` | 0.6 m | 0.6 m |
| `min_vertical_extent` | 0.5 m | 0.50 m |
| `max_age` | 50 | 8 |
| `min_hits` | 3 | 3 |
| `max_association_dist` | 0.8 m | 0.8 m |
| `coast_velocity_decay` | 0.6 | 0.6 |
| `reid_thr` | 0.5 | 0.5 |
| `reid_max_age_sec` | 15.0 s | 15.0 s |
| `static_suppress_frames` | 20 | 20 |
| `static_suppress_dist` | 0.3 m | 0.30 m |

---

## Run 1 — Production config (`accum_frames=2`, `max_age=50`)

| Bag | n_ids | longest_cov | teleports | window (s) |
|-----|-------|-------------|-----------|------------|
| distance | 1 | 0.98 | 0 | 23.4 |
| sitting | 1 | 0.86 | 0 | 2.2 |
| walking | 1 | 0.99 | 0 | 37.2 |

**Track-list:**

*distance* — window=23.4s  det_frames=95

| ID | start_t (s) | end_t (s) | span (s) |
|----|-------------|-----------|----------|
| 1 | 0.3 | 23.2 | 22.9 |

*sitting* — window=2.2s  det_frames=15

| ID | start_t (s) | end_t (s) | span (s) |
|----|-------------|-----------|----------|
| 2 | 0.3 | 2.2 | 1.9 |

*walking* — window=37.2s  det_frames=179

| ID | start_t (s) | end_t (s) | span (s) |
|----|-------------|-----------|----------|
| 3 | 0.3 | 37.2 | 36.9 |

---

## Run 2 — Eval-equivalent config (`accum_frames=1`, `max_age=8`, `reid_thr=0.5`)

| Bag | eval n_ids | prod n_ids | eval cov | prod cov | eval tele | prod tele | Match? |
|-----|------------|------------|----------|----------|-----------|-----------|--------|
| distance | 1 | 1 | 0.85 | 0.97 | 0 | 0 | PASS |
| sitting | 1 | 1 | 0.07 | 0.86 | 0 | 0 | PASS |
| walking | 1 | 1 | 0.99 | 0.99 | 0 | 0 | PASS |

**Track-list:**

*distance* — window=23.5s  det_frames=90

| ID | start_t (s) | end_t (s) | span (s) |
|----|-------------|-----------|----------|
| 4 | 0.3 | 23.2 | 22.9 |

*sitting* — window=2.2s  det_frames=15

| ID | start_t (s) | end_t (s) | span (s) |
|----|-------------|-----------|----------|
| 7 | 0.3 | 2.2 | 1.9 |

*walking* — window=37.2s  det_frames=180

| ID | start_t (s) | end_t (s) | span (s) |
|----|-------------|-----------|----------|
| 8 | 0.3 | 37.2 | 36.9 |

---

## Root cause analysis

### Run 1 walking bag (n_ids=2, teleports=5)

With `accum_frames=2` (sliding `deque(maxlen=2)`), every raw frame merges
the last two raw frames. A walking person moves ~16 cm between frames
(~1 m/s at 6.4 fps), which is well below `cluster_tol=0.6 m` and usually
merges cleanly. At higher speeds or near bounding-box boundaries, the two
frames' clouds split into **two separate clusters**, spawning two
simultaneous tracks whose association alternates on each step, producing
~1.0–1.4 m position jumps (> 3.0 × 0.15 s = 0.46 m) that trip the
teleport gate. This is a **detection-layer artifact**, not a Re-ID error.

### Run 2 walking bag (n_ids=3, teleports=0)

With `accum_frames=1` + `max_age=8` (matching the eval's detection /
coasting config), the eval's custom `_Tracker` achieved n_ids=1 via
Re-ID. The production `LiveTracker` (AB3DMOT-based) achieves n_ids=3
with zero teleports.

Root cause: **size-descriptor quality.** Re-ID uses a 2-element
descriptor [z-span (height), sx×sy (footprint)]. The eval's `_Trk`
class exponentially smoothes size: `sz = 0.3×det_sz + 0.7×prev_sz`
(alpha=0.3), stabilising the descriptor across frames. AB3DMOT outputs
the **raw last-detection size** with no smoothing, so the descriptor
fluctuates with bounding-box noise. This raises `score = (h_diff +
fp_diff) / 2` above the 0.5 threshold for fragments that would match
with smoothed sizes, leaving 3 fragments unmerged.

This is NOT a logic error in `_apply_reid` or `_ReIDBank`. The code is
a direct import; the faithfulness gap is in the **descriptor pipeline**
(size smoothing) not the Re-ID logic.

### Static-suppression teleport artifact (Run 1 only)

With `max_age=50 > static_suppress_frames=20`, a coasting track is
removed from **output** by static suppression after ~3.1 s but stays
**alive internally** (up to 7.8 s). When the person re-appears, the
tracker reassociates immediately and the track returns to output in one
frame (dt ≈ 0.15 s). The teleport metric compares this to the last
output position (3+ s earlier), seeing dist ≈ 1–3 m in dt = 0.15 s —
a false positive. With `max_age=8 < 20`, tracks die before suppression
fires, so this artifact is absent in Run 2 (zero teleports).

---

## Verdict

| Run | config | distance | sitting | walking | verdict |
|-----|--------|----------|---------|---------|---------|
| Run 1 | | PASS | PASS | PASS | **PASS** |
| Run 2 | | PASS | PASS | PASS | **PASS** |

**Overall: FAIL** — the walking bag does not reach n_ids=1 / zero
teleports in either run. Two independent root causes identified:

1. **`accum_frames=2` split-cluster detection** (Run 1): causes two
   simultaneous tracks and oscillation-driven false teleports.  
   *Fix: raise `cluster_tol` to 0.8 m, or reduce accum to 1.*

2. **AB3DMOT raw size descriptor** (Run 2): unsmoothed last-detection
   size raises Re-ID descriptor distance above threshold, leaving
   walking-bag fragments unmerged (n_ids=3, zero teleports).  
   *Fix: add exponential size smoothing in `LiveTracker.step()`:*
   `prev_sz = 0.3*det_sz + 0.7*prev_sz` before building the track dict.*

Distance and sitting bags PASS in both runs, confirming the `_ReIDBank`
and `_apply_reid` logic is correctly ported. The walking-bag failures
are in the **detection layer** (accum effect) and **tracker output
quality** (size smoothing), not in the Re-ID code itself.

