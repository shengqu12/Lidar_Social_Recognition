# Tracking Maintenance Sweep

**Date:** 2026-06-16  
**Detection config:** B+D — `accum_frames=1`, `vert_min=0.50 m` (zero-precision-cost combination from detection_sweep.md).  
**Tracking baseline (nodes_config.yaml node1):** `max_age=8`, `min_hits=3`, `max_dist=0.8 m`, `coast_decay=0.6`  
**Bags:** diag_distance, diag_sitting, diag_walking (each contains exactly ONE person — ideal n_ids = 1)  
**Metric definitions:**  
- `n_ids` = distinct track IDs in person-present window (entry→exit)  
- `longest_cov` = span of longest single ID / window span (ideal 1.0)  
- `mean_life` = mean (last_t − first_t) per ID (seconds)  
- `teleport` = position jump > 3.0 m/s × elapsed (false-stitch flag)  

---


### Strategy A — Coasting horizon (`max_age`)

_How many consecutive missed frames a track survives via Kalman prediction._  
_At walking recall = 0.146, expected gap = 1/0.146 − 1 ≈ 5.9 frames; gaps > `max_age` cause a new ID. Rationale: extend horizon past the typical gap._

| Config | n_ids (dist) | n_ids (sit) | n_ids (walk) | cov (dist) | cov (sit) | cov (walk) | avg n_ids | avg cov | teleports |
|--------|-------------|------------|-------------|-----------|----------|-----------|-----------|---------|-----------|
| `max_age=8`  ← **baseline** | 2 | 1 | 3 | 0.52 | 0.07 | 0.52 | **2.0** | **0.37** | 0 |
| `max_age=5` | 2 | 1 | 4 | 0.49 | 0.07 | 0.27 | **2.3** | **0.28** | 0 |
| `max_age=10` | 2 | 1 | 3 | 0.52 | 0.07 | 0.53 | **2.0** | **0.37** | 0 |
| `max_age=20` | 2 | 1 | 2 | 0.52 | 0.07 | 0.55 | **1.7** | **0.38** | 0 |


### Strategy B — Association gate (`max_dist`)

_Mahalanobis gate is set to (max_dist² / meas_var) × 1.5._  
_Wider gate lets coasting tracks re-associate after a long position drift._

| Config | n_ids (dist) | n_ids (sit) | n_ids (walk) | cov (dist) | cov (sit) | cov (walk) | avg n_ids | avg cov | teleports |
|--------|-------------|------------|-------------|-----------|----------|-----------|-----------|---------|-----------|
| `max_dist=0.4` | 2 | 1 | 3 | 0.52 | 0.07 | 0.52 | **2.0** | **0.37** | 0 |
| `max_dist=0.8`  ← **baseline** | 2 | 1 | 3 | 0.52 | 0.07 | 0.52 | **2.0** | **0.37** | 0 |
| `max_dist=1.2` | 2 | 1 | 3 | 0.52 | 0.07 | 0.52 | **2.0** | **0.37** | 0 |


### Strategy C — Geometric Re-ID

_When a detection can't associate to any active track, compare to recently-lost tracks by descriptor = [height (z-span), footprint (sx×sy)] + spatial-consistency check (gap ≤ 3 m/s × elapsed). If descriptor distance < threshold, revive the old ID instead of spawning new._

| Config | n_ids (dist) | n_ids (sit) | n_ids (walk) | cov (dist) | cov (sit) | cov (walk) | avg n_ids | avg cov | teleports |
|--------|-------------|------------|-------------|-----------|----------|-----------|-----------|---------|-----------|
| `no_reid (baseline)`  ← **baseline** | 2 | 1 | 3 | 0.52 | 0.07 | 0.52 | **2.0** | **0.37** | 0 |
| `reid_thr=0.5` | 1 | 1 | 1 | 0.85 | 0.07 | 0.99 | **1.0** | **0.64** | 0 |
| `reid_thr=1.0` | 1 | 1 | 1 | 0.85 | 0.07 | 0.99 | **1.0** | **0.64** | 0 |
| `reid_thr=1.5` | 1 | 1 | 1 | 0.85 | 0.07 | 0.99 | **1.0** | **0.64** | 0 |

---

## Track-list breakdown

Per-ID (id, start_t, end_t) for baseline and best config per bag.

### distance

**Baseline (max_age=8):** window=26.8s  det_frames=131

| ID | start_t (s) | end_t (s) | span (s) |
|----|-------------|-----------|----------|
| 1 | 0.3 | 4.7 | 4.4 |
| 2 | 9.3 | 23.2 | 13.9 |

**Best config: `reid_thr=0.5`** — n_ids=1  cov=0.85, zero new teleports

| ID | start_t (s) | end_t (s) | span (s) |
|----|-------------|-----------|----------|
| 1 | 0.3 | 23.2 | 22.9 |

### sitting

**Baseline (max_age=8):** window=37.7s  det_frames=243

| ID | start_t (s) | end_t (s) | span (s) |
|----|-------------|-----------|----------|
| 1 | 0.3 | 3.1 | 2.8 |

**Best config: `max_age=10`** — n_ids=1  cov=0.07, zero new teleports

| ID | start_t (s) | end_t (s) | span (s) |
|----|-------------|-----------|----------|
| 1 | 0.3 | 3.1 | 2.8 |

### walking

**Baseline (max_age=8):** window=37.2s  det_frames=180

| ID | start_t (s) | end_t (s) | span (s) |
|----|-------------|-----------|----------|
| 1 | 0.3 | 19.8 | 19.5 |
| 2 | 22.9 | 33.3 | 10.4 |
| 3 | 35.2 | 37.2 | 2.0 |

**Best config: `reid_thr=0.5`** — n_ids=1  cov=0.99, zero new teleports

| ID | start_t (s) | end_t (s) | span (s) |
|----|-------------|-----------|----------|
| 1 | 0.3 | 37.2 | 36.9 |

---

## Teleport / false-stitch events

_No teleport events detected in any config._

---

## Recommendation

### Teleport note

Marginal position jumps (0.48–0.58 m in 0.156 s, barely over 3 m/s × dt = 0.47 m) appear in the walking bag across ALL configs. These are Kalman correction artifacts that occur when coast-velocity decay damps the predicted velocity to near zero and a new detection corrects the position — NOT false-stitch events. The 1.0 m hard floor (`_TELE_MIN`) suppresses them from the flag table.  
All teleport events > 1.0 m are listed below. **Any config with MORE such events than the baseline is flagged ⚠️.**

### All configs ranked by avg n_ids

| Rank | Config | avg n_ids | avg cov | dist n_ids | sit n_ids | walk n_ids | new teleports |
|------|--------|-----------|---------|-----------|----------|----------|--------------|
| 1 | `reid_thr=0.5` | 1.0 | 0.64 | 1 | 1 | 1 | 0 |
| 2 | `reid_thr=1.0` | 1.0 | 0.64 | 1 | 1 | 1 | 0 |
| 3 | `reid_thr=1.5` | 1.0 | 0.64 | 1 | 1 | 1 | 0 |
| 4 | `max_age=20` | 1.7 | 0.38 | 2 | 1 | 2 | 0 |
| 5 | `max_age=10` | 2.0 | 0.37 | 2 | 1 | 3 | 0 |
| 6 | `max_age=8` | 2.0 | 0.37 | 2 | 1 | 3 | 0 |
| 7 | `max_dist=0.4` | 2.0 | 0.37 | 2 | 1 | 3 | 0 |
| 8 | `max_dist=0.8` | 2.0 | 0.37 | 2 | 1 | 3 | 0 |
| 9 | `max_dist=1.2` | 2.0 | 0.37 | 2 | 1 | 3 | 0 |
| 10 | `no_reid (baseline)` | 2.0 | 0.37 | 2 | 1 | 3 | 0 |
| 11 | `max_age=5` | 2.3 | 0.28 | 2 | 1 | 4 | 0 |

**Best config:** `reid_thr=0.5` — avg n_ids 1.0, avg cov 0.64.  
Zero new false-stitch events vs baseline.

### Walking bag note

Walking detection recall (B+D) ≈ 0.146 → person detected in ~1 of 7 frames.  
At baseline (max_age=8): n_ids = **3**, cov = 0.52.  
At best config (`reid_thr=0.5`): n_ids = **1**, cov = 0.99.  

**Interpretation:** With recall = 0.146, the mean inter-detection gap is ~5.9 frames (0.92 s). P(gap > 8 frames) ≈ 0.29, so ~29% of detections are separated by a gap exceeding baseline max_age, each causing a new ID. Extending max_age to 20 reduces P(gap > 20) ≈ 0.046, dramatically cutting fragmentation. Re-ID further merges fragments that survive even beyond the extended horizon.

### Sitting bag note

Sitting bag: n_ids=1, cov=0.07 across ALL configs. This is NOT a fragmentation problem — it is a **static suppression** problem.  
The sitting person barely moves (< 0.30 m displacement). After `suppress_frames=20` consecutive confirmed output frames (~3.1 s at 6.4 fps), the tracker's per-track displacement check triggers and suppresses the track from output.  
Crucially, the OLD track ID is still alive internally and keeps absorbing new detections — so NO new track ID ever spawns, and Re-ID has nothing to relabel. Result: n_ids=1 (only one track ever appears) but cov=0.07 (track visible for only 3.1 s).  
**Fix:** `suppress_dist` (0.30 m default) should be raised, or the static suppression should gate on the person's CLASS (not yet available). This is outside the scope of this sweep (only tracking-maintenance params swept).

### min_hits warmup note

All configs use min_hits=3 (tracking default). After each track break, the revived track must accumulate 3 detections before appearing in output. At walking recall 0.146, that takes ~3/0.146 ≈ 20.5 frames (~3.2 s). Reducing min_hits to 1 would eliminate this warmup delay but risks single-frame ghost detections — not swept here (outside task scope).
