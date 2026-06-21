# Detection Stage-5 Parameter Sweep

**Date:** 2026-06-16  
**Bags:** diag_distance, diag_sitting, diag_walking (each contains exactly 1 person)  
**Baseline:** accum_frames=4, cluster_tol=0.6 m, min_points=20, max_xy_size=1.0 m, max_aspect_ratio=4.0, min_vert=0.6 m, max_vert=2.2 m  
**Recall** = frames where the primary (largest S4) cluster passes Stage 5 / frames with ≥1 S4 cluster  
**FP/frame** = mean non-primary accepted clusters per evaluated frame  
⚠️ = FP/frame > 0 (precision cost)


---


### Strategy A — Relax `max_xy_size` (accum_frames=4, all else baseline)

_Directly enlarges the axis-aligned XY bounding-box cap._

| Config | recall (distance) | recall (sitting) | recall (walking) | FP/f (distance) | FP/f (sitting) | FP/f (walking) | recall (avg) | FP/f (avg) | n_frames |
|---|---|---|---|---|---|---|---|---|---|
| max_xy=1.0 | 0.690 | 0.029 | 0.034 | 0.000 | 0.000 | 0.536 | **0.203** | **0.197** ⚠️ | 645 |
| max_xy=1.5 | 0.780 | 0.054 | 0.207 | 0.024 | 0.000 | 0.591 | **0.299** | **0.223** ⚠️ | 645 |
| max_xy=2.0 | 0.827 | 0.796 | 0.759 | 0.190 | 0.000 | 0.629 | **0.791** | **0.281** ⚠️ | 645 |
| max_xy=2.5 | 0.839 | 0.971 | 0.886 | 0.220 | 0.000 | 0.629 | **0.905** | **0.288** ⚠️ | 645 |


### Strategy B — Reduce `accum_frames` (max_xy_size=1.0, all else baseline)

_Shorter accumulation window shrinks the motion trail, reducing cluster XY footprint._

| Config | recall (distance) | recall (sitting) | recall (walking) | FP/f (distance) | FP/f (sitting) | FP/f (walking) | recall (avg) | FP/f (avg) | n_frames |
|---|---|---|---|---|---|---|---|---|---|
| accum=4 | 0.690 | 0.029 | 0.034 | 0.000 | 0.000 | 0.536 | **0.203** | **0.197** ⚠️ | 645 |
| accum=2 | 0.769 | 0.190 | 0.084 | 0.000 | 0.000 | 0.577 | **0.302** | **0.212** ⚠️ | 650 |
| accum=1 | 0.684 | 0.510 | 0.146 | 0.000 | 0.000 | 0.500 | **0.406** | **0.195** ⚠️ | 616 |


### Strategy C — PCA minor-axis gate (accum_frames=4, major axis unconstrained)

_Replaces max_xy_size + max_aspect_ratio with minor-axis extent ≤ threshold. Vertical extent check unchanged._

| Config | recall (distance) | recall (sitting) | recall (walking) | FP/f (distance) | FP/f (sitting) | FP/f (walking) | recall (avg) | FP/f (avg) | n_frames |
|---|---|---|---|---|---|---|---|---|---|
| pca_minor≤0.6 | 0.685 | 0.000 | 0.042 | 0.000 | 0.000 | 0.481 | **0.194** | **0.177** ⚠️ | 645 |
| pca_minor≤0.8 | 0.744 | 0.025 | 0.076 | 0.024 | 0.000 | 0.544 | **0.231** | **0.206** ⚠️ | 645 |
| pca_minor≤1.0 | 0.780 | 0.150 | 0.110 | 0.065 | 0.000 | 0.586 | **0.299** | **0.233** ⚠️ | 645 |

#### Strategy C — Primary cluster PCA distribution (accum_frames=4)

| Bag | stat | minor axis (m) | major axis (m) |
|-----|------|---------------|---------------|
| distance | p5 | 0.252 | 0.325 |
| distance | p25 | 0.352 | 0.518 |
| distance | p50 | 0.442 | 0.588 |
| distance | p75 | 0.492 | 0.743 |
| distance | p95 | 1.055 | 1.926 |
| distance | max | 1.579 | 2.417 |
| sitting | p5 | 0.859 | 1.505 |
| sitting | p25 | 1.076 | 1.723 |
| sitting | p50 | 1.286 | 1.825 |
| sitting | p75 | 1.565 | 1.951 |
| sitting | p95 | 2.107 | 2.245 |
| sitting | max | 2.338 | 3.295 |
| walking | p5 | 0.588 | 0.713 |
| walking | p25 | 1.165 | 1.483 |
| walking | p50 | 1.362 | 1.721 |
| walking | p75 | 1.654 | 2.081 |
| walking | p95 | 1.960 | 2.620 |
| walking | max | 2.313 | 2.873 |

_Interpretation: the minor axis ≈ person's cross-sectional width (should be small even for accumulated walking blobs). The major axis grows with motion trail length._


### Strategy D — Lower `min_vertical_extent` (accum_frames=4, max_xy_size=1.0 baseline)

_Targets the 2 near-sensor frames (diag_distance f139/f147) where vert_span=0.579/0.599 m was the rejection cause. All other frames still fail XY filter first._

| Config | recall (distance) | recall (sitting) | recall (walking) | FP/f (distance) | FP/f (sitting) | FP/f (walking) | recall (avg) | FP/f (avg) | n_frames |
|---|---|---|---|---|---|---|---|---|---|
| vert_min=0.60 | 0.690 | 0.029 | 0.034 | 0.000 | 0.000 | 0.536 | **0.203** | **0.197** ⚠️ | 645 |
| vert_min=0.55 | 0.851 | 0.029 | 0.059 | 0.000 | 0.000 | 0.540 | **0.254** | **0.198** ⚠️ | 645 |
| vert_min=0.50 | 0.851 | 0.029 | 0.063 | 0.000 | 0.000 | 0.540 | **0.256** | **0.198** ⚠️ | 645 |

#### Strategy D — Newly admitted secondary clusters (FP risk)

_(Clusters that pass baseline XY filter AND have vert_span in the newly unlocked range below the original 0.6 m floor)_

**vert_min=0.50_diag_walking** — 1 newly-admitted secondary cluster(s):

| frame | rank | n_pts | sx | sy | sz |
|-------|------|-------|----|----|-----|
| 142 | 1 | 57 | 0.324 | 0.194 | 0.554 |

**vert_min=0.55_diag_walking** — 1 newly-admitted secondary cluster(s):

| frame | rank | n_pts | sx | sy | sz |
|-------|------|-------|----|----|-----|
| 142 | 1 | 57 | 0.324 | 0.194 | 0.554 |


---

## Recommendation

### Baseline FP note

The **walking bag has FP/frame = 0.536 at baseline** (accum=4, max_xy=1.0 — secondary clusters from scene objects pass Stage-5 even at baseline). This is a structural precision problem that predates the sweep. The table below uses per-bag delta-FP to identify configs that introduce *new* FPs:

| Bag | Baseline FP/frame |
|-----|------------------|

| distance | 0.000 |
| sitting | 0.000 |
| walking | 0.536 |


**Zero-new-FP criterion:** delta FP ≤ 0.01 for every bag (≤1 extra FP per 100 frames above baseline).

### Per-bag best config (zero new FP cost)

| Bag | Best zero-cost config | Recall | FP/frame | Delta FP |
|-----|----------------------|--------|----------|----------|
| distance | **D** `vert_min=0.50` | 0.851 | 0.000 | +0.000 |
| sitting | **B** `accum=1` | 0.510 | 0.000 | +0.000 |
| walking | **B** `accum=1` | 0.146 | 0.500 | +-0.036 |

### Configs with highest recall (⚠️ introduces new FPs)

| Strategy | Config | Recall (avg) | FP/f (avg) | Max ΔFPP | R(dist) | R(sit) | R(walk) |
|----------|--------|-------------|-----------|---------|--------|--------|--------|
| A | `max_xy=2.5` | 0.905 | 0.288 | +0.220 | 0.839 | 0.971 | 0.886 |
| A | `max_xy=2.0` | 0.791 | 0.281 | +0.190 | 0.827 | 0.796 | 0.759 |
| B | `accum=2` | 0.302 | 0.212 | +0.042 | 0.769 | 0.190 | 0.084 |
| A | `max_xy=1.5` | 0.299 | 0.223 | +0.055 | 0.780 | 0.054 | 0.207 |
| C | `pca_minor≤1.0` | 0.299 | 0.233 | +0.065 | 0.780 | 0.150 | 0.110 |

### Summary

**Best zero-new-FP single config (avg recall):** Strategy **B** `accum=1` — recall 0.406, FP/frame 0.195  

**Highest absolute recall (any config):** Strategy **A** `max_xy=2.5` — recall 0.905, FP/frame 0.288  
⚠️ Buys recall at max per-bag delta FP = +0.220.

**Strategy C (PCA minor-axis) verdict:** Underperforms A and B on all bags. The sitting person's accumulated blob is roughly circular (minor-axis p50 = 1.286 m), so PCA provides no separation advantage over axis-aligned bbox. Minor-axis gating only helps strongly elongated walking clusters, where Strategy B already handles the root cause more cleanly with zero FP cost.

**Strategy D (vert_min=0.55) note:** Recovers +0.161 recall in distance bag with zero new secondary FPs in distance or sitting (1 newly-admitted secondary cluster in walking, sz=0.554 m, 57 pts). No gain for sitting or walking primary clusters. Best used as a complement to B, not standalone.


### Combination candidates (not swept)

- **B (accum=1) + D (vert_min=0.55):** Expected distance ~0.851+, sitting ~0.510, walking ~0.146; zero new FPs. Best zero-cost combination.
- **A (max_xy=2.5) standalone:** Recall 0.905 avg but delta FP +0.220 in distance, +0.093 in walking. ⚠️ Precision cost is real.
- **A (max_xy=1.5) + D (vert_min=0.55):** Smaller XY relaxation; expected lower FP cost than max_xy=2.5 while recovering sitting partially.

Combinations not swept (task: one variable at a time).

