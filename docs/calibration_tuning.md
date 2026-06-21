# Calibration and Tuning

The parameters in this system are **deliberate design decisions with rationale**,
not incidental defaults. This document records what each one is for, why it is
set where it is, and — for the ones with sharp edges — what goes wrong if you
move it. Values quoted here are the current `config/nodes_config.yaml` settings;
the config is the source of truth, this is the reasoning behind it.

One constraint sits above all the others and is stated first because several
parameters exist to protect it.

## The research-critical constraint: do not cluster away social distance

The whole point of the study is the spatial relationship between people — who is
near whom, and how that maps to interaction. Two people standing a metre apart is
a *signal*, not noise. Any parameter that would merge two genuinely separate
people into one detection destroys the measurement.

Concretely: **clustering tolerance must stay at `cluster_tol: 0.6` m, not 0.8 m.**
At 0.8 m, two people standing about a metre apart get merged into a single
cluster — exactly the social distance the research is trying to observe. Several
other parameters below (ghost-track merging, exclusion zones, static handling)
are tuned narrowly for the same reason: they suppress artifacts without ever
collapsing two real people together. When tuning, the test is always "could this
merge two nearby people?" — if yes, it is wrong regardless of how clean the
output looks.

## ICP calibration (node3 → node1)

The two LiDARs see overlapping parts of a roughly rectangular room and must be
expressed in one common frame. node1 is the reference frame; node3 is aligned to
it with an ICP transform stored in `calib_out/node3_to_node1.txt` and promoted
into `fusion.sources[node3].transform_to_common` in the config.

**The 180° symmetry trap.** A rectangular room is nearly symmetric under a 180°
rotation. Plain ICP, optimising for fitness, happily converges to the flipped
solution — node3 mounted on the south-east looking like it sits on the north-west
— and reports an excellent fitness score for a physically wrong answer. **Fitness
cannot be trusted to distinguish the two.**

The alignment is therefore pinned with two independent checks:

- **Physical prior as the seed.** ICP is seeded with the known approximate
  orientation (yaw ≈ 180°, node3 on the south-east vs node1 on the north-west)
  so it starts in the basin of the correct solution rather than the flipped one.
- **An asymmetric feature for validation.** A corridor feature that is *not*
  symmetric under the 180° flip is used to confirm the result. The corridor
  alignment score (0.76) tells you the solution is the right one, in a way that
  fitness alone cannot.

The accepted result is yaw ≈ -179°, corridor score 0.76, RMSE 0.15 m. If you
ever re-run calibration, re-derive the physical-prior seed and re-check the
corridor feature; do not accept a transform on fitness alone, however good it
looks.

## Background model

Background removal is statistical: each voxel stores a mean and standard
deviation from an empty-scene recording, and a live point is kept as foreground
only if it lies more than `bg_sigma` standard deviations from its voxel's mean.
The LiDAR's few-centimetre ranging noise is absorbed into the std, so genuine
surfaces are not flagged as foreground. `bg_sigma` is 2.5.

**Voxel size matters more than it looks.** node3 was originally built at a 0.15 m
voxel and furniture leaked through as false detections — at 0.15 m a voxel
straddles a furniture edge, the std is inflated, and real points near that edge
fall inside the band and get dropped, or background points fall outside it and
get kept. Rebuilding node3 at **0.10 m** fixed the leakage. Use 0.10 m for nodes
prone to furniture leakage; the coarser 0.15 m is acceptable only where the
background is simple. Rebuild the model whenever fixed furniture moves — see
[operations.md](operations.md).

## Region of interest (ROI)

The ROI is an axis-aligned crop applied before clustering, removing walls and
out-of-area returns. The current bounds are **temporary and only need to be good
enough** for the development room — they will be re-tuned for the actual Hunt
Library space, so they are not worth over-fitting now.

The bounds are per-node (each LiDAR sees the shared volume from its own origin,
so node1 and node3 carry different `roi` blocks), and `roi_calibrate.py` helps
find them: it collects a few seconds of foreground and prints the xyz range plus
a text histogram so walls are visible as dense bands to crop out.

## Detection parameters

**Adaptive frame accumulation.** Sparse overhead point clouds need several frames
accumulated to form a person-sized cluster, but accumulating too many smears a
walking person across space. Accumulation is therefore adaptive: stationary
targets accumulate up to `max_frames: 6`, fast movers as few as `min_frames: 1`,
switching on speed (`low_speed_thr: 0.2`, `high_speed_thr: 0.3` m/s) with targets
associated across frames within `assoc_radius: 1.5` m. This is a deliberate
scene-appropriate adaptation, not a fixed convenience value.

**Clustering.** Euclidean clustering with `cluster_tol: 0.6` m (see the
research-critical constraint above — do not raise this), `min_points: 20` to
reject sparse noise, and a per-frame cap of `max_persons: 10`.

**Cluster shape validation.** A cluster is accepted as a person only if its
footprint and height are person-like: `min_xy_size: 0.10` / `max_xy_size: 1.0` m,
`max_aspect_ratio: 4.0`, and vertical extent between `min_vertical_extent: 0.50`
and `max_vertical_extent: 2.2` m. The lower vertical bound rejects floor and
low-furniture noise; the upper bound rejects spuriously merged tall clusters.

**Exclusion zones.** A small number of fixed circular zones suppress known static
clutter that survives background removal (e.g. a fixture that the statistical
model can't fully account for). These are deliberately small and few — a large
exclusion zone risks swallowing a real person, which violates the constraint
above.

## Tracking parameters

Tracking is AB3DMOT-style: Kalman prediction plus Hungarian association, with
several scene-appropriate additions.

**Track lifetime and Re-ID.** `max_age: 30` frames sets how long a track coasts
before death. It was reduced from 50 to 30 because geometric Re-ID
(`reid_thr: 0.5`, revivable for `reid_max_age_sec: 15.0` s) now recovers tracks
that disappear for longer absences, so a long `max_age` is no longer needed to
bridge gaps — and a shorter one produces fewer stale ghost tracks.
`min_hits: 3` before a track is confirmed; `max_association_dist: 0.8` m for the
Hungarian step.

**Ghost-track suppression.** Duplicate boxes on a single person were eliminated
(~95%) with track-merge logic: tracks closer than `merge_dist: 0.40` m for at
least `merge_min_frames: 5` frames are merged. The merge distance is well below
the social distances being studied, so it never merges two real people — that
margin is the whole reason it is safe.

**Direction-aided association** (`direction_weight: 0.5`,
`direction_min_speed: 0.3`) biases association toward a track's direction of
travel, and **velocity-decay coasting** (`coast_velocity_decay: 0.6`) damps a
coasting track's predicted velocity each frame so a lost track does not sail off
in a straight line. Both are deliberate adaptations to the scene, and are worth
foregrounding as such rather than treating as defaults.

## Static-person handling

A person sitting still looks, to a tracker built to suppress static clutter, much
like furniture — and early on, seated people were being filtered out as
background. The `StaticZoneFilter` resolves this with a **whitelist**: a confirmed
track in a region keeps that region from being suppressed, so a known person who
stops moving is not re-classified as furniture.

The relevant settings: `static_suppress_frames: 20` and `static_suppress_dist:
0.30` govern when a static region is suppressed, and the static-zone history
parameters (history 30, radius 0.5, density 0.75, min_history 15) govern how a
zone is characterised. The net effect is that genuine static clutter is still
suppressed, but a seated participant is not — which matters because seated
conversations are precisely the kind of interaction the study cares about.

## Behaviour classification

Behaviour is classified over a sliding window (`window_frames: 30`, usable from
`min_window_frames: 15`) into walking / stationary / talking, with
`behavior_hold_frames: 5` to avoid flicker. Walking is gated at
`walk_speed_threshold: 0.5` m/s; a "talking" candidate requires two people within
`talk_distance_threshold: 1.5` m for at least `talk_duration_sec: 30.0` s.

**Sitting and standing are deliberately merged into a single "stationary"
label.** Distinguishing them would rely on vertical extent, which is unreliable
in sparse overhead point clouds at distance — the same sparsity that the adaptive
accumulation above works around. Rather than emit an unreliable sit/stand
distinction, the pipeline reports "stationary" and leaves the finer distinction
out. This is a design decision, not a missing feature.
