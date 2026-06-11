# Step 04: Encounter Detection

## What this does

`collision_detection.py` (imported as `load_atc.py`) processes a pedestrian trajectory dataset and detects social encounter events using a multi-stage filter pipeline. It is currently developed and validated on the ATC dataset; the same logic is intended for live tracked trajectories once Step 03 (tracking) is complete.

## Input / Output

| | File / Topic |
|---|---|
| Input | ATC CSV: `person_ATC-*.csv` (trajectory data) |
| Input (validation) | `groups_ATC-1.dat` (ground-truth interaction pairs) |
| Output | `encounters_raw.csv` (detected encounter events) |

## Detection pipeline

1. **Displacement filter** — discard persons with total travel < 1.0 m (likely stationary objects)
2. **Artifact ID filter** — discard IDs appearing in more than 30% of total frames
3. **Velocity filter** — keep persons with average velocity between 0.1 and 3.0 m/s
4. **Proximity check** — flag pairs whose 2D distance is < 1.5 m at the same timestamp
5. **Heading check** — require heading angle difference >= 90 degrees (facing each other or crossing)
6. **Deceleration filter** — at least one person must slow down by > 0.2 m/s in the 10 frames before the encounter
7. **Deduplication** — the same pair is counted only once within a 3.0-second window

## Usage

```bash
# Run encounter detection on ATC dataset
python3 pipeline/04_encounter_detection/collision_detection.py
# Output: encounters_raw.csv, collision_heatmap.png
```

## Key parameters

| Parameter | Default | Effect |
|---|---|---|
| min_displacement | 1.0 m | Minimum total travel to count as a moving person |
| max_frame_ratio | 0.3 | Persons in >30% of frames are treated as artifacts |
| min_avg_velocity | 0.1 m/s | Lower bound for valid pedestrian |
| max_avg_velocity | 3.0 m/s | Upper bound for valid pedestrian |
| proximity | 1.5 m | 2D distance threshold for candidate encounter |
| heading_threshold | 90 degrees | Minimum angle difference between headings |
| decel_threshold | 0.2 m/s | Required velocity drop before encounter |
| time_window | 3.0 s | Deduplication window for the same pair |
