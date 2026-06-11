# Eval: ATC Dataset Validation

## What this does

Two scripts validate the encounter detection algorithm against the publicly available ATC pedestrian dataset before deploying on live Hunt Library data.

**`load_atc.py`** loads ATC trajectory CSVs, applies the full encounter detection pipeline (displacement filter → artifact filter → velocity filter → proximity + heading check → deceleration filter → deduplication), and saves detected encounter pairs to `encounters_raw.csv`. It also generates a scatter-plot heatmap of encounter locations.

**`validation.py`** loads the algorithm output (`encounters_raw.csv`) and the ATC ground-truth file (`groups_ATC-1.dat`), then computes Precision, Recall, and F1 at the pair level (direction-agnostic: pair (A, B) == pair (B, A)).

## Dataset format

ATC CSV files (`person_ATC-*.csv`):

| Column | Unit | Description |
|---|---|---|
| timestamp | seconds | Frame timestamp |
| person_id | — | Unique person identifier |
| x | mm → m | X position (converted on load) |
| y | mm → m | Y position (converted on load) |
| z | mm | Z position (not converted) |
| velocity | mm/s → m/s | Speed (converted on load) |
| angle1 | radians | Movement heading direction |
| angle2 | radians | Body orientation angle |

Ground-truth file (`groups_ATC-1.dat`): space-separated records with `person_id`, `partner_id`, and interaction type. Only type 1 (genuine social interaction) is used.

## Usage

```bash
# Run encounter detection on ATC dataset
cd eval
python3 load_atc.py
# Outputs: encounters_raw.csv, collision_heatmap.png, atc_first_frame.png

# Evaluate against ground truth
python3 validation.py
# Outputs: Precision / Recall / F1 printed to stdout
```

## Expected files

```
dataset/ATC_dataset/
├── person_ATC-1_1000.csv   # trajectory data
├── person_ATC-1_1200.csv
├── person_ATC-1_1500.csv
├── person_ATC-1_1900.csv
└── groups_ATC-1.dat        # ground-truth interaction pairs
data/encounters/
└── encounters_raw.csv      # algorithm output (input to validation.py)
```

## Benchmark reference

The paper baseline cited in `validation.py` is **Precision = 0.861**.
