#!/usr/bin/env python3
"""
ATC Validation: Precision / Recall against groups_ATC-1.dat ground truth.

Bug fixed (2026-06-15): the original code compared detected pairs against
ALL GT pairs from the full-day groups file, inflating the denominator and
making precision appear artificially low (~0.015).  The fix filters GT to
only pairs where BOTH persons appear in the evaluated session trajectory,
so the denominator reflects only what the algorithm could realistically have
detected.

Usage:
    cd <project_root>
    python3 eval/validation.py \
        --gt   dataset/ATC_dataset/groups_ATC-1.dat \
        --det  data/encounters/encounters_raw.csv \
        --session dataset/ATC_dataset/person_ATC-1_1000.csv
"""

import argparse
import sys
from pathlib import Path

import pandas as pd


# ─── Ground truth loader ───────────────────────────────────────────────────────

def load_ground_truth(filepath: str) -> set:
    """
    Read groups_ATC-1.dat and return the set of real social interaction pairs
    (interaction_type == 1).  Pairs are stored as frozenset so (A,B) == (B,A).
    """
    gt_pairs = set()
    with open(filepath) as f:
        for line in f:
            parts = line.strip().split()
            if len(parts) < 4:
                continue
            person_id  = int(parts[0])
            partner_id = int(parts[2])

            interaction_type = None
            for p in parts[3:]:
                if p in ("0", "1"):
                    interaction_type = int(p)
                    break

            if interaction_type == 1:
                gt_pairs.add(frozenset([person_id, partner_id]))

    return gt_pairs


# ─── Session person IDs ────────────────────────────────────────────────────────

def load_session_ids(filepath: str) -> set:
    """
    Load the ATC trajectory CSV used for this evaluation run and return the
    set of person_id values that appear in it.  Only pairs where BOTH persons
    are in this set can appear in the evaluation output; using the full-day GT
    without this filter inflates the GT denominator and crushes precision.
    """
    df = pd.read_csv(
        filepath,
        header=None,
        names=["timestamp", "person_id", "x", "y", "z",
               "velocity", "angle1", "angle2"],
    )
    return set(df["person_id"].unique())


# ─── Detection loader ──────────────────────────────────────────────────────────

def load_detections(filepath: str) -> set:
    """Load encounters_raw.csv and return detected pairs as a set of frozensets."""
    df = pd.read_csv(filepath)
    detected_pairs = set()
    for _, row in df.iterrows():
        pair = frozenset([int(row["person1"]), int(row["person2"])])
        detected_pairs.add(pair)
    return detected_pairs


# ─── Evaluation ───────────────────────────────────────────────────────────────

def evaluate(detected_pairs: set, gt_pairs_all: set,
             session_ids: set) -> tuple:
    """
    Compute precision/recall using session-filtered GT.

    The full-day GT file covers every person seen that day.  We restrict it to
    pairs where BOTH persons appear in session_ids so the denominator reflects
    only pairs the algorithm had a chance to observe.
    """
    gt_pairs_session = {
        p for p in gt_pairs_all
        if p.issubset(session_ids)
    }

    tp = detected_pairs & gt_pairs_session
    fp = detected_pairs - gt_pairs_session
    fn = gt_pairs_session - detected_pairs

    precision = len(tp) / (len(tp) + len(fp)) if detected_pairs else 0.0
    recall    = len(tp) / (len(tp) + len(fn)) if gt_pairs_session else 0.0
    f1        = (2 * precision * recall / (precision + recall)
                 if (precision + recall) else 0.0)

    print("=" * 50)
    print(f"GT pairs in full file:    {len(gt_pairs_all)}")
    print(f"GT pairs in this session: {len(gt_pairs_session)}")
    print(f"Detected pairs:           {len(detected_pairs)}")
    print(f"True positives:           {len(tp)}")
    print(f"False positives:          {len(fp)}")
    print(f"False negatives:          {len(fn)}")
    print("-" * 50)
    print(f"Precision:  {precision:.3f}")
    print(f"Recall:     {recall:.3f}")
    print(f"F1 Score:   {f1:.3f}")
    print("=" * 50)
    print("Flack 2013 benchmark: Precision=0.861")

    return precision, recall, f1


# ─── main ─────────────────────────────────────────────────────────────────────

def main():
    # Resolve paths relative to the project root (one level up from eval/)
    _here = Path(__file__).resolve().parent
    _root = _here.parent

    default_gt  = str(_root / "dataset" / "ATC_dataset" / "groups_ATC-1.dat")
    default_det = str(_root / "data"    / "encounters"  / "encounters_raw.csv")
    default_ses = str(_root / "dataset" / "ATC_dataset" / "person_ATC-1_1000.csv")

    parser = argparse.ArgumentParser(
        description="ATC precision/recall evaluation with session-filtered GT",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    parser.add_argument("--gt",      default=default_gt,  metavar="FILE",
                        help=f"Ground truth file (default: {default_gt})")
    parser.add_argument("--det",     default=default_det, metavar="FILE",
                        help=f"Detected encounters CSV (default: {default_det})")
    parser.add_argument("--session", default=default_ses, metavar="FILE",
                        help=f"ATC trajectory CSV used for this run "
                             f"(default: {default_ses})")
    args = parser.parse_args()

    # Verify files exist before loading
    for label, path in [("GT", args.gt), ("detections", args.det),
                         ("session trajectory", args.session)]:
        if not Path(path).exists():
            print(f"ERROR: {label} file not found: {path}")
            sys.exit(1)

    print("Loading ground truth ...")
    gt_pairs_all = load_ground_truth(args.gt)
    print(f"  Total GT social pairs (full day): {len(gt_pairs_all)}")

    print("Loading session trajectory to get active person IDs ...")
    session_ids = load_session_ids(args.session)
    print(f"  Persons in session: {len(session_ids)}")

    print("Loading detections ...")
    detected_pairs = load_detections(args.det)
    print(f"  Detected pairs: {len(detected_pairs)}")

    print()
    print("Evaluation (GT filtered to session persons):")
    evaluate(detected_pairs, gt_pairs_all, session_ids)


if __name__ == "__main__":
    main()
