#!/usr/bin/env python3
"""
calib_visualize.py — Stage 1 of LiDAR calibration.

Reads the two statistical background models (node1 = reference, node3 = to be
aligned), extracts the per-voxel mean points (the real static structure: walls,
floor, pillars), and renders top-down + side views so you can judge the rough
initial transform before running ICP.

This is fully OFFLINE — no rosbridge, no Foxglove, no network. Just the two .npz
files on disk.

Run from repo root:
    python3 pipeline/06_fusion/calib_visualize.py \
        --node1 models/background_statistical_node1.npz \
        --node3 models/background_statistical_node3.npz

Outputs (in ./calib_out/):
    node1_topdown.png      node1 alone, top-down
    node3_topdown.png      node3 alone, top-down
    overlay_identity.png   both, no transform (how they sit now = misaligned)
"""
import argparse
import os
import numpy as np

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt


def load_points(npz_path):
    """Extract per-voxel mean points = real static structure coordinates."""
    d = np.load(npz_path, allow_pickle=True)
    pts = np.asarray(d["means"], dtype=np.float64)  # (N,3)
    vs = float(d["voxel_size"])
    return pts, vs


def topdown(ax, pts, color, label):
    ax.scatter(pts[:, 0], pts[:, 1], s=2, c=color, label=label, alpha=0.6)
    ax.set_aspect("equal")
    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.grid(True, alpha=0.3)
    # mark the sensor origin (0,0) — each LiDAR is its own origin
    ax.scatter([0], [0], s=120, c="red", marker="x", linewidths=3,
               zorder=5, label=f"{label} origin")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--node1", default="models/background_statistical_node1.npz")
    ap.add_argument("--node3", default="models/background_statistical_node3.npz")
    ap.add_argument("--outdir", default="calib_out")
    args = ap.parse_args()
    os.makedirs(args.outdir, exist_ok=True)

    p1, vs1 = load_points(args.node1)
    p3, vs3 = load_points(args.node3)

    print(f"node1: {len(p1):5d} pts | voxel {vs1:.2f} | "
          f"X[{p1[:,0].min():.1f},{p1[:,0].max():.1f}] "
          f"Y[{p1[:,1].min():.1f},{p1[:,1].max():.1f}] "
          f"Z[{p1[:,2].min():.1f},{p1[:,2].max():.1f}]")
    print(f"node3: {len(p3):5d} pts | voxel {vs3:.2f} | "
          f"X[{p3[:,0].min():.1f},{p3[:,0].max():.1f}] "
          f"Y[{p3[:,1].min():.1f},{p3[:,1].max():.1f}] "
          f"Z[{p3[:,2].min():.1f},{p3[:,2].max():.1f}]")

    # 1) node1 alone, top-down
    fig, ax = plt.subplots(figsize=(9, 9))
    topdown(ax, p1, "tab:orange", "node1")
    ax.set_title("node1 background — top-down (X-Y)")
    ax.legend()
    fig.savefig(f"{args.outdir}/node1_topdown.png", dpi=120, bbox_inches="tight")
    plt.close(fig)

    # 2) node3 alone, top-down
    fig, ax = plt.subplots(figsize=(9, 9))
    topdown(ax, p3, "tab:blue", "node3")
    ax.set_title("node3 background — top-down (X-Y)")
    ax.legend()
    fig.savefig(f"{args.outdir}/node3_topdown.png", dpi=120, bbox_inches="tight")
    plt.close(fig)

    # 3) overlay with IDENTITY (how they sit now — misaligned, origins coincide)
    fig, ax = plt.subplots(figsize=(11, 11))
    ax.scatter(p1[:, 0], p1[:, 1], s=2, c="tab:orange", alpha=0.5, label="node1 (ref)")
    ax.scatter(p3[:, 0], p3[:, 1], s=2, c="tab:blue", alpha=0.5, label="node3 (to align)")
    ax.scatter([0], [0], s=150, c="red", marker="x", linewidths=3, zorder=5,
               label="shared origin (identity)")
    ax.set_aspect("equal")
    ax.set_xlabel("X (m)"); ax.set_ylabel("Y (m)")
    ax.grid(True, alpha=0.3)
    ax.set_title("IDENTITY overlay — both origins forced to (0,0).\n"
                 "This is why they 'coincide'. Calibration must push node3 to its real diagonal spot.")
    ax.legend()
    fig.savefig(f"{args.outdir}/overlay_identity.png", dpi=120, bbox_inches="tight")
    plt.close(fig)

    print(f"\nSaved 3 PNGs to {args.outdir}/:")
    print("  node1_topdown.png, node3_topdown.png, overlay_identity.png")
    print("\nLook at node1_topdown vs node3_topdown:")
    print("  - find a SHARED feature (a long wall, a corner) in both")
    print("  - judge how much node3 must ROTATE (degrees about vertical/Z) and")
    print("    TRANSLATE to make its walls line up with node1's walls.")
    print("  That rough rotation+translation is the ICP initial guess.")


if __name__ == "__main__":
    main()
