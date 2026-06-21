#!/usr/bin/env python3
"""
calib_roi_preview.py - Fused top-down (node1 + calibrated node3) with a
candidate ROI rectangle that can be ROTATED, so you can see how much the room
is tilted and whether a rotated ROI is actually needed.

Run from repo root:
    python3 pipeline/06_fusion/calib_roi_preview.py --xmin 0 --xmax 11 --ymin -5 --ymax 0 --angle 0
    python3 pipeline/06_fusion/calib_roi_preview.py --cx 5 --cy -2.5 --w 11 --h 5 --angle 8

Two ways to specify the box:
  (A) axis-aligned bounds: --xmin --xmax --ymin --ymax  (use --angle to rotate
      about the box center)
  (B) center + size: --cx --cy --w --h --angle

The green box rotates by --angle degrees (CCW) about its center. Watch how many
degrees make its edges line up with the room walls. If it's only a few degrees,
a slightly larger axis-aligned ROI is simpler than a rotated one.

Output: calib_out/roi_preview.png
"""
import argparse, os
import numpy as np
import matplotlib; matplotlib.use("Agg")
import matplotlib.pyplot as plt


def load_means(npz):
    d = np.load(npz, allow_pickle=True)
    return np.asarray(d["means"], dtype=np.float64)


def rot_rect(cx, cy, w, h, angle_deg):
    """Return 5 corner points (closed loop) of a rectangle centered at (cx,cy),
    size w x h, rotated angle_deg CCW about center."""
    a = np.radians(angle_deg)
    c, s = np.cos(a), np.sin(a)
    R = np.array([[c, -s], [s, c]])
    hw, hh = w/2, h/2
    corners = np.array([[-hw, -hh], [hw, -hh], [hw, hh], [-hw, hh], [-hw, -hh]])
    rc = (R @ corners.T).T + np.array([cx, cy])
    return rc


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--node1", default="models/background_statistical_node1.npz")
    ap.add_argument("--node3", default="models/background_statistical_node3.npz")
    ap.add_argument("--tf", default="calib_out/node3_to_node1.txt")
    # box via bounds OR via center+size
    ap.add_argument("--xmin", type=float, default=None)
    ap.add_argument("--xmax", type=float, default=None)
    ap.add_argument("--ymin", type=float, default=None)
    ap.add_argument("--ymax", type=float, default=None)
    ap.add_argument("--cx", type=float, default=None)
    ap.add_argument("--cy", type=float, default=None)
    ap.add_argument("--w", type=float, default=None)
    ap.add_argument("--h", type=float, default=None)
    ap.add_argument("--angle", type=float, default=0.0, help="rotation deg CCW")
    ap.add_argument("--out", default="calib_out/roi_preview.png")
    args = ap.parse_args()
    os.makedirs(os.path.dirname(args.out), exist_ok=True)

    # resolve box center+size
    if args.cx is not None:
        cx, cy, w, h = args.cx, args.cy, args.w, args.h
    else:
        xmin = 0.0 if args.xmin is None else args.xmin
        xmax = 11.0 if args.xmax is None else args.xmax
        ymin = -5.0 if args.ymin is None else args.ymin
        ymax = 0.0 if args.ymax is None else args.ymax
        cx, cy = (xmin+xmax)/2, (ymin+ymax)/2
        w, h = xmax-xmin, ymax-ymin

    p1 = load_means(args.node1)
    p3 = load_means(args.node3)
    T = np.loadtxt(args.tf)
    p3h = (T[:3, :3] @ p3.T).T + T[:3, 3]

    fig, ax = plt.subplots(figsize=(13, 9))
    ax.scatter(p1[:, 0], p1[:, 1], s=2, c="tab:orange", alpha=.5, label="node1")
    ax.scatter(p3h[:, 0], p3h[:, 1], s=2, c="tab:blue", alpha=.5, label="node3 (calibrated)")
    ax.scatter([0], [0], s=160, c="red", marker="x", linewidths=3, zorder=6,
               label="node1 origin")

    rc = rot_rect(cx, cy, w, h, args.angle)
    ax.plot(rc[:, 0], rc[:, 1], "g--", lw=3, label=f"ROI {w:.1f}x{h:.1f}m @ {args.angle:.0f}deg")
    ft = 3.28084
    ax.text(cx, cy + h/2 + 0.4,
            f"{w:.1f}x{h:.1f}m = {w*ft:.0f}'x{h*ft:.0f}'  angle={args.angle:.0f}deg",
            ha="center", color="green", fontsize=12, weight="bold")

    ax.set_aspect("equal"); ax.set_xlabel("X (m)"); ax.set_ylabel("Y (m)")
    ax.grid(True, alpha=.3); ax.legend(loc="upper right")
    ax.set_title("Rotate --angle until green edges align with the room walls.\n"
                 "Few degrees -> use a slightly larger axis-aligned ROI instead.")
    fig.savefig(args.out, dpi=120, bbox_inches="tight")
    print(f"saved -> {args.out}")
    print(f"box center=({cx:.2f},{cy:.2f}) size={w:.1f}x{h:.1f}m angle={args.angle}deg")


if __name__ == "__main__":
    main()