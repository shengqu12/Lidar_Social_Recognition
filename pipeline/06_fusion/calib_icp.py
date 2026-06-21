#!/usr/bin/env python3
"""
calib_icp.py v2 - ICP calibration with PHYSICAL PRIOR (180 deg SE<->NW mount).

Why v2: a rectangular room is ~symmetric under 180 deg, so unconstrained ICP
fell into a near-identity local minimum (3 deg rotation) with a deceptively high
fitness (0.808) - physically WRONG, since the two LiDARs are mounted SE vs NW,
~180 deg apart in heading. node3 also had roll=180 set in its livox config.

This version seeds ONLY physically-plausible orientations (180 yaw, with/without
180 roll) plus the tape-measured SE<->NW translation, runs ICP from each, and
judges the result by BOTH fitness AND a non-symmetric diagonal-corridor overlap
metric (symmetry cannot fake that). It prints the top candidates so we pick the
physically-correct one, not just the highest fitness.

Run:
    python3 pipeline/06_fusion/calib_icp.py --tx 7.32 --ty 3.71
"""
import argparse, os
import numpy as np
import open3d as o3d


def load_pcd(npz_path, voxel_down=0.10):
    d = np.load(npz_path, allow_pickle=True)
    pts = np.asarray(d["means"], dtype=np.float64)
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(pts)
    if voxel_down > 0:
        pcd = pcd.voxel_down_sample(voxel_down)
    pcd.estimate_normals(
        search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.3, max_nn=30))
    return pcd


def roll_yaw_T(roll_deg, yaw_deg, tx, ty):
    rr, ry = np.radians(roll_deg), np.radians(yaw_deg)
    Rx = np.array([[1, 0, 0],
                   [0, np.cos(rr), -np.sin(rr)],
                   [0, np.sin(rr),  np.cos(rr)]])
    Rz = np.array([[np.cos(ry), -np.sin(ry), 0],
                   [np.sin(ry),  np.cos(ry), 0],
                   [0, 0, 1]])
    T = np.eye(4); T[:3, :3] = Rz @ Rx; T[:3, 3] = [tx, ty, 0]
    return T


def yaw_of(T):
    return np.degrees(np.arctan2(T[1, 0], T[0, 0]))


def corridor_score(p_aligned, p_ref, region):
    """Overlap in a NON-symmetric region (the diagonal corridor).
    region = (xmin,xmax,ymin,ymax). Fraction of aligned points in the region
    with a ref point within 0.4 m. Symmetry cannot fake this."""
    xmin, xmax, ymin, ymax = region
    def crop(p):
        m = (p[:,0]>=xmin)&(p[:,0]<=xmax)&(p[:,1]>=ymin)&(p[:,1]<=ymax)
        return p[m]
    a, r = crop(p_aligned), crop(p_ref)
    if len(a) == 0 or len(r) == 0:
        return 0.0, len(a), len(r)
    tree = o3d.geometry.KDTreeFlann(
        o3d.geometry.PointCloud(o3d.utility.Vector3dVector(r)))
    hit = 0
    for q in a:
        k, idx, d2 = tree.search_knn_vector_3d(q, 1)
        if k > 0 and d2[0] <= 0.4**2:
            hit += 1
    return hit/len(a), len(a), len(r)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--node1", default="models/background_statistical_node1.npz")
    ap.add_argument("--node3", default="models/background_statistical_node3.npz")
    ap.add_argument("--tx", type=float, default=7.32)
    ap.add_argument("--ty", type=float, default=3.71)
    ap.add_argument("--threshold", type=float, default=0.4)
    ap.add_argument("--outdir", default="calib_out")
    ap.add_argument("--corr", default="2,9,2.5,6.5",
                    help="non-symmetric corridor region xmin,xmax,ymin,ymax")
    args = ap.parse_args()
    os.makedirs(args.outdir, exist_ok=True)
    region = tuple(float(v) for v in args.corr.split(","))

    target = load_pcd(args.node1)
    source = load_pcd(args.node3)
    p1 = np.asarray(target.points)
    print(f"node1(target): {len(target.points)} | node3(source): {len(source.points)}")

    inits = []
    trans = [(args.tx, args.ty), (-args.tx, -args.ty),
             (args.tx, -args.ty), (-args.tx, args.ty), (0.0, 0.0)]
    for (tx, ty) in trans:
        for (roll, yaw) in [(0, 180), (180, 180), (180, 0)]:
            inits.append((roll, yaw, tx, ty, roll_yaw_T(roll, yaw, tx, ty)))

    crit = o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=300)
    results = []
    for (roll, yaw, tx, ty, T0) in inits:
        reg = o3d.pipelines.registration.registration_icp(
            source, target, args.threshold, T0,
            o3d.pipelines.registration.TransformationEstimationPointToPlane(), crit)
        T = reg.transformation
        p3h = (T[:3,:3] @ np.asarray(source.points).T).T + T[:3,3]
        corr, na, nr = corridor_score(p3h, p1, region)
        results.append(dict(roll=roll, yaw=yaw, tx=tx, ty=ty,
                            fitness=reg.fitness, rmse=reg.inlier_rmse,
                            corr=corr, na=na, T=T))

    results.sort(key=lambda r: (r["corr"], r["fitness"]), reverse=True)

    print("\n=== TOP CANDIDATES (ranked by corridor overlap, then fitness) ===")
    print(f"{'init(roll,yaw,tx,ty)':30s} {'fit':>7} {'rmse':>7} {'corridor':>9} {'finalYaw':>9}")
    for r in results[:6]:
        print(f"(r={r['roll']:>3},y={r['yaw']:>3},tx={r['tx']:>5.1f},ty={r['ty']:>5.1f})  "
              f"{r['fitness']:7.3f} {r['rmse']:7.3f} {r['corr']:9.3f} {yaw_of(r['T']):9.1f}")

    best = results[0]
    T = best["T"]
    print("\n=== CHOSEN (highest corridor overlap) ===")
    print(f"  init: roll={best['roll']} yaw={best['yaw']} tx={best['tx']} ty={best['ty']}")
    print(f"  fitness={best['fitness']:.3f}  rmse={best['rmse']:.3f}  "
          f"corridor_overlap={best['corr']:.3f}  final_yaw={yaw_of(T):.1f} deg")
    print("  transform node3 -> node1 (4x4):")
    print(np.array2string(T, precision=5, suppress_small=True))
    np.savetxt(f"{args.outdir}/node3_to_node1.txt", T, fmt="%.6f")
    print(f"  saved -> {args.outdir}/node3_to_node1.txt")

    import matplotlib; matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    p3 = np.asarray(source.points)
    p3h = (T[:3,:3] @ p3.T).T + T[:3,3]
    fig, (a1, a2) = plt.subplots(1, 2, figsize=(20, 10))
    a1.scatter(p1[:,0], p1[:,1], s=2, c="tab:orange", alpha=.5, label="node1")
    a1.scatter(p3[:,0], p3[:,1], s=2, c="tab:blue", alpha=.5, label="node3 (raw)")
    a1.set_title("BEFORE"); a1.set_aspect("equal"); a1.legend(); a1.grid(alpha=.3)
    a2.scatter(p1[:,0], p1[:,1], s=2, c="tab:orange", alpha=.5, label="node1")
    a2.scatter(p3h[:,0], p3h[:,1], s=2, c="tab:blue", alpha=.5, label="node3 (aligned)")
    xmin,xmax,ymin,ymax = region
    a2.plot([xmin,xmax,xmax,xmin,xmin],[ymin,ymin,ymax,ymax,ymin],
            "g--", lw=2, label="corridor check")
    a2.set_title(f"AFTER  yaw={yaw_of(T):.0f}deg  fit={best['fitness']:.3f} "
                 f"rmse={best['rmse']:.3f}  corridor={best['corr']:.2f}")
    a2.set_aspect("equal"); a2.legend(); a2.grid(alpha=.3)
    fig.savefig(f"{args.outdir}/icp_result.png", dpi=120, bbox_inches="tight")
    plt.close(fig)
    print(f"  saved overlay -> {args.outdir}/icp_result.png")
    print("\nJudge by AFTER panel: inside the green corridor box, do orange and")
    print("blue overlap? And is final_yaw near +/-180 (matching SE-vs-NW)?")
    print("If yes -> real calibration. If corridor overlap low -> tell me.")


if __name__ == "__main__":
    main()