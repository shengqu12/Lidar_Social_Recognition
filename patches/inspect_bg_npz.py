#!/usr/bin/env python3
"""
inspect_bg_npz.py - Print the internal structure of the statistical background
model .npz files so we know how to extract voxel-center points for ICP.

Run from the repo root:
    python3 inspect_bg_npz.py models/background_statistical_node1.npz
    python3 inspect_bg_npz.py models/background_statistical_node3.npz
"""
import sys
import numpy as np

if len(sys.argv) < 2:
    sys.exit("usage: python3 inspect_bg_npz.py <model.npz>")

path = sys.argv[1]
data = np.load(path, allow_pickle=True)

print(f"\n=== {path} ===")
print(f"keys: {list(data.keys())}")
for k in data.keys():
    arr = data[k]
    try:
        print(f"\n  [{k}]")
        print(f"    shape : {getattr(arr, 'shape', 'scalar')}")
        print(f"    dtype : {getattr(arr, 'dtype', type(arr))}")
        flat = np.asarray(arr).ravel()
        if flat.size and np.issubdtype(flat.dtype, np.number):
            print(f"    range : {flat.min()} .. {flat.max()}")
            # show a few sample rows if it looks like coordinates
            a = np.asarray(arr)
            if a.ndim == 2 and a.shape[1] in (3, 4, 6):
                print(f"    first 3 rows:\n{a[:3]}")
            elif a.ndim == 1 and a.size <= 8:
                print(f"    values: {a}")
    except Exception as e:
        print(f"    (could not summarize: {e})")
