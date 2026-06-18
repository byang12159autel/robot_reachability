#!/usr/bin/env python3
"""Stage C: reachability-index map (needs IK).

Takes the Stage-A reachable positions (collision-free if available), voxelizes
them, and at each voxel center samples orientations over SO(3) and runs IK. The
reachability index of a voxel = (orientations with an in-limit IK solution) /
(orientations tested). index 1.0 => reachable in every sampled orientation
(dexterous); 0.0 => position reachable but no tested orientation solvable.

Joint-limit feasibility only (no collision on the IK solutions).

Usage:
    pixi run reachmap-fr3
    python reachability/reach_map.py franka_fr3v2 --res 0.075 --orient 32 --max-voxels 300
"""
from __future__ import annotations

import argparse
import time

import numpy as np
import pinocchio as pin

from robots import ROBOTS, RESULTS
from fk_reachability import load_model
from ik import pose_reachable


def occupied_voxels(points: np.ndarray, res: float) -> np.ndarray:
    keys = np.unique(np.floor(points / res).astype(np.int64), axis=0)
    return (keys + 0.5) * res  # cell centers


def uniform_rotations(n: int, rng) -> list[np.ndarray]:
    """n uniformly-distributed rotation matrices (Shoemake's method)."""
    u1, u2, u3 = rng.random(n), rng.random(n), rng.random(n)
    x = np.sqrt(1 - u1) * np.sin(2 * np.pi * u2)
    y = np.sqrt(1 - u1) * np.cos(2 * np.pi * u2)
    z = np.sqrt(u1) * np.sin(2 * np.pi * u3)
    w = np.sqrt(u1) * np.cos(2 * np.pi * u3)
    out = []
    for i in range(n):
        q = pin.Quaternion(float(w[i]), float(x[i]), float(y[i]), float(z[i]))
        q.normalize()
        out.append(q.matrix())
    return out


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("robot", nargs="?", default="franka_fr3v2", choices=sorted(ROBOTS))
    ap.add_argument("--res", type=float, default=0.075, help="voxel size (m)")
    ap.add_argument("--orient", type=int, default=32, help="orientations sampled per voxel")
    ap.add_argument("--restarts", type=int, default=8, help="IK restarts per pose")
    ap.add_argument("--collision", action="store_true",
                    help="require a self-collision-free IK solution (needs SRDF)")
    ap.add_argument("--max-voxels", type=int, default=0, help="subsample voxels (0 = all)")
    ap.add_argument("--seed", type=int, default=0)
    args = ap.parse_args()

    cfg = ROBOTS[args.robot]
    model, data, fid, lower, upper = load_model(cfg["urdf"], cfg["ee_frame"])

    src = RESULTS / f"{args.robot}_reach.npz"
    if not src.exists():
        raise SystemExit(f"missing {src}; run Stage A first (pixi run reach-fr3-sc)")
    d = np.load(src)
    pts = d["points"]
    if "collision_free" in d.files:
        pts = pts[d["collision_free"]]
    centers = occupied_voxels(pts, args.res)

    checker = None
    if args.collision:
        from collision import SelfCollisionChecker
        if not cfg.get("srdf"):
            raise SystemExit(f"no SRDF registered for {args.robot}")
        checker = SelfCollisionChecker(cfg["urdf"], cfg["srdf"])

    rng = np.random.default_rng(args.seed)
    if args.max_voxels and len(centers) > args.max_voxels:
        centers = centers[rng.choice(len(centers), args.max_voxels, replace=False)]
    rots = uniform_rotations(args.orient, rng)

    mode = "self-collision-checked" if checker else "joint-limit-only"
    print(f"{args.robot}: {len(centers):,} voxels @ {args.res*100:.1f}cm × "
          f"{args.orient} orientations = {len(centers)*args.orient:,} IK queries ({mode})")

    index = np.zeros(len(centers), dtype=np.float32)
    t0 = time.time()
    step = max(len(centers) // 20, 1)
    for vi, c in enumerate(centers):
        solved = sum(
            pose_reachable(model, data, fid, pin.SE3(R, c), lower, upper, rng,
                           restarts=args.restarts, checker=checker)
            for R in rots
        )
        index[vi] = solved / args.orient
        if (vi + 1) % step == 0:
            el = time.time() - t0
            eta = el / (vi + 1) * (len(centers) - vi - 1)
            print(f"  {(vi+1)/len(centers):4.0%}  {vi+1}/{len(centers)}  "
                  f"{el:5.0f}s elapsed  ~{eta:4.0f}s left", flush=True)

    dt = time.time() - t0
    vox_vol = args.res ** 3
    print(f"\n=== reachability index ({dt:.0f}s, {dt/max(len(centers)*args.orient,1)*1e3:.2f} ms/query) ===")
    print(f"  voxels                 : {len(centers):,}")
    print(f"  index  mean / median   : {index.mean():.3f} / {np.median(index):.3f}")
    for thr in (0.99, 0.9, 0.5, 0.0):
        n = int((index >= thr - 1e-9).sum()) if thr > 0 else int((index > 0).sum())
        label = f"index >= {thr}" if thr > 0 else "index > 0"
        print(f"  {label:<16}: {n:6,} voxels  ({n*vox_vol:.3f} m^3)")

    RESULTS.mkdir(exist_ok=True)
    out = RESULTS / f"{args.robot}_reachmap.npz"
    np.savez_compressed(out, centers=centers.astype(np.float32), index=index,
                        res=args.res, n_orient=args.orient, robot=args.robot,
                        collision_checked=bool(args.collision))
    print(f"\nSaved -> {out.relative_to(RESULTS.parent)}")


if __name__ == "__main__":
    main()
