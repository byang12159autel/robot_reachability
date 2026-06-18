#!/usr/bin/env python3
"""Stage D: rate-feasibility map — velocity & acceleration capacity (no IK).

At each sampled configuration:
  * joint VELOCITY limits bound the EE velocity:     ẋ = J·q̇,  |q̇| ≤ q̇_max
  * joint ACCELERATION limits bound the EE accel:    ẍ = J·q̈,  |q̈| ≤ q̈_max  (at q̇=0)

Each capacity is the smallest singular value of the limit-scaled position Jacobian
= the speed / acceleration guaranteed in *every* Cartesian direction (worst-
direction, a conservative inner bound on the true polytope), then clamped to the
robot's own Cartesian caps. Per voxel we keep the best value over the postures
reaching it (best-case posture).

Limits are the documented FR3 values (not in the URDF), from
frankarobotics.github.io/docs/robot_specifications.html, kept in the robot
registry (reachability/robots.py).

  velocity capacity     -> m/s
  acceleration capacity -> m/s²

Usage:
    pixi run rate-fr3
    python reachability/rate_map.py franka_fr3v2 -n 300000 --res 0.05 --collision
"""
from __future__ import annotations

import argparse
import time

import numpy as np
import pinocchio as pin

from robots import ROBOTS, RESULTS
from fk_reachability import load_model


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("robot", nargs="?", default="franka_fr3v2", choices=sorted(ROBOTS))
    ap.add_argument("-n", "--samples", type=int, default=300_000)
    ap.add_argument("--res", type=float, default=0.05, help="voxel size (m)")
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--collision", action="store_true",
                    help="only count self-collision-free postures (needs SRDF)")
    args = ap.parse_args()

    cfg = ROBOTS[args.robot]
    rl = cfg.get("rate_limits")
    if not rl or "qdd_max" not in rl:
        raise SystemExit(f"no rate_limits (qdd_max) registered for {args.robot}")
    model, data, fid, lower, upper = load_model(cfg["urdf"], cfg["ee_frame"])
    qd = np.asarray(rl["qd_max"], float)
    qdd = np.asarray(rl["qdd_max"], float)
    cart_v = rl.get("cart_v_max", np.inf)
    cart_a = rl.get("cart_a_max", np.inf)
    print(f"{args.robot}: q̇_max {qd} rad/s,  q̈_max {qdd} rad/s²")
    print(f"  Cartesian caps: {cart_v} m/s, {cart_a} m/s²")

    checker = None
    if args.collision:
        from collision import SelfCollisionChecker
        checker = SelfCollisionChecker(cfg["urdf"], cfg["srdf"])

    rng = np.random.default_rng(args.seed)
    qs = lower + (upper - lower) * rng.random((args.samples, model.nq))

    res = args.res
    LWA = pin.ReferenceFrame.LOCAL_WORLD_ALIGNED
    best: dict[tuple, list] = {}     # voxel key -> [vel_cap, acc_cap]
    kept = 0
    t0 = time.time()
    step = max(args.samples // 10, 1)
    for i in range(args.samples):
        q = qs[i]
        if checker is not None and checker.in_collision(q):
            continue
        pin.computeJointJacobians(model, data, q)
        pin.updateFramePlacements(model, data)
        p = data.oMf[fid].translation
        Jv = pin.getFrameJacobian(model, data, fid, LWA)[:3]          # 3 x nv
        vel = min(np.linalg.svd(Jv * qd, compute_uv=False)[-1], cart_v)    # σ_min, m/s
        acc = min(np.linalg.svd(Jv * qdd, compute_uv=False)[-1], cart_a)   # σ_min, m/s²
        key = tuple(np.floor(p / res).astype(np.int64))
        b = best.get(key)
        if b is None:
            best[key] = [vel, acc]
        else:
            b[0] = max(b[0], vel)
            b[1] = max(b[1], acc)
        kept += 1
        if (i + 1) % step == 0:
            print(f"  {(i+1)/args.samples:4.0%}  {time.time()-t0:5.1f}s  "
                  f"{len(best):,} voxels", flush=True)

    keys = np.array(list(best.keys()))
    centers = ((keys + 0.5) * res).astype(np.float32)
    vals = np.array(list(best.values()), dtype=np.float32)
    vel_cap, acc_cap = vals[:, 0], vals[:, 1]

    print(f"\n=== rate capacity (best posture per voxel, {kept:,} collision-free samples) ===")
    print(f"  voxels                  : {len(centers):,}")
    print(f"  velocity cap (m/s)      : min {vel_cap.min():.3f}  median {np.median(vel_cap):.3f}  max {vel_cap.max():.3f}")
    print(f"  acceleration cap (m/s²) : min {acc_cap.min():.2f}  median {np.median(acc_cap):.2f}  max {acc_cap.max():.2f}")

    RESULTS.mkdir(exist_ok=True)
    out = RESULTS / f"{args.robot}_rate.npz"
    np.savez_compressed(out, centers=centers, vel_cap=vel_cap, acc_cap=acc_cap,
                        res=res, robot=args.robot)
    print(f"\nSaved -> {out.relative_to(RESULTS.parent)}")


if __name__ == "__main__":
    main()
