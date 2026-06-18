#!/usr/bin/env python3
"""Stage A + B: FK reachable envelope and manipulability map (no IK).

Monte-Carlo samples joint configurations uniformly within the URDF joint limits,
runs Pinocchio forward kinematics to the end-effector frame, and records:

  * the flange position           -> reachable position envelope (Stage A)
  * the frame Jacobian            -> Yoshikawa manipulability    (Stage B)

Results are saved to results/<robot>_reach.npz and workspace metrics printed.
This is a joint-limit-only analysis (no collision) -> an UPPER BOUND on the
reachable set.

Usage:
    pixi run reach-fr3
    python reachability/fk_reachability.py franka_fr3v2 -n 300000 --seed 0
"""
from __future__ import annotations

import argparse
import time

import numpy as np
import pinocchio as pin

from robots import ROBOTS, RESULTS


def load_model(urdf_path, ee_frame):
    model = pin.buildModelFromUrdf(str(urdf_path))
    if not model.existFrame(ee_frame):
        frames = [f.name for f in model.frames]
        raise SystemExit(f"frame {ee_frame!r} not in model; available: {frames}")
    data = model.createData()
    fid = model.getFrameId(ee_frame)

    lower = np.asarray(model.lowerPositionLimit, dtype=float)
    upper = np.asarray(model.upperPositionLimit, dtype=float)
    # Guard against unbounded (continuous) joints.
    unbounded = ~np.isfinite(lower) | ~np.isfinite(upper)
    lower = np.where(unbounded, -np.pi, lower)
    upper = np.where(unbounded, np.pi, upper)
    return model, data, fid, lower, upper


def manipulability(J: np.ndarray) -> float:
    """Yoshikawa manipulability sqrt(det(J Jᵀ)); clamp tiny negatives to 0."""
    return float(np.sqrt(max(np.linalg.det(J @ J.T), 0.0)))


def sample(model, data, fid, lower, upper, n, seed, checker=None):
    rng = np.random.default_rng(seed)
    qs = lower + (upper - lower) * rng.random((n, model.nq))

    points = np.empty((n, 3), dtype=np.float32)
    w_trans = np.empty(n, dtype=np.float32)
    w_full = np.empty(n, dtype=np.float32)
    free = np.ones(n, dtype=bool) if checker is not None else None

    LWA = pin.ReferenceFrame.LOCAL_WORLD_ALIGNED
    t0 = time.time()
    step = max(n // 10, 1)
    for i in range(n):
        q = qs[i]
        pin.computeJointJacobians(model, data, q)  # also runs FK
        pin.updateFramePlacements(model, data)
        points[i] = data.oMf[fid].translation
        J = pin.getFrameJacobian(model, data, fid, LWA)  # 6 x nv
        w_trans[i] = manipulability(J[:3])   # translational ellipsoid (m)
        w_full[i] = manipulability(J)        # full 6D (mixed units)
        if checker is not None:
            free[i] = not checker.in_collision(q)
        if (i + 1) % step == 0:
            done = (i + 1) / n
            print(f"  {done:4.0%}  ({i + 1}/{n})  {time.time() - t0:5.1f}s", flush=True)
    return points, w_trans, w_full, free


def voxel_volume(points: np.ndarray, res: float) -> tuple[float, int]:
    keys = np.floor(points / res).astype(np.int64)
    n_vox = len(np.unique(keys, axis=0))
    return n_vox * res**3, n_vox


def _envelope(points, res, label):
    r = np.linalg.norm(points, axis=1)            # distance from base origin (link0)
    r_xy = np.linalg.norm(points[:, :2], axis=1)  # horizontal radius
    vol, n_vox = voxel_volume(points, res)
    lo = points.min(0)
    hi = points.max(0)
    print(f"\n--- {label}  ({len(points):,} pts) ---")
    print(f"  reach radius |p|  : min {r.min():.3f}  max {r.max():.3f}  mean {r.mean():.3f} m")
    print(f"  horizontal radius : max {r_xy.max():.3f} m")
    print(f"  x / y / z range   : [{lo[0]:+.3f},{hi[0]:+.3f}] [{lo[1]:+.3f},{hi[1]:+.3f}] "
          f"[{lo[2]:+.3f},{hi[2]:+.3f}] m")
    print(f"  voxel volume @{res*100:.0f}cm: {vol:.3f} m^3 ({n_vox:,} voxels)")


def report(points, w_trans, w_full, res, free=None):
    print("\n=== reachable workspace (joint-limit-only) ===")
    _envelope(points, res, "all joint-limit samples")
    sel = slice(None)
    if free is not None:
        n, nf = len(points), int(free.sum())
        print(f"\n  self-collision: {n - nf:,}/{n:,} pruned ({(n - nf) / n:.1%}) "
              f"-> {nf:,} collision-free")
        _envelope(points[free], res, "collision-free")
        sel = free
    suffix = " (collision-free)" if free is not None else ""
    print(f"\n=== manipulability{suffix} ===")
    for name, w in (("translational (m)", w_trans[sel]), ("full 6D (mixed units)", w_full[sel])):
        print(f"  {name:<24}: min {w.min():.4f}  median {np.median(w):.4f}  max {w.max():.4f}")


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("robot", nargs="?", default="franka_fr3v2", choices=sorted(ROBOTS))
    ap.add_argument("-n", "--samples", type=int, default=300_000)
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--voxel", type=float, default=0.05, help="voxel size (m) for volume estimate")
    ap.add_argument("--collision", action="store_true",
                    help="prune self-colliding configs (needs an SRDF for the robot)")
    args = ap.parse_args()

    cfg = ROBOTS[args.robot]
    print(f"Robot: {args.robot}   EE frame: {cfg['ee_frame']}")
    model, data, fid, lower, upper = load_model(cfg["urdf"], cfg["ee_frame"])
    print(f"  DOF (nq/nv): {model.nq}/{model.nv}")
    print("  joint limits (rad):")
    for i, name in enumerate(model.names[1:]):  # names[0] == "universe"
        print(f"    {name:<16} [{lower[i]:+.3f}, {upper[i]:+.3f}]")
    checker = None
    if args.collision:
        from collision import SelfCollisionChecker
        if not cfg.get("srdf"):
            raise SystemExit(f"no SRDF registered for {args.robot}; cannot self-collision check")
        checker = SelfCollisionChecker(cfg["urdf"], cfg["srdf"])
        print(f"self-collision pruning ON: {checker.n_pairs} active link pairs")

    print(f"\nSampling {args.samples:,} configs...")
    points, w_trans, w_full, free = sample(
        model, data, fid, lower, upper, args.samples, args.seed, checker=checker
    )
    report(points, w_trans, w_full, args.voxel, free=free)

    RESULTS.mkdir(exist_ok=True)
    out = RESULTS / f"{args.robot}_reach.npz"
    save = dict(points=points, w_trans=w_trans, w_full=w_full,
                ee_frame=cfg["ee_frame"], robot=args.robot)
    if free is not None:
        save["collision_free"] = free
    np.savez_compressed(out, **save)
    print(f"\nSaved -> {out.relative_to(RESULTS.parent)}")


if __name__ == "__main__":
    main()
