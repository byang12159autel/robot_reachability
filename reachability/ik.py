"""Numerical inverse kinematics (Pinocchio, damped least-squares).

A 6-DOF pose IK with joint-limit clamping and random restarts. Used by the
Stage-C reachability map to test, per (position, orientation) target, whether a
joint solution exists within the URDF limits.

Solver weakness: like all numerical IK it can miss a solution that exists near
singularities / limit boundaries (false negatives). Restarts mitigate this; the
FR3 analytical IK would remove it entirely.
"""
from __future__ import annotations

import numpy as np
import pinocchio as pin


def solve_ik(model, data, fid, oMdes, q0, lower, upper,
             iters: int = 120, eps: float = 2e-3, damp: float = 1e-6):
    """Damped-least-squares IK from seed q0 toward SE3 target oMdes.

    Returns (converged_within_limits, q). q is clamped to [lower, upper] each step.
    """
    q = np.clip(q0.copy(), lower, upper)
    I6 = np.eye(6)
    for _ in range(iters):
        pin.forwardKinematics(model, data, q)
        pin.updateFramePlacement(model, data, fid)
        iMd = data.oMf[fid].actInv(oMdes)        # current -> desired, in local frame
        err = pin.log6(iMd).vector
        if np.linalg.norm(err) < eps:
            return True, q
        J = pin.computeFrameJacobian(model, data, q, fid)  # LOCAL
        J = -pin.Jlog6(iMd.inverse()) @ J
        dq = -J.T @ np.linalg.solve(J @ J.T + damp * I6, err)
        q = np.clip(pin.integrate(model, q, dq), lower, upper)

    pin.forwardKinematics(model, data, q)
    pin.updateFramePlacement(model, data, fid)
    err = pin.log6(data.oMf[fid].actInv(oMdes)).vector
    return bool(np.linalg.norm(err) < eps), q


def pose_reachable(model, data, fid, oMdes, lower, upper, rng,
                   restarts: int = 8, checker=None, **kw) -> bool:
    """True if a restart finds an in-limit IK solution for the target pose.

    If `checker` is given, the solution must also be self-collision-free; restarts
    keep searching the redundant null space for a non-colliding configuration.
    """
    mid = (lower + upper) / 2.0
    for k in range(restarts + 1):
        q0 = mid if k == 0 else lower + (upper - lower) * rng.random(model.nq)
        ok, q = solve_ik(model, data, fid, oMdes, q0, lower, upper, **kw)
        if ok and (checker is None or not checker.in_collision(q)):
            return True
    return False


if __name__ == "__main__":
    import time
    from pathlib import Path

    ROOT = Path(__file__).resolve().parent.parent
    model = pin.buildModelFromUrdf(str(ROOT / "robots/franka_fr3v2/franka_fr3v2.urdf"))
    data = model.createData()
    fid = model.getFrameId("fr3v2_link8")
    lower = np.asarray(model.lowerPositionLimit)
    upper = np.asarray(model.upperPositionLimit)
    rng = np.random.default_rng(0)

    # 1) round-trip: FK a random config -> IK should recover a valid solution
    hits, n = 0, 200
    for _ in range(n):
        q_true = lower + (upper - lower) * rng.random(model.nq)
        pin.forwardKinematics(model, data, q_true)
        pin.updateFramePlacement(model, data, fid)
        target = data.oMf[fid].copy()
        if pose_reachable(model, data, fid, target, lower, upper, rng):
            hits += 1
    print(f"round-trip reachable poses: {hits}/{n} = {hits / n:.1%}  (should be ~100%)")

    # 2) clearly-unreachable pose 5 m away -> should fail
    far = pin.SE3(np.eye(3), np.array([5.0, 0.0, 0.0]))
    print(f"5 m-away pose reachable? {pose_reachable(model, data, fid, far, lower, upper, rng)}"
          "  (should be False)")

    # 3) timing
    q_true = lower + (upper - lower) * rng.random(model.nq)
    pin.forwardKinematics(model, data, q_true); pin.updateFramePlacement(model, data, fid)
    target = data.oMf[fid].copy()
    t0 = time.time()
    for _ in range(2000):
        pose_reachable(model, data, fid, target, lower, upper, rng)
    print(f"~{(time.time() - t0) / 2000 * 1e3:.2f} ms per reachable-pose query (warm)")
