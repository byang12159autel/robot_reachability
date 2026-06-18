"""Self-collision checking with Pinocchio + coal.

Builds the collision geometry from the URDF's collision meshes, enables all link
pairs, then removes the allowed pairs listed in the robot's SRDF (adjacent links
and pairs that can never touch). What remains are the meaningful self-collision
pairs.
"""
from __future__ import annotations

import contextlib
import os
from pathlib import Path

import numpy as np
import pinocchio as pin


@contextlib.contextmanager
def _chdir(path: Path):
    """Resolve the URDF's relative mesh paths (`meshes/...`) against its own dir."""
    prev = os.getcwd()
    os.chdir(path)
    try:
        yield
    finally:
        os.chdir(prev)


class SelfCollisionChecker:
    def __init__(self, urdf_path: Path, srdf_path: Path):
        urdf_path = Path(urdf_path)
        self.model = pin.buildModelFromUrdf(str(urdf_path))
        with _chdir(urdf_path.parent):
            self.geom = pin.buildGeomFromUrdf(
                self.model, str(urdf_path), pin.GeometryType.COLLISION,
                package_dirs=[str(urdf_path.parent)],
            )
        self.geom.addAllCollisionPairs()
        if srdf_path is not None:
            pin.removeCollisionPairs(self.model, self.geom, str(srdf_path))
        self.data = self.model.createData()
        self.geom_data = self.geom.createData()
        self.n_pairs = len(self.geom.collisionPairs)

    def in_collision(self, q: np.ndarray) -> bool:
        """True if configuration q is in self-collision (stops at first hit)."""
        return pin.computeCollisions(
            self.model, self.data, self.geom, self.geom_data, q, True
        )


if __name__ == "__main__":
    # quick self-test
    ROOT = Path(__file__).resolve().parent.parent
    urdf = ROOT / "robots" / "franka_fr3v2" / "franka_fr3v2.urdf"
    srdf = ROOT / "robots" / "franka_fr3v2" / "franka_fr3v2.srdf"
    chk = SelfCollisionChecker(urdf, srdf)
    print(f"collision geometries: {len(chk.geom.geometryObjects)}")
    print(f"active collision pairs (after SRDF prune): {chk.n_pairs}")
    lo = np.asarray(chk.model.lowerPositionLimit)
    hi = np.asarray(chk.model.upperPositionLimit)
    mid = (lo + hi) / 2
    print(f"midpoint config in collision? {chk.in_collision(mid)}")
    rng = np.random.default_rng(0)
    n = 20000
    hits = sum(chk.in_collision(lo + (hi - lo) * rng.random(chk.model.nq)) for _ in range(n))
    print(f"random configs in collision: {hits}/{n} = {hits / n:.1%}")
