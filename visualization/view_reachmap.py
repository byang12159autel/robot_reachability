#!/usr/bin/env python3
"""Visualize the Stage-C reachability-index map in viser.

Shows one point per voxel center, colored by reachability index (0 = position
reachable but no tested orientation solvable, 1 = reachable in every sampled
orientation = dexterous). A "min index" slider filters the cloud so you can peel
away to the dexterous core; robot overlay has joint sliders.

Usage:
    pixi run view-reachmap
"""
from __future__ import annotations

import argparse
import time
from pathlib import Path

import numpy as np
import viser
import yourdfpy
from viser.extras import ViserUrdf

from robots import ROBOTS
from view_reachability import colormap, add_joint_sliders

ROOT = Path(__file__).resolve().parent.parent
RESULTS = ROOT / "results"


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("robot", nargs="?", default="franka_fr3v2", choices=sorted(ROBOTS))
    ap.add_argument("--port", type=int, default=8080)
    args = ap.parse_args()

    npz = RESULTS / f"{args.robot}_reachmap.npz"
    if not npz.exists():
        raise SystemExit(f"missing {npz}; run: pixi run reachmap-fr3")
    d = np.load(npz)
    centers, index, res = d["centers"], d["index"], float(d["res"])
    print(f"{args.robot}: {len(centers):,} voxels @ {res*100:.1f}cm, mean index {index.mean():.3f}")

    server = viser.ViserServer(host="0.0.0.0", port=args.port)
    server.scene.add_grid("/ground", width=2.0, height=2.0, cell_size=0.1)

    urdf = yourdfpy.URDF.load(str(ROBOTS[args.robot]))
    viser_urdf = ViserUrdf(server, urdf, root_node_name="/robot")

    ps = res * 0.9
    cloud = server.scene.add_point_cloud(
        "/reachmap", points=centers, colors=colormap(index, 0.0, 1.0), point_size=ps,
    )

    server.gui.add_markdown(
        "**Reachability index** — fraction of\nsampled orientations with an IK solution\n\n"
        "🟣🔵🟢🟡 &nbsp; 0 → 1\n(position-only → fully dexterous)"
    )
    thr = server.gui.add_slider("min index", min=0.0, max=1.0, step=0.05, initial_value=0.0)
    size = server.gui.add_slider("point size", min=0.01, max=0.08, step=0.005, initial_value=ps)

    @thr.on_update
    def _(_) -> None:
        m = index >= thr.value
        if not m.any():
            return
        cloud.points = centers[m]
        cloud.colors = colormap(index[m], 0.0, 1.0)

    @size.on_update
    def _(_) -> None:
        cloud.point_size = size.value

    add_joint_sliders(server, viser_urdf)

    print(f"viser ready -> http://localhost:{args.port}   (Ctrl-C to quit)")
    while True:
        time.sleep(1.0)


if __name__ == "__main__":
    main()
