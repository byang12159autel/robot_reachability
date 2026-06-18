#!/usr/bin/env python3
"""Visualize the Stage-D rate-feasibility map (velocity / acceleration capacity).

Voxel cloud colored by EE velocity capacity (m/s) or acceleration capacity
(m/s²) — the best-posture, guaranteed-any-direction rate at each location. A
"min capacity" slider (fraction of max) filters to the high-rate region; robot
overlay has joint sliders.

Usage:
    pixi run view-rate
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

METRICS = {
    "velocity": ("vel_cap", "Velocity capacity (m/s)", "joint vel limits, capped at 3 m/s"),
    "acceleration": ("acc_cap", "Acceleration capacity (m/s²)", "joint accel 10 rad/s², capped at 9 m/s²"),
}


def legend_text(name, sub, lo, hi):
    return (f"**{name}**\n{sub}\n\n🟣🔵🟢🟡 &nbsp; `{lo:.3g}` → `{hi:.3g}`\n\n"
            "(dark = near-singular / low rate)")


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("robot", nargs="?", default="franka_fr3v2", choices=sorted(ROBOTS))
    ap.add_argument("--port", type=int, default=8080)
    args = ap.parse_args()

    npz = RESULTS / f"{args.robot}_rate.npz"
    if not npz.exists():
        raise SystemExit(f"missing {npz}; run: pixi run rate-fr3")
    d = np.load(npz)
    centers, res = d["centers"], float(d["res"])
    arrays = {"velocity": d["vel_cap"], "acceleration": d["acc_cap"]}
    print(f"{args.robot}: {len(centers):,} voxels @ {res*100:.1f}cm")

    server = viser.ViserServer(host="0.0.0.0", port=args.port)
    server.scene.add_grid("/ground", width=2.0, height=2.0, cell_size=0.1)
    urdf = yourdfpy.URDF.load(str(ROBOTS[args.robot]))
    viser_urdf = ViserUrdf(server, urdf, root_node_name="/robot")

    ps = res * 0.9
    cloud = server.scene.add_point_cloud("/rate", points=centers,
                                         colors=np.zeros((len(centers), 3), np.uint8),
                                         point_size=ps)

    legend = server.gui.add_markdown("")
    metric = server.gui.add_dropdown("capacity", tuple(METRICS), initial_value="velocity")
    thr = server.gui.add_slider("min capacity (frac of max)", min=0.0, max=1.0, step=0.05,
                                initial_value=0.0)
    size = server.gui.add_slider("point size", min=0.01, max=0.08, step=0.005, initial_value=ps)

    def refresh() -> None:
        vals = arrays[metric.value]
        lo, hi = np.percentile(vals, [2, 98])
        m = vals >= thr.value * vals.max()
        if not m.any():
            return
        cloud.points = centers[m]
        cloud.colors = colormap(vals[m], lo, hi)
        _, name, sub = METRICS[metric.value]
        legend.content = legend_text(name, sub, lo, hi)

    metric.on_update(lambda _: refresh())
    thr.on_update(lambda _: refresh())
    size.on_update(lambda _: setattr(cloud, "point_size", size.value))
    refresh()

    add_joint_sliders(server, viser_urdf)
    print(f"viser ready -> http://localhost:{args.port}   (Ctrl-C to quit)")
    while True:
        time.sleep(1.0)


if __name__ == "__main__":
    main()
