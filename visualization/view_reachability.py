#!/usr/bin/env python3
"""Visualize a Stage-A/B reachability result in viser.

Renders the reachable flange point cloud (from reachability/fk_reachability.py),
colored by manipulability, with the robot overlaid at a mid-range pose. If the
result includes a self-collision mask, colliding configs are shown separately in
grey and can be toggled. A GUI legend explains the colors.

Usage:
    pixi run view-reach
    python visualization/view_reachability.py franka_fr3v2 --metric trans
"""
from __future__ import annotations

import argparse
import time
from pathlib import Path

import numpy as np
import viser
import yourdfpy
from viser.extras import ViserUrdf

from robots import ROBOTS  # visualization/robots.py: name -> urdf path

ROOT = Path(__file__).resolve().parent.parent
RESULTS = ROOT / "results"

# viridis control points (value -> RGB in [0,1])
_VIRIDIS = np.array([
    [0.267, 0.005, 0.329],
    [0.231, 0.318, 0.545],
    [0.128, 0.566, 0.551],
    [0.369, 0.789, 0.383],
    [0.993, 0.906, 0.144],
])

_METRIC_LABEL = {
    "trans": "translational manipulability  √det(Jᵥ·Jᵥᵀ)",
    "full": "full-6D manipulability  √det(J·Jᵀ)",
}


def _pctile(values):
    return np.percentile(values, [2, 98])


def colormap(values, lo, hi) -> np.ndarray:
    """Map values -> uint8 RGB via viridis, normalized to [lo, hi]."""
    t = np.clip((values - lo) / (hi - lo + 1e-12), 0.0, 1.0)
    xs = np.linspace(0.0, 1.0, len(_VIRIDIS))
    rgb = np.stack([np.interp(t, xs, _VIRIDIS[:, c]) for c in range(3)], axis=1)
    return (rgb * 255).astype(np.uint8)


def legend_md(metric: str, lo: float, hi: float, n_pruned: int) -> str:
    lines = [
        f"**Color — {_METRIC_LABEL[metric]}**",
        "",
        "🟣🔵🟢🟡 &nbsp; low → high",
        "(near-singular → dexterous)",
        "",
        f"scale (2–98 pctile): `{lo:.3f}` → `{hi:.3f}`",
    ]
    if n_pruned:
        lines += ["", f"⚪ grey = self-colliding ({n_pruned:,} pts, pruned)"]
    return "\n".join(lines)


def add_joint_sliders(server, viser_urdf):
    """One slider per actuated joint (grouped in a folder); robot starts mid-range."""
    sliders, initial = [], []
    with server.gui.add_folder("Joint control"):
        for name, (lo, hi) in viser_urdf.get_actuated_joint_limits().items():
            lo = -np.pi if lo is None else float(lo)
            hi = np.pi if hi is None else float(hi)
            start = (lo + hi) / 2.0
            sliders.append(server.gui.add_slider(name, min=lo, max=hi, step=1e-3,
                                                 initial_value=start))
            initial.append(start)
        reset = server.gui.add_button("reset to mid-range")

    initial = np.array(initial)

    def update(_=None) -> None:
        viser_urdf.update_cfg(np.array([s.value for s in sliders]))

    for s in sliders:
        s.on_update(update)

    @reset.on_click
    def _(_) -> None:
        for s, v in zip(sliders, initial):
            s.value = float(v)

    viser_urdf.update_cfg(initial)


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("robot", nargs="?", default="franka_fr3v2", choices=sorted(ROBOTS))
    ap.add_argument("--metric", choices=["trans", "full"], default="trans")
    ap.add_argument("--max-points", type=int, default=200_000)
    ap.add_argument("--point-size", type=float, default=0.004)
    ap.add_argument("--port", type=int, default=8080)
    args = ap.parse_args()

    npz = RESULTS / f"{args.robot}_reach.npz"
    if not npz.exists():
        raise SystemExit(f"missing {npz}; run: pixi run reach-fr3")
    data = np.load(npz)
    points = data["points"]
    metrics = {"trans": data["w_trans"], "full": data["w_full"]}
    free = data["collision_free"] if "collision_free" in data.files else np.ones(len(points), bool)

    # Downsample for a responsive browser scene.
    if len(points) > args.max_points:
        idx = np.random.default_rng(0).choice(len(points), args.max_points, replace=False)
        points, free = points[idx], free[idx]
        metrics = {k: v[idx] for k, v in metrics.items()}

    n_pruned = int((~free).sum())
    print(f"{args.robot}: {free.sum():,} collision-free + {n_pruned:,} pruned, "
          f"coloring by '{args.metric}'")

    server = viser.ViserServer(host="0.0.0.0", port=args.port)
    server.scene.add_grid("/ground", width=2.0, height=2.0, cell_size=0.1)

    # Robot at the mid-range of its joint limits (valid for FR3's asymmetric limits).
    urdf = yourdfpy.URDF.load(str(ROBOTS[args.robot]))
    viser_urdf = ViserUrdf(server, urdf, root_node_name="/robot")

    free_pts, free_w = points[free], {k: v[free] for k, v in metrics.items()}
    lo, hi = _pctile(free_w[args.metric])
    reach_cloud = server.scene.add_point_cloud(
        "/reachable", points=free_pts, colors=colormap(free_w[args.metric], lo, hi),
        point_size=args.point_size,
    )

    pruned_cloud = None
    if n_pruned:
        pruned_cloud = server.scene.add_point_cloud(
            "/self_collision", points=points[~free],
            colors=np.full((n_pruned, 3), 150, np.uint8), point_size=args.point_size,
        )
        pruned_cloud.visible = False

    # --- GUI ---
    legend = server.gui.add_markdown(legend_md(args.metric, lo, hi, n_pruned))
    metric_gui = server.gui.add_dropdown("manipulability", ("trans", "full"),
                                         initial_value=args.metric)
    size_gui = server.gui.add_slider("point size", min=0.001, max=0.02, step=0.001,
                                     initial_value=args.point_size)
    show_pruned = (server.gui.add_checkbox("show self-colliding", False) if n_pruned else None)
    add_joint_sliders(server, viser_urdf)

    @metric_gui.on_update
    def _(_) -> None:
        m = metric_gui.value
        lo2, hi2 = _pctile(free_w[m])
        reach_cloud.colors = colormap(free_w[m], lo2, hi2)
        legend.content = legend_md(m, lo2, hi2, n_pruned)

    @size_gui.on_update
    def _(_) -> None:
        reach_cloud.point_size = size_gui.value
        if pruned_cloud is not None:
            pruned_cloud.point_size = size_gui.value

    if show_pruned is not None:
        @show_pruned.on_update
        def _(_) -> None:
            pruned_cloud.visible = show_pruned.value

    print(f"viser ready -> http://localhost:{args.port}   (Ctrl-C to quit)")
    while True:
        time.sleep(1.0)


if __name__ == "__main__":
    main()
