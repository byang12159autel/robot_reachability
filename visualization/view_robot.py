#!/usr/bin/env python3
"""Interactive viser viewer for a vendored robot URDF.

Loads the URDF, renders it in a browser-based viser scene, and exposes one GUI
slider per actuated joint (clamped to the URDF joint limits).

Run with:
    pixi run view                 # defaults to ur5
    pixi run view franka_fr3v2
    pixi run view flexiv_rizon4 --collision --port 8080
"""
from __future__ import annotations

import argparse
import time

import numpy as np
import viser
import yourdfpy
from viser.extras import ViserUrdf

from robots import ROBOTS


def add_joint_sliders(server: viser.ViserServer, viser_urdf: ViserUrdf):
    """Create a slider per actuated joint; returns (sliders, initial config)."""
    sliders: list[viser.GuiInputHandle[float]] = []
    initial: list[float] = []

    for name, (lower, upper) in viser_urdf.get_actuated_joint_limits().items():
        lower = -np.pi if lower is None else float(lower)
        upper = np.pi if upper is None else float(upper)
        start = 0.0 if lower < 0.0 < upper else (lower + upper) / 2.0
        slider = server.gui.add_slider(
            name, min=lower, max=upper, step=1e-3, initial_value=start
        )
        sliders.append(slider)
        initial.append(start)

    def update(_=None) -> None:
        viser_urdf.update_cfg(np.array([s.value for s in sliders]))

    for slider in sliders:
        slider.on_update(update)

    return sliders, np.array(initial)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "robot", nargs="?", default="ur5", choices=sorted(ROBOTS),
        help="which vendored robot to view",
    )
    parser.add_argument("--port", type=int, default=8080)
    parser.add_argument(
        "--collision", action="store_true",
        help="render collision meshes instead of/with visual meshes",
    )
    args = parser.parse_args()

    path = ROBOTS[args.robot]
    print(f"Loading {args.robot}: {path}")
    urdf = yourdfpy.URDF.load(str(path))

    server = viser.ViserServer(host="0.0.0.0", port=args.port)
    server.scene.add_grid("/ground", width=2.0, height=2.0, cell_size=0.1)
    viser_urdf = ViserUrdf(
        server,
        urdf,
        load_meshes=not args.collision,
        load_collision_meshes=args.collision,
    )

    sliders, initial = add_joint_sliders(server, viser_urdf)
    viser_urdf.update_cfg(initial)

    reset = server.gui.add_button("Reset to default")

    @reset.on_click
    def _(_) -> None:
        for slider, value in zip(sliders, initial):
            slider.value = float(value)

    print(f"viser ready -> http://localhost:{args.port}   (Ctrl-C to quit)")
    while True:
        time.sleep(1.0)


if __name__ == "__main__":
    main()
