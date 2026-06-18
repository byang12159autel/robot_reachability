#!/usr/bin/env python3
"""Verify every vendored URDF loads cleanly.

Three independent checks per robot:
  1. yourdfpy parse        -> kinematic tree (links / joints / DOF)
  2. mesh resolution       -> every `filename=` resolves to a file on disk
  3. viser ViserUrdf load  -> loads into a viser scene (meshes -> trimesh)

Exit code is non-zero if any robot fails any check.

Run with:  pixi run verify
"""
from __future__ import annotations

import re
import sys
from pathlib import Path

import yourdfpy

from robots import ROBOTS

MESH_RE = re.compile(r'filename="([^"]+)"')


def missing_meshes(urdf_path: Path) -> tuple[int, list[str]]:
    base = urdf_path.parent
    refs = MESH_RE.findall(urdf_path.read_text())
    missing = [r for r in refs if not (base / r).exists()]
    return len(refs), missing


def main() -> None:
    import viser
    from viser.extras import ViserUrdf

    server = viser.ViserServer(host="127.0.0.1", port=8092)
    all_ok = True

    header = f"{'robot':<16}{'links':>6}{'joints':>7}{'DOF':>5}{'meshes':>8}   status"
    print(header)
    print("-" * len(header))

    for name, path in ROBOTS.items():
        try:
            urdf = yourdfpy.URDF.load(str(path))
            n_links = len(urdf.link_map)
            n_joints = len(urdf.joint_map)
            dof = len(urdf.actuated_joints)

            n_meshes, missing = missing_meshes(path)

            # Load into viser; this builds trimesh geometry for every visual mesh.
            viser_urdf = ViserUrdf(server, urdf, root_node_name=f"/{name}")
            n_actuated = len(viser_urdf.get_actuated_joint_limits())

            status = "OK"
            if missing:
                status = f"FAIL: {len(missing)} missing mesh(es) e.g. {missing[:2]}"
                all_ok = False
            elif n_actuated != dof:
                status = f"WARN: viser actuated={n_actuated} != urdf DOF={dof}"

            print(f"{name:<16}{n_links:>6}{n_joints:>7}{dof:>5}{n_meshes:>8}   {status}")
        except Exception as exc:  # noqa: BLE001 - report and continue
            all_ok = False
            print(f"{name:<16}{'':>6}{'':>7}{'':>5}{'':>8}   FAIL: {exc!r}")

    server.stop()
    print()
    if all_ok:
        print("✅ All URDFs load cleanly (structure + meshes + viser).")
        sys.exit(0)
    print("❌ One or more URDFs failed to load.")
    sys.exit(1)


if __name__ == "__main__":
    main()
