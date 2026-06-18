#!/usr/bin/env python3
"""Flatten the vendored xacro robot descriptions into self-contained URDFs.

For each robot we:
  1. Run `xacro` on the upstream entry point (resolving `$(find <pkg>)` via a
     throw-away ament prefix that points at the cloned repos in third_party/).
  2. Copy only the meshes that robot references into robots/<name>/meshes/.
  3. Rewrite every mesh `filename=` to a path relative to the output URDF so the
     result is portable and has no ROS/`package://` dependency.

Run with:  pixi run python scripts/import_robots.py
"""
from __future__ import annotations

import os
import re
import shutil
import subprocess
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
THIRD_PARTY = ROOT / "third_party"
OUT = ROOT / "robots"
BUILD = ROOT / "build"

# package name (as used in $(find ...) / package://) -> cloned source directory
PKGS = {
    "ur_description": THIRD_PARTY / "Universal_Robots_ROS2_Description",
    "flexiv_description": THIRD_PARTY / "flexiv_description",
    "franka_description": THIRD_PARTY / "franka_description",
}

ROBOTS = [
    dict(
        name="ur5",
        pkg="ur_description",
        xacro="urdf/ur.urdf.xacro",
        args=["name:=ur5", "ur_type:=ur5"],
    ),
    dict(
        name="flexiv_rizon4",
        pkg="flexiv_description",
        xacro="urdf/rizon.urdf.xacro",
        args=["rizon_type:=Rizon4", "load_gripper:=false"],
    ),
    dict(
        name="franka_fr3v2",
        pkg="franka_description",
        xacro="robots/fr3v2/fr3v2.urdf.xacro",
        # 7-DOF arm to the flange; no gripper / ros2_control / gazebo plugins.
        args=["hand:=false", "ros2_control:=false", "gazebo:=false"],
        # SRDF -> allowed-collision pairs for self-collision checking (same prefix).
        srdf="robots/fr3v2/fr3v2.srdf.xacro",
        srdf_args=["hand:=false"],
    ),
]

MESH_RE = re.compile(r'filename="([^"]+)"')


def setup_ament() -> Path:
    """Build a minimal ament prefix so `$(find <pkg>)` resolves to our clones."""
    ament = BUILD / "ament"
    shutil.rmtree(ament, ignore_errors=True)
    markers = ament / "share" / "ament_index" / "resource_index" / "packages"
    markers.mkdir(parents=True)
    for pkg, src in PKGS.items():
        (markers / pkg).write_text("")
        (ament / "share" / pkg).symlink_to(src.resolve())
    return ament


def run_xacro(robot: dict, ament: Path) -> Path:
    src = PKGS[robot["pkg"]]
    raw = BUILD / f"{robot['name']}.raw.urdf"
    cmd = ["xacro", str(src / robot["xacro"]), *robot["args"], "-o", str(raw)]
    env = dict(os.environ)
    env["AMENT_PREFIX_PATH"] = f"{ament}:" + env.get("AMENT_PREFIX_PATH", "")
    print("  $", " ".join(cmd))
    subprocess.run(cmd, check=True, env=env)
    return raw


def generate_srdf(robot: dict, ament: Path) -> None:
    """Flatten the SRDF (allowed-collision matrix) into robots/<name>/<name>.srdf."""
    src = PKGS[robot["pkg"]]
    out = OUT / robot["name"] / f"{robot['name']}.srdf"
    cmd = ["xacro", str(src / robot["srdf"]), *robot.get("srdf_args", []), "-o", str(out)]
    env = dict(os.environ)
    env["AMENT_PREFIX_PATH"] = f"{ament}:" + env.get("AMENT_PREFIX_PATH", "")
    print("  $", " ".join(cmd))
    subprocess.run(cmd, check=True, env=env)
    print(f"  -> {out.relative_to(ROOT)}")


def map_filename(fn: str):
    """Map a mesh filename in the raw URDF to (source_dir, relative_path)."""
    if fn.startswith("package://"):
        pkgname, _, rest = fn[len("package://"):].partition("/")
        if pkgname in PKGS:
            return PKGS[pkgname].resolve(), rest
        return None
    p = Path(fn)
    if p.is_absolute():
        real = p.resolve()
        for src in PKGS.values():
            try:
                return src.resolve(), str(real.relative_to(src.resolve()))
            except ValueError:
                continue
    return None


def localize(robot: dict, raw: Path):
    name = robot["name"]
    dst_root = OUT / name
    if dst_root.exists():
        shutil.rmtree(dst_root)
    dst_root.mkdir(parents=True)

    text = raw.read_text()
    mapping: dict[str, str] = {}        # original filename -> relative path
    roots: set[tuple[Path, str]] = set()  # (source_dir, mesh-subtree to copy)
    unmapped: list[str] = []

    for fn in sorted(set(MESH_RE.findall(text))):
        m = map_filename(fn)
        if m is None:
            unmapped.append(fn)
            continue
        src_dir, rel = m
        mapping[fn] = rel
        parts = rel.split("/")
        if "visual" in parts:
            idx = parts.index("visual")
        elif "collision" in parts:
            idx = parts.index("collision")
        else:
            idx = len(parts) - 1
        roots.add((src_dir, "/".join(parts[:idx])))

    # Copy whole visual/collision subtrees (captures .mtl / textures alongside .obj).
    for src_dir, rel_root in sorted(roots, key=lambda t: str(t)):
        shutil.copytree(src_dir / rel_root, dst_root / rel_root, dirs_exist_ok=True)

    for fn, rel in mapping.items():
        text = text.replace(f'filename="{fn}"', f'filename="{rel}"')

    out_urdf = dst_root / f"{name}.urdf"
    out_urdf.write_text(text)
    return out_urdf, len(mapping), len(roots), unmapped


def main() -> None:
    BUILD.mkdir(exist_ok=True)
    OUT.mkdir(exist_ok=True)
    ament = setup_ament()
    for robot in ROBOTS:
        print(f"\n=== {robot['name']} ===")
        raw = run_xacro(robot, ament)
        out_urdf, n_meshes, n_roots, unmapped = localize(robot, raw)
        rel = out_urdf.relative_to(ROOT)
        print(f"  -> {rel}  ({n_meshes} mesh refs, {n_roots} subtree(s) copied)")
        for u in unmapped:
            print(f"  [WARN] unmapped mesh reference: {u}")
        if robot.get("srdf"):
            generate_srdf(robot, ament)
    print("\nDone.")


if __name__ == "__main__":
    main()
