"""Shared registry of vendored robot URDFs."""
from __future__ import annotations

from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent

ROBOTS: dict[str, Path] = {
    "ur5": ROOT / "robots" / "ur5" / "ur5.urdf",
    "flexiv_rizon4": ROOT / "robots" / "flexiv_rizon4" / "flexiv_rizon4.urdf",
    "franka_fr3v2": ROOT / "robots" / "franka_fr3v2" / "franka_fr3v2.urdf",
}
