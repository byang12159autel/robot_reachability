"""Robot registry for reachability analysis: URDF path + end-effector frame."""
from __future__ import annotations

from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent

ROBOTS: dict[str, dict] = {
    "franka_fr3v2": {
        "urdf": ROOT / "robots" / "franka_fr3v2" / "franka_fr3v2.urdf",
        "srdf": ROOT / "robots" / "franka_fr3v2" / "franka_fr3v2.srdf",
        "ee_frame": "fr3v2_link8",  # flange (arm vendored without hand)
        # Documented limits (not in URDF): frankarobotics.github.io/docs/robot_specifications.html
        "rate_limits": {
            "qd_max": [2.62, 2.62, 2.62, 2.62, 5.26, 4.18, 5.26],  # rad/s
            "qdd_max": [10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0],  # rad/s^2
            "cart_v_max": 3.0,   # m/s   (Cartesian translational velocity cap)
            "cart_a_max": 9.0,   # m/s^2 (Cartesian translational acceleration cap)
        },
    },
    "ur5": {
        "urdf": ROOT / "robots" / "ur5" / "ur5.urdf",
        "ee_frame": "tool0",
    },
    "flexiv_rizon4": {
        "urdf": ROOT / "robots" / "flexiv_rizon4" / "flexiv_rizon4.urdf",
        "ee_frame": "flange",
    },
}

RESULTS = ROOT / "results"
