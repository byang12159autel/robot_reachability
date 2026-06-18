# Reachability Analysis — Plan

First robot under study: **Franka FR3v2** — 7-DOF, arm-only, end-effector frame
`fr3v2_link8` (flange).

## Do I need IK?

| What you want | Need IK? | Method |
|---|---|---|
| Reachable position envelope | **No** | FK Monte Carlo — exact, no false negatives |
| Manipulability / conditioning map (Yoshikawa index) | **No** | FK + **Jacobian** at each sampled config, aggregate per voxel |
| Dexterous workspace (reachable in *all* orientations) / reachability index | **Practically yes** | Voxel × orientation grid, IK per pose |

Forward sampling can technically estimate orientation coverage by binning poses
in 6D, but it can't *prove the absence* of orientation gaps (empty bin =
unreachable **or** undersampled), and SO(3) coverage is sample-hungry — so the
dexterous workspace is the one place IK pays off. A numerical IK solver has the
mirror problem (false negatives near singularities); the FR3 semi-analytical IK
(redundancy resolved by a free elbow/`q7` parameter) removes that.

## Staged approach

- **Stage A — FK envelope (no IK).** Monte-Carlo sample joint configs within URDF
  limits → FK to flange → reachable point cloud. Metrics: reach radius, bbox,
  z-range, voxel-occupancy volume.
- **Stage B — Manipulability map (no IK).** At each sampled config compute the
  frame Jacobian → Yoshikawa manipulability (translational + full 6D). A local
  dexterity proxy; feeds the "reachable but poor" tier.
- **Stage C — Reachability index (IK).** Voxelize the workspace, sample
  orientations per voxel, solve IK → fraction reachable per voxel = reachability
  index → the rigorous dexterous workspace.
- **Stage D — Rate feasibility (no IK).** Per configuration, joint velocity and
  acceleration limits bound the EE velocity (ẋ = J·q̇) and acceleration (ẍ = J·q̈,
  at q̇=0), each clamped to the robot's Cartesian caps. Map the best-posture,
  guaranteed-any-direction velocity (m/s) and acceleration (m/s²) capacity per voxel.
  FR3 limits from the spec sheet (not in URDF): q̈_max = 10 rad/s², caps 3 m/s / 9 m/s².

## Scope / caveats

- **Self-collision** is available across Stages A–D via `--collision` (Pinocchio
  + coal + the Franka SRDF allowed-collision matrix). **Environment collision**
  (table / mount) is not modeled yet, so the set remains an upper bound.
- EE frame = flange `fr3v2_link8` (no hand/TCP; FR3 was vendored arm-only).
- The full 6D manipulability mixes linear (m) and angular (rad) units;
  translational manipulability (det of the 3×n position Jacobian) is the cleaner
  Cartesian-reach measure. Both are recorded.

## Roadmap tiers (from README)

Unreachable → Reachable but poor → Kinematically feasible → Rate feasible →
Trackable. Stage A/B cover *kinematically feasible* and start *reachable but
poor*; Stage C completes orientation reachability; Stage D covers *rate feasible*.
*Trackable* (closed-loop tracking) is not yet implemented.

## Status

- [x] Vendored URDFs (ur5, flexiv_rizon4, franka_fr3v2) + viser load verification
- [x] Stage A — FK envelope (FR3): horizontal reach 0.858 m, z ∈ [-0.35, 1.19] m,
      voxel volume ≈ 2.63 m³ @ 5 cm (300k samples) — `pixi run reach-fr3`
- [x] Stage B — Manipulability map (FR3): translational median 0.068, max 0.163;
      full-6D median 0.033 — saved with the cloud, viewable via `pixi run view-reach`
- [x] Self-collision pruning (FR3): 2.8% pruned (8 active pairs); min reach
      0.009→0.047 m, envelope volume ~unchanged (boundary is collision-free) —
      `pixi run reach-fr3-sc`
- [x] Stage C — IK reachability index (FR3): numerical IK (Pinocchio DLS, 99%
      round-trip), **self-collision-checked**, 6,750 voxels @ 7.5cm × 24 orientations.
      mean index 0.64; dexterous core (≥0.99) 0.37 m³, ≥0.9 0.94 m³, ≥0.5 1.98 m³.
      Self-collision trims the dexterous core ~5% vs joint-limit-only (0.39→0.37 m³).
      — `pixi run reachmap-fr3` (collision-checked), view `pixi run view-reachmap`
- [x] Stage D — Rate feasibility (FR3): velocity cap median 0.67 m/s (max 0.86),
      acceleration cap median 2.05 m/s² (max 2.69; documented q̈_max=10 rad/s²,
      Cartesian-capped 3 m/s / 9 m/s²), 20,987 voxels @ 5cm
      — `pixi run rate-fr3`, view `pixi run view-rate`
