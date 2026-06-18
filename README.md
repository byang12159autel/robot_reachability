# Roadmap
Unreachable: no valid IK solution
Reachable but poor: IK exists, but margins or manipulability are low
Kinematically feasible: continuous IK exists within position limits
Rate feasible: velocity and acceleration limits are respected
Trackable: closed-loop error remains below tolerance

# Robots

Each robot is **vendored** as a standalone URDF with its meshes, under
`robots/<name>/`. Mesh paths are rewritten to be relative to the URDF, so there
is no ROS / `package://` runtime dependency.

| robot           | DOF | source repo                                          | ref             | xacro entry · args |
|-----------------|-----|------------------------------------------------------|-----------------|--------------------|
| `ur5`           | 6   | `UniversalRobots/Universal_Robots_ROS2_Description`  | default         | `urdf/ur.urdf.xacro` · `name:=ur5 ur_type:=ur5` |
| `flexiv_rizon4` | 7   | `flexivrobotics/flexiv_description`                  | `humble-v1.9.1` | `urdf/rizon.urdf.xacro` · `rizon_type:=Rizon4 load_gripper:=false` |
| `franka_fr3v2`  | 7   | `frankarobotics/franka_description`                  | default         | `robots/fr3v2/fr3v2.urdf.xacro` · `hand:=false ros2_control:=false gazebo:=false` |

Notes:
- **Flexiv Rizon4** comes from the `humble-v1` line, **not** `main`. The current
  `main` branch was restructured around a model called *EnlightL*, which is a
  **different product** from the Rizon4 — the Rizon configs/meshes only exist on
  the `v1` releases.
- **Franka FR3v2** is generated as the bare 7-DOF arm to the flange
  (`hand:=false`). The default `ee_id=franka_hand` references a mesh folder that
  doesn't exist upstream (only `franka_hand_white` / `_black` do), so the hand is
  omitted for now. It can be added back later if a gripper is needed.

## Environment

Managed with [pixi](https://pixi.sh) (`pixi.toml`). Channels: `conda-forge` +
`robostack-staging` (the latter only for `xacro`, used to flatten the upstream
descriptions).

```bash
pixi install
```

## Commands

| command | what it does |
|---|---|
| `pixi install` | create the environment |
| `pixi run import-robots` | re-flatten upstream xacro → `robots/<name>/<name>.urdf` |
| `pixi run verify` | headless load check (parse + on-disk meshes + viser) |
| `pixi run view <robot>` | interactive viser viewer at `http://localhost:8080` |
| `pixi run view <robot> --collision` | same, showing collision meshes |

`<robot>` is one of `ur5`, `flexiv_rizon4`, `franka_fr3v2`.

## Reachability analysis (Franka FR3v2)

Staged workspace analysis built on Pinocchio (FK / Jacobians / IK / dynamics).
See `plan.md` for method, results, and caveats. Current scope: joint-limit +
self-collision (no environment collision yet).

| stage | command | output | viewer |
|---|---|---|---|
| A+B — FK envelope + manipulability (self-collision pruned) | `pixi run reach-fr3-sc` | `results/franka_fr3v2_reach.npz` | `pixi run view-reach` |
| C — IK reachability index (dexterous workspace) | `pixi run reachmap-fr3` | `results/franka_fr3v2_reachmap.npz` | `pixi run view-reachmap` |
| D — rate feasibility (velocity / acceleration capacity) | `pixi run rate-fr3` | `results/franka_fr3v2_rate.npz` | `pixi run view-rate` |

(`reach-fr3` is the Stage-A run without self-collision pruning.) Reachability
viewers have joint sliders, a colormap legend, and filter sliders.

## Regenerate the URDFs

Upstream sources are cloned into `third_party/`. To re-flatten everything:

```bash
pixi run import-robots
```

This runs `xacro` (resolving `$(find <pkg>)` against a throwaway ament shim that
points at `third_party/`), copies only the referenced meshes, and rewrites mesh
paths to be relative. See `scripts/import_robots.py`.

## Visualize / verify

```bash
pixi run verify                  # headless: parse + mesh-resolution + viser load check
pixi run view ur5                # interactive viewer with per-joint sliders (port 8080)
pixi run view franka_fr3v2
pixi run view flexiv_rizon4 --collision
```

`pixi run verify` loads each URDF three ways (yourdfpy parse, on-disk mesh
existence, and `viser.extras.ViserUrdf`) and exits non-zero on any failure.

## Layout

```
robots/<name>/<name>.urdf      # flat, self-contained URDF (+ .srdf for franka)
robots/<name>/meshes/...       # visual (.dae/.obj) + collision (.stl)
scripts/import_robots.py       # xacro -> flat URDF/SRDF pipeline
reachability/                  # analysis (Pinocchio): FK / IK / collision / dynamics
  robots.py                    #   registry: URDF/SRDF, EE frame, rate limits
  fk_reachability.py           #   Stage A/B: envelope + manipulability (--collision)
  ik.py                        #   numerical IK (damped least-squares, multi-restart)
  reach_map.py                 #   Stage C: IK reachability-index map
  rate_map.py                  #   Stage D: velocity / acceleration capacity map
  collision.py                 #   self-collision checker (coal + SRDF)
visualization/                 # viser viewers (robot + reach / reachmap / rate)
  robots.py  view_robot.py  verify_load.py
  view_reachability.py  view_reachmap.py  view_rate.py
results/                       # *.npz analysis outputs
third_party/                   # cloned upstream descriptions (regeneration source)
plan.md                        # reachability method, staged plan, results
```
