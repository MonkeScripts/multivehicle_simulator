# Third-party licenses & attribution

`multivehicle_simulator` is licensed under **Apache-2.0** (see `LICENSE`). That
grant covers **only the first-party code and models authored by MonkeScripts /
BumblebeeAS**.

This repository also bundles and references third-party 3D models, meshes,
textures, and code that remain under **their own** licenses — they are **not**
relicensed under Apache-2.0. They are listed here with their origin and license
so the attribution obligations are met. Where a license is permissive, the only
obligation is to keep this credit.


## Bundled vehicle models

### BlueROV2 — `models/bluerov2/`
- **Origin:** [clydemcqueen/bluerov2_gz](https://github.com/clydemcqueen/bluerov2_gz) (Clyde McQueen). Meshes are Blue Robotics designs obtained via GrabCAD ([BlueROV2](https://grabcad.com/library/bluerov2-1), [T200 thruster](https://grabcad.com/library/bluerobotics-t200-thruster-1)); see `models/bluerov2/BlueROV2.md`.
- **License:** none stated upstream.
- **Status:** retained with attribution as community practice; not covered by this repo's Apache-2.0 grant. If you intend to commercialize, obtain explicit permission from the author first.

### BlueBoat — `models/blueboat/`
- **Origin:** Rhys Mainwaring; ArduPilot `ardupilot_gazebo` model lineage. Meshes from Blue Robotics CAD (`cad.bluerobotics.com`).
- **License:** **LGPL-3.0** (inherited from `ardupilot_gazebo`).
- **Status:** retained under LGPL-3.0 — **not** relicensed as Apache-2.0.

## Bundled scenery / textures

### Pier — `models/pier/`
- **Origin:** Open Robotics / Nate Koenig, Gazebo Fuel "Pier" model.
- **License:** Creative Commons Attribution 4.0 (CC-BY-4.0) — confirm on the Fuel listing.
- **Status:** retained with attribution. (The redundant original-download archive `Pier.zip` has been removed; only the extracted model is kept.)

### Sea-floor terrain texture — `models/robotx26/sea_floor/materials/`
- **Origin:** standard OSRF Gazebo media ("dirt" — `dirt_diffusespecular.png`, `dirt_tiled.material`).
- **License:** OSRF Gazebo media terms (permissive/CC — confirm).
- **Status:** retained with attribution.

### Docking-task dock & LED meshes — `models/robotx26/docking_task/`
- **Origin:** `.mtl` headers indicate a Bunkspeed export; likely RobotX / OSRF VRX dock lineage.
- **License:** not confirmed. If sourced from OSRF VRX (`vrx`), that is Apache-2.0 + attribution.
- **Status:** treated as in-house unless/until traced otherwise; revisit if commercializing.

## Acknowledgments (courtesy credits — first-party code, no license obligation)

### PX4 offboard demo — `src/PX4OffboardDemo.cpp`, `src/PX4OffboardDemo.hpp`
- **License:** Apache-2.0 (first-party — this repository).
- **Note:** this node was **rewritten as an independent implementation** for this project; it is not a copy of any upstream source. It uses the standard PX4 offboard pattern (a continuous `OffboardControlMode` + `TrajectorySetpoint` stream over uXRCE-DDS, with mode/arming via `VehicleCommand`), which follows PX4's BSD-3 ROS 2 examples and is also demonstrated in the [Dronecode ROSCon-25 workshop](https://github.com/Dronecode/roscon-25-workshop) (CC-BY-SA-4.0). Both are credited here as a courtesy / acknowledgment — the file carries **no** ShareAlike or attribution obligation.

## Referenced at runtime via Gazebo Fuel (fetched, NOT redistributed)

The world fetches these from Fuel at runtime; each carries its own per-listing
license. Not redistributed in this repo.

| Fuel model | Owner |
|------------|-------|
| `Sand Heightmap` | `hmoyen` |
| `Coast Water` | `OpenRobotics` (OSRF) |
| `Singapore River (Cropped out for Robot X 2026)` | `monkescripts` (this project's own Fuel listing — ensure it has a clear license and that you hold rights to the underlying terrain data) |

## Build / runtime dependencies (separate packages, own licenses)

Pulled by `multivehicle_simulator.repos` / built in the Docker image — each under
its own license, not part of this repository's licensed work:

| Dependency | License |
|------------|---------|
| `px4_msgs` | BSD-3-Clause |
| `Micro-XRCE-DDS-Agent` | Apache-2.0 |
| `ardupilot_gazebo` | LGPL-3.0 |
| ArduPilot / ArduSub (built in image) | GPL-3.0 |
| PX4-Autopilot (built in image) | BSD-3-Clause |
| `gz_led_plugin` (`BumblebeeAS/bb_led_plugin`), `bb_robotx_dashboard`, `bb_robotx_msgs` | BumblebeeAS / MonkeScripts |

## First-party (covered by this repo's Apache-2.0)

Authored by MonkeScripts / BumblebeeAS: all `launch/`, `scripts/`, `src/`,
`config/`, `mavros_params/`, `worlds/robotx_2026_sg_river.world`, and the in-house
`models/robotx24/tins/*` and procedural `models/robotx26/*` models
(`floating_dock_simple`, `safe_passage_buoy`, `incident_cube`, `docking_led`,
`pipe_led`, `pipe_task`, `sea_floor` wrapper, `sg_river_course` wrapper).
