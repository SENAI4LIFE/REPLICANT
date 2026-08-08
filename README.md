# Tiffany — Hexapod Simulation (Gazebo / ROS 2)

Simulation and autonomous-navigation stack for **Tiffany**, a 3-DOF-per-leg (coxa/femur/tibia) hexapod robot, built on **ROS 2 Jazzy** and **Gazebo Sim 8 (Harmonic)**.
Hardware repo: https://github.com/Penguin-Lab/tiffany

[![ROS2](https://img.shields.io/badge/ROS2-Jazzy-22314E?logo=ros)](#)
[![Gazebo](https://img.shields.io/badge/Gazebo-Harmonic-orange)](#)
[![Nav2](https://img.shields.io/badge/Nav2-Jazzy-00599C)](#)
[![SLAM Toolbox](https://img.shields.io/badge/SLAM-Toolbox-green)](#)
[![Ubuntu](https://img.shields.io/badge/Ubuntu-24.04-E95420?logo=ubuntu)](#)
[![Python](https://img.shields.io/badge/Python-3-yellow?logo=python)](#)

<!-- Gallery placeholder — screenshots/GIFs of Tiffany walking, RViz, and Gazebo go here -->
<p align="center">
  <img src="images/tiffany_hero.gif" width="80%" alt="Tiffany hexapod demo (placeholder)"/>
</p>

---

## Table of Contents

- [Overview](#overview)
- [Objectives](#objectives)
- [Requirements](#requirements)
- [Features](#features)
- [Gallery](#gallery)
- [Video Demonstrations](#video-demonstrations)
- [Quick Start](#quick-start)
- [Installation](#installation)
- [Build](#build)
- [Usage](#usage)
- [Controls](#controls)
- [Worlds](#worlds)
- [Sensors](#sensors)
- [RViz](#rviz)
- [SLAM (Mapping)](#slam-mapping)
- [Autonomous Navigation (Nav2)](#autonomous-navigation-nav2)
- [Localization](#localization)
- [System Architecture](#system-architecture)
- [Hexapod-Specific Nav2 Adaptations](#hexapod-specific-nav2-adaptations)
- [Gait and Kinematics](#gait-and-kinematics)
- [Project Structure](#project-structure)
- [Configuration Reference](#configuration-reference)
- [Detailed Implementation](#detailed-implementation)
- [Design Decisions](#design-decisions)
- [Performance Considerations](#performance-considerations)
- [Troubleshooting and Known Limitations](#troubleshooting-and-known-limitations)
- [Useful ROS 2 Commands](#useful-ros-2-commands)

---

## Overview

This repository simulates Tiffany, a 6-legged (hexapod) robot, in Gazebo and drives it through a full ROS 2 navigation stack: real-time gait generation and inverse kinematics, SLAM Toolbox mapping, Nav2 path planning/following, and AMCL-style localization against a saved map. Unlike a wheeled robot, Tiffany's "drive" layer is a **custom gait engine** (`hexapod_runner.py`) that turns a `/cmd_vel` twist into 18 leg-joint position commands, so the rest of the stack (Nav2, SLAM, teleop) talks to it exactly as it would to a differential or omni-wheeled base.

### What the robot can do

- Walk, strafe, and turn using body-frame Bezier-curve leg trajectories.
- Boot/shutdown through a smooth stand-up/sit-down sequence.
- Balance its body against terrain tilt using IMU feedback.
- Perform two idle animations (**Rebolar** — a hip-sway wiggle, **Patinha** — a "paw" gesture).
- Be driven manually (keyboard or virtual joystick) or autonomously (Nav2).
- Build a map live with SLAM Toolbox, or navigate a previously saved map.
- Self-protect: a costmap-based failsafe and lidar-based clearance guard slow it down or steer it out of high-cost/near-collision situations independent of Nav2.

## Objectives

- Provide a realistic Gazebo Harmonic simulation of a legged robot driven through the standard ROS 2 Nav2/SLAM stack.
- Translate wheeled-robot navigation concepts (`/cmd_vel`, occupancy costmaps, AMCL-style localization) into a hexapod gait engine without modifying Nav2 itself.
- Support both first-time environment mapping and repeat autonomous navigation on a saved map.
- Offer multiple manual control surfaces (keyboard, virtual joystick, point-and-click Nav2 goals) for development and testing.
- Add hexapod-appropriate safety behaviors (safe mode, costmap failsafe, obstacle strafing) that a purely wheeled Nav2 setup wouldn't need.

## Requirements

### Software

- Ubuntu 24.04
- ROS 2 Jazzy — https://docs.ros.org/en/jazzy/Installation.html
- Gazebo Sim 8 (ships with `ros-jazzy-ros-gz`)
- Python 3 with `numpy`, `PyYAML`, `PyQt5` (installed as ROS package dependencies below)

### Recommended hardware

- GPU recommended for Gazebo's rendering and the camera/LiDAR sensors. Not required if launching with `camera:=false` (LiDAR-only workloads are lighter).
- Different worlds have different computational cost: the bundled `obstacle_arena` is lightweight (primitive shapes only), `living_room` (the default) adds furniture meshes, and `small_house` is the heaviest of the three — its larger floor plan, furnished rooms, and additional simulated objects increase rendering, sensor, and physics load, so a more capable GPU/system is recommended when using it. No specific hardware numbers are prescribed; scale expectations to the world you choose.

## Features

| Feature | Description |
|---|---|
| **Custom gait engine** | Bezier-curve leg trajectories with per-leg forward/inverse kinematics, ticking at 50 Hz (`hexapod_runner.py`) |
| **Four navigation modes** | `Omni 1`, `Omni 2`, `Turn 1`, `Turn 2` — different mappings from `/cmd_vel` to gait direction (see [below](#hexapod-specific-nav2-adaptations)) |
| **SLAM Toolbox mapping** | Asynchronous, pose-graph-based live mapping, resumable across sessions |
| **Nav2 autonomous navigation** | Full stack (planner, DWB-based controller via rotation-shim, behavior server, BT navigator, waypoint follower) driving the gait engine through `/cmd_vel` |
| **Point-and-click goal GUI** | `nav_goal_gui.py` — PyQt5 map view with live costmap, plan, robot pose, and one-click goal + heading |
| **Keyboard & virtual joystick teleop** | Full manual control, independent of Nav2 |
| **Costmap failsafe** | Autonomously steers off dangerously costed cells even outside of active Nav2 goals |
| **Lateral clearance / corridor centering** | Biases heading to stay centered in corridors and away from close walls while Nav2-driven |
| **Stall / stuck detection** | Detects lack of progress and performs a sideways strafe escape |
| **Safe Mode** | Lidar-based speed limiting/blocking toward nearby obstacles during manual driving |
| **RGBD camera + 360° LiDAR** | Both independently toggleable; camera point cloud feeds the local costmap alongside the LiDAR |
| **Multiple Gazebo worlds** | A bundled offline living room (default), a bundled obstacle arena for gait testing, and support for the AWS RoboMaker Small House world |

## Gallery

<!-- Add screenshots here, e.g.: -->
<!--
<p align="center">
  <img src="images/gazebo_view.png" width="45%"/>
  <img src="images/rviz_view.png" width="45%"/>
</p>
-->
*(Images to be added.)*

## Video Demonstrations

<!-- Add demo video links/embeds here once recorded. -->
*(Video walkthroughs of SLAM mapping and Nav2 navigation to be added.)*

---

## Quick Start

Clone the repository:

```bash
git clone https://github.com/SENAI4LIFE/tiffany_gazebo.git && cd tiffany_gazebo
```

Install ROS 2 Jazzy, Gazebo Harmonic, and dependencies (see [Installation](#installation) for the repo-setup step):

```bash
sudo apt install -y ros-jazzy-desktop ros-jazzy-ros-gz ros-jazzy-gz-ros2-control ros-jazzy-ros2-control \
  ros-jazzy-ros2-controllers ros-jazzy-robot-state-publisher ros-jazzy-xacro ros-jazzy-rviz2 \
  ros-jazzy-slam-toolbox ros-jazzy-navigation2 python3-colcon-common-extensions python3-pyqt5 \
  python3-catkin-pkg python3-lark python3-empy python3-jinja2 python3-yaml python3-typeguard
```

Build the workspace:

```bash
source /opt/ros/jazzy/setup.bash && chmod +x src/hexapod_ws/scripts/*.py && colcon build --symlink-install
```

Run the simulation (first launch defaults to live SLAM mapping in the `living_room` world):

```bash
source setup.bash && ros2 launch hexapod_ws main.launch.py
```

In another terminal, boot and drive the robot:

```bash
source setup.bash && ros2 run hexapod_ws teleop_hexapod.py
```

See [Installation](#installation) and [Usage](#usage) for full details, and [Worlds](#worlds) for the `small_house` world's one-time setup.

---

## Build

```bash
source /opt/ros/jazzy/setup.bash
chmod +x src/hexapod_ws/scripts/*.py
colcon build --symlink-install
```

---

## Usage

**Terminal 1 — Simulation**
```bash
source setup.bash && ros2 launch hexapod_ws main.launch.py
```
`world:=living_room|obstacle_arena|small_house` (default `living_room` — see [Worlds](#worlds) for `small_house`'s one-time setup). `nav2:=true` only if a saved map (`~/map.yaml` by default) exists, otherwise the launch defaults to live SLAM mapping. `rviz:=false` to skip RViz. `camera:=false` / `lidar:=false` to disable those sensors (both default `true`).

**Terminal 2 — Control** (pick one, each needs `source setup.bash` first)

Keyboard:
```bash
source setup.bash && ros2 run hexapod_ws teleop_hexapod.py
```
Virtual joystick:
```bash
source setup.bash && ros2 run hexapod_ws joystick_hexapod.py
```
> Do not use the virtual joystick while Nav2 is actively navigating — driving manually cancels the active Nav2 goal (see [System Architecture](#system-architecture)).

Nav2 destination picker (only meaningful when Nav2 is enabled):
```bash
source setup.bash && ros2 run hexapod_ws nav_goal_gui.py
```

**RViz standalone** — if you launched with `rviz:=false`:
```bash
source setup.bash && ros2 run rviz2 rviz2 -d src/hexapod_ws/rviz/hexapod.rviz --ros-args -p use_sim_time:=true
```

**Save the map** — writes `map.pgm`/`map.yaml` to the given path:
```bash
source setup.bash && ros2 run nav2_map_server map_saver_cli -f ~/map
```

### Basic controls

Boot the robot before doing anything else (`E` on the keyboard, **Boot** on the joystick GUI or `nav_goal_gui.py`), drive it with `WASD`/the joystick pad, and `Q` / **Shutdown** to sit back down. See [Controls](#controls) for the full reference.

---

## Controls

### Keyboard (`teleop_hexapod.py`)

| Key | Action |
|---|---|
| `E` / `Q` | Boot / Shutdown |
| `W`/`S`, `A`/`D` | Walk forward/back, rotate left/right |
| `↑↓←→` | Same as WASD (outside pose mode) |
| `Z` | Toggle pose mode (tilt body with the same keys instead of walking) |
| `R` | Rebolar (wiggle animation) |
| `B` | Balance mode (auto-levels body against IMU roll/pitch) |
| `P` | Patinha (paw animation, toggle) |
| `C` / `X` | Nav mode: `C` → Turn 1, `X` → Omni 2 |
| `SPACE` | Stop |

### Virtual joystick (`joystick_hexapod.py`)

Drag the pad to move, release to stop. Buttons for Boot, Shutdown, Pose mode, and Safe Mode; a row of buttons switches between all four nav modes (Omni 1 / Omni 2 / Turn 1 / Turn 2); a speed slider scales the joystick's output; Rebolar and Patinha buttons trigger the animations.

A status row shows robot state, confirmed nav mode (echoed back from the robot's own state feedback), control source (Manual while the pad is held, Nav2 otherwise), Safe Mode, and Nav2 status. While the joystick is actively driving, autonomous nav-mode changes from Nav2 or the obstacle-recovery behaviors are held off — the selected mode stays in effect until you release the stick. Safe Mode is off by default; when enabled it uses the LiDAR to limit or block forward/lateral movement toward nearby obstacles, without switching nav mode or taking control away from you.

### Nav2 goal picker (`nav_goal_gui.py`)

Boot the robot from the GUI, then click and drag on the map to send a goal with a heading (drag sets the direction the robot should end up facing). **Cancel Goal** aborts the active goal. The map view shows the live SLAM/Nav2 map, the saved-map overlay (if any), the local costmap-derived clearance color, the planned path, and the robot's current state — all colour-coded. See [System Architecture](#system-architecture) for what the GUI does automatically while a goal is active (lateral optimization, final-approach correction, stuck-goal recovery).

---

## Worlds

- `world:=living_room` (**default**) — bundled, offline, no extra setup. A single furnished room (sofa, coffee table, chairs, TV stand, lamp, bookshelf, rug, posters, seated visitor models) enclosed by walls.
- `world:=obstacle_arena` — bundled, offline. A walled arena scattered with boxes, cylinders, and spheres of varying sizes, intended for gait and obstacle-avoidance testing rather than realism.
- `world:=small_house` — the AWS RoboMaker Small House world; a full multi-room house. Requires the one-time clone below. Significantly heavier than the other two worlds — its larger floor plan, furnished rooms, and additional simulated objects increase rendering, sensor, and physics load, so a more capable GPU/system is recommended when using it.

One-time setup for `small_house`:
```bash
git clone -b ros2 https://github.com/aws-robotics/aws-robomaker-small-house-world.git ~/aws-robomaker-small-house-world
```
`setup.bash` automatically patches this cloned world the first time it's sourced (see [Design Decisions](#design-decisions)) — no manual edits needed after cloning it.

---

## Sensors

### LiDAR

720 samples over 360°, 15 Hz, range 0.10–12.0 m, mounted on `base_link`. The raw bridged topic `/scan_bridge` carries a frame ID that doesn't match ROS's TF tree; use `/scan` (relayed and re-framed by the `ScanRelay` node inside `hexapod_runner.py`) instead. That relay also drops scans while the robot's body tilt exceeds 15°, so a stumble or a stair-step in the gait doesn't feed a corrupted, tilted scan into SLAM or Nav2's costmaps.

Enabled by default. Disable it with `lidar:=false`:
```bash
source setup.bash && ros2 launch hexapod_ws main.launch.py lidar:=false
```
This removes the LiDAR sensor from the robot and skips its bridge topic. Camera, Gazebo, odometry, and the rest of the simulation are unaffected. **SLAM and Nav2 both require `/scan`, so neither is started while `lidar:=false`** — a message is logged explaining why.

### Camera

RGBD, 320×240 @ 15 Hz, bridged on `/camera/image`, `/camera/depth_image`, `/camera/points`, `/camera/camera_info`. `/camera/points` is shown directly in RViz as `CameraPointCloud` with a 5 s decay time, so recently seen points persist briefly instead of vanishing every frame; it's also fed into Nav2's local costmap as a second obstacle source alongside the LiDAR scan.

Enabled by default. Disable it with `camera:=false`:
```bash
source setup.bash && ros2 launch hexapod_ws main.launch.py camera:=false
```
This removes the camera sensor, skips its bridge topics, and stops point-cloud processing. LiDAR, SLAM, Nav2, and odometry are unaffected — the local costmap simply falls back to `/scan` only.

Combine flags as usual, e.g. mapping with no camera, or camera-only inspection with no navigation:
```bash
source setup.bash && ros2 launch hexapod_ws main.launch.py camera:=false nav2:=false
source setup.bash && ros2 launch hexapod_ws main.launch.py lidar:=false nav2:=false
```

### IMU

100 Hz, used for the tilt-based scan cutoff above, for `Balance` mode's auto-leveling, and by the `ScanRelay` node.

---

## RViz

The bundled config (`src/hexapod_ws/rviz/hexapod.rviz`) includes: the robot model and TF tree; the live `LaserScan`; camera `Image`, `DepthImage`, and `CameraPointCloud`; the live SLAM/Nav2 `Map` alongside a `SavedMap` overlay layer; `GlobalCostmap` and `LocalCostmap`; the Nav2 `GlobalPlan`; and a `Trajectory` path built from odometry (see [`trajectory_publisher.py`](#detailed-implementation)). It launches automatically unless `rviz:=false` is passed.

---

## SLAM (Mapping)

Launched automatically whenever `nav2` is not `true` (i.e. by default, until a saved map exists). SLAM Toolbox runs in asynchronous mapping mode against `/scan`, publishing `map -> odom`.

### Continuing a mapping session

SLAM starts a fresh pose-graph every launch by default. Save the graph before stopping (use a real absolute path, not `~`, since it's read back by another process):
```bash
source setup.bash && ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph "{filename: '/home/you/map_slam'}"
```

### Saving the final map

```bash
source setup.bash && ros2 run nav2_map_server map_saver_cli -f ~/map
```
This writes `map.pgm`/`map.yaml`, which is what `nav2:=true` (and the `SavedMap` RViz overlay) look for by default.

---

## Autonomous Navigation (Nav2)

1. **First time:** leave `nav2` unset (or pass `nav2:=false`) so the world launches in SLAM mode, drive the robot to map the space (see [SLAM](#slam-mapping)), then save the map to `~/map.yaml`.
2. **From then on:** a plain `ros2 launch hexapod_ws main.launch.py` auto-detects `~/map.yaml` and starts in Nav2 mode instead of SLAM. Pass `map:=/other/path.yaml` to localize against a different map.
3. Send goals with `ros2 run hexapod_ws nav_goal_gui.py` → **Boot** → click-and-drag on the map to set a destination and heading. **Cancel Goal** aborts.

The Nav2 stack itself (`controller_server`, `planner_server`, `behavior_server`, `bt_navigator`, `waypoint_follower`, `map_server`, `amcl`, all under a `lifecycle_manager`) is standard Nav2 — see [Hexapod-Specific Nav2 Adaptations](#hexapod-specific-nav2-adaptations) for what's layered on top to make it work with a legged gait instead of a wheeled base.

The local costmap fuses `/scan` and `/camera/points`. Both SLAM and Nav2 require `/scan`, so neither is started when `lidar:=false`.

---

## Localization

Localization while `nav2:=true` is handled by `nav2_amcl`, configured with `robot_model_type: nav2_amcl::OmniMotionModel` (matched to the robot's omnidirectional gait rather than a differential-drive model), an initial pose set from `config/nav2_params.yaml`, and standard AMCL particle-filter parameters (see [Configuration Reference](#configuration-reference)). While SLAM Toolbox is active (`nav2` not `true`), `map -> odom` instead comes from SLAM Toolbox's own scan matching.

---

## System Architecture

### Node graph and boot order

`main.launch.py` brings the system up in a strict dependency chain, each stage triggered by the previous process starting or exiting:

1. **Gazebo** (server + GUI) loads the selected world; `robot_state_publisher` publishes `robot_description` from the xacro model.
2. `ros_gz_sim create` spawns the robot from `robot_description`; the `ros_gz_bridge` nodes (base, camera, LiDAR) bridge Gazebo topics to ROS.
3. Once spawned, `joint_state_broadcaster` is spawned, then `hexapod_controller` (a `forward_command_controller/ForwardCommandController`) once the broadcaster is active.
4. Once the controller is active, **`hexapod_runner.py`** (the "brain") starts — this is the node that turns `/cmd_vel` into leg joint commands.
5. Once the brain starts: SLAM Toolbox (unless `nav2:=true`), Nav2's nodes (if `nav2:=true`), `trajectory_publisher.py`, and the Gazebo GUI are all launched a few seconds later, giving the brain time to initialize.

### `/cmd_vel` and ros2_control

`hexapod_runner.py` subscribes to the standard `/cmd_vel` (`geometry_msgs/Twist`) — the same topic Nav2's `controller_server` and the teleop scripts publish to — and converts it every 20 ms into an 18-element `Float64MultiArray` of joint positions (`coxa`/`femur`/`tibia` × 6 legs) on `/hexapod_controller/commands`. `ros2_control`'s `hexapod_controller` (`ForwardCommandController`, position interface) forwards those positions to the `gz_ros2_control` plugin, which drives the corresponding joints in Gazebo. This is what lets the rest of the stack (Nav2, teleop, SLAM) treat Tiffany like any `/cmd_vel`-driven base without knowing it's a hexapod.

### TF and odometry

Gazebo's `OdometryPublisher` plugin publishes odometry and TF for the spawned model under a `tiffany/`-prefixed namespace (`/model/tiffany/odometry`, and TF on `/model/tiffany/tf` and `/tf_static`, bridged to ROS as `/odom`, `/tf_raw`, `/tf_static_raw`). A small **`TFRemapper`** node (also inside `hexapod_runner.py`'s process) strips that `tiffany/` prefix and republishes onto the standard `/tf` and `/tf_static`, so the rest of the ROS 2 ecosystem (RViz, Nav2, SLAM Toolbox) sees the conventional `map -> odom -> base_footprint -> base_link -> ...` tree without any per-package configuration. `/odom` also feeds `trajectory_publisher.py`, which builds an RViz-visualizable `/trajectory` path (points sampled at ≥2 cm of movement, capped at 20000 poses).

### Gazebo integration

Sensors (IMU, LiDAR, RGBD camera) are declared directly in the xacro's `<gazebo>` blocks and simulated by Gazebo's native sensor plugins, then bridged to ROS via `ros_gz_bridge` using `config/bridge*.yaml`. Joint actuation goes through `gz_ros2_control`'s `GazeboSimSystem` hardware plugin rather than a bespoke Gazebo plugin, so the same `ros2_control` interfaces used on real hardware are exercised in simulation.

### Topics summary

| Topic | Type | Purpose |
|---|---|---|
| `/cmd_vel` | `geometry_msgs/Twist` | Velocity command in (from teleop, joystick, or Nav2) |
| `/tiffany/state` | `std_msgs/String` | Discrete commands: `BOOT`, `SHUTDOWN`, `IDLE`, `BALANCE`, `REBOLAR`, `PATINHA`, `POSE <roll> <pitch>`, `NAV_OMNI_1`/`NAV_OMNI_2`/`NAV_TURN_1`/`NAV_TURN_2`, `MANUAL_ON`/`OFF`, `SAFE_ON`/`OFF`, `LATERAL_OPT_ON`/`OFF` |
| `/tiffany/state_feedback` | `std_msgs/String` (JSON) | Robot state, nav mode, gait speed, tilt, clearance, strafe/failsafe status, etc. — consumed by the joystick and Nav2 goal GUIs |
| `/tiffany/nav2_status` | `std_msgs/String` | High-level Nav2 goal status (`idle`, `sending`, `navigating`, `succeeded`, `canceled`, ...) published by `nav_goal_gui.py` |
| `/scan` | `sensor_msgs/LaserScan` | Re-framed, tilt-filtered LiDAR scan (use this, not `/scan_bridge`) |
| `/odom` | `nav_msgs/Odometry` | Gazebo-simulated odometry |
| `/trajectory` | `nav_msgs/Path` | Traveled path, built from `/odom`, for RViz |
| `/camera/*` | `sensor_msgs/Image`, `PointCloud2`, `CameraInfo` | RGBD camera outputs |
| `/hexapod_controller/commands` | `std_msgs/Float64MultiArray` | 18 joint position targets sent to `ros2_control` |

---

## Hexapod-Specific Nav2 Adaptations

Nav2 itself is unmodified — everything below is a translation layer inside `hexapod_runner.py` and `nav_goal_gui.py` that lets a legged gait stand in for the wheeled/differential base Nav2 expects.

### Navigation (gait) modes

Four modes decide how a `/cmd_vel` twist maps onto a walking direction and gait state. They're switched with `/tiffany/state` messages (`NAV_OMNI_1`, `NAV_OMNI_2`, `NAV_TURN_1`, `NAV_TURN_2`) and reflected back on `/tiffany/state_feedback`:

- **Omni 1** (default, and what Nav2 drives in) — axis-locked omnidirectional: whichever of `linear.x`/`linear.y` is larger is used alone (no diagonal walking), with `angular.z` alone triggering an in-place turn when there's no translation. This keeps Nav2-commanded motion predictable and easy for the DWB-style controller to reason about.
- **Omni 2** — fully unrestricted omnidirectional: the walk direction is `atan2(-linear.y, -linear.x)` for any combination of `linear.x`/`linear.y`, allowing diagonal strafing. Used by the keyboard's `X` and the joystick's default mode.
- **Turn 1** — discrete, car-like: forward/backward motion only walks at heading 0°/180°, and `angular.z` alone turns in place; each leg's trajectory is blended between a straight step and an arc-shaped step depending on how "turny" the current command is. Keyboard's `C`.
- **Turn 2** — continuous car-like steering: the walk heading is `atan2(angular.z, -linear.x)`, so combined forward+turn commands produce a smoothly curving walk rather than a stop-turn-go motion; joystick/Nav2-style continuous commands only (not exposed on the keyboard).

### Costmap failsafe

Independent of whether Nav2 has an active goal, `hexapod_runner.py` continuously checks the local costmap cell under the robot. If that cell's cost exceeds a risk threshold, it declares a failsafe state and walks the robot toward the nearest low-cost cell found by an expanding ring search around the robot, overriding whatever `/cmd_vel` is currently commanding, until enough consecutive ticks confirm the robot is back on safe ground.

### Lateral clearance optimization / corridor centering

While Nav2 has an active goal (and only then — not during manual driving, the costmap failsafe, or the final-approach strafe below), `nav_goal_gui.py` enables `LATERAL_OPT`, which biases the walking heading a few degrees toward whichever side (from LiDAR sector scans, and secondarily from costmap sampling) has more clearance — nudging the robot to walk centered through corridors and doorways instead of hugging one side.

### Stall detection and strafe escape

`hexapod_runner.py` tracks XY and yaw progress over time; if the robot isn't making minimum progress for several consecutive checks (or is detected as forward-blocked for many ticks in a row), it triggers a timed sideways strafe maneuver to break out of the stuck situation — direction chosen toward whichever side has more measured clearance, with hysteresis to avoid flip-flopping and a cooldown before it can trigger again.

### Nav2 goal GUI behaviors

On top of the standard `NavigateToPose` action client, `nav_goal_gui.py` adds:
- **Final approach correction** — near the goal, if the robot is spinning in place without closing the remaining distance, it briefly takes over `/cmd_vel` directly with a slow strafe to nudge into position, rather than relying purely on Nav2's own goal checker.
- **Stuck-goal recovery** — if navigation stalls, it can issue a `BackUp` action and retry toward a nearby costmap-verified safe point before giving up on a goal.
- **Automatic goal cancellation** — if `MANUAL_ON` feedback arrives (i.e. the joystick pad is pressed) while a goal is active, the GUI cancels the goal so manual control isn't fighting Nav2.

### Safe Mode

A manual-driving-only safety net (`SAFE_ON`/`SAFE_OFF`, toggled from the joystick GUI): scales or zeroes commanded linear velocity as the robot approaches an obstacle in its direction of travel, using the same LiDAR-derived clearance distances as the costmap failsafe's thresholds. It never changes nav mode or state on its own — it only limits speed.

---

## Gait and Kinematics

Each leg is a 3-DOF serial chain (coxa → femur → tibia) with link lengths `L1 = 0.0256 m`, `L2 = 0.0900 m`, `L3 = 0.1216 m`. `fk()`/`ik()` implement forward and inverse kinematics for a single leg from those lengths; every gait function ultimately produces a target foot XYZ per leg, which `ik()` converts to the three joint angles sent to `ros2_control`.

Walking and turning trajectories are built from cubic Bezier swing curves (`build_bezier_points`, `trajetoria_linear`) sampled over a 25-point cycle (`TOTAL_PONTOS`) with smoothstep easing, and legs are grouped into two opposing tripods via a fixed per-leg phase offset (`OFFSETS`) — a standard alternating tripod gait. `compute_turn_1`/`compute_turn_2` additionally blend each leg's straight-line step with a circular (in-place-rotation) step (`mapeia_circular`) by a weight that depends on how much of the current command is "turning" vs. "walking". `compute_ik_corpo` handles whole-body roll/pitch/yaw posing (used by `Balance` and `Pose` mode) by rotating each leg's shoulder-relative foot position. `Rebolar` and `Patinha` are pre-scripted animations built on top of the same body-posing and per-leg-trajectory primitives.

Boot and shutdown are smooth 50-step interpolations between a stowed/folded pose and the standing/idle pose (raising or lowering the whole body in two phases — first horizontal positioning, then vertical), rather than an instantaneous joint snap.

---

## Project Structure

```
tiffany_gazebo/
├── setup.bash                     # Sources ROS + workspace, sets GZ resource paths and patches small_house
├── stop.py                        # Kills all simulation/ROS processes started by this project
├── import.py / export.py / clear.py   # Maintainer git helper scripts (pull / commit+push / force-reset history)
├── src/
│   └── hexapod_ws/                 # The ROS 2 package (ament_cmake)
│       ├── CMakeLists.txt
│       ├── package.xml
│       ├── description/
│       │   ├── hexapod.urdf.xacro  # Robot model: links, joints, ros2_control, sensors, Gazebo plugins
│       │   └── meshes/
│       ├── launch/
│       │   └── main.launch.py      # Single entry point for the whole stack
│       ├── config/
│       │   ├── parameters.yaml     # ros2_control controllers
│       │   ├── bridge.yaml / bridge_camera.yaml / bridge_lidar.yaml   # ros_gz_bridge topic maps
│       │   ├── slam_params.yaml    # SLAM Toolbox parameters
│       │   ├── nav2_params.yaml    # AMCL, controller, planner, behavior, costmap parameters
│       │   └── gz_gui_top_down.config  # Gazebo GUI layout
│       ├── worlds/
│       │   ├── living_room.sdf
│       │   └── obstacle_arena.sdf
│       ├── models/                 # Furniture/prop meshes used by living_room.sdf
│       ├── rviz/
│       │   └── hexapod.rviz
│       └── scripts/
│           ├── hexapod_runner.py       # Gait engine + TF remapper + scan relay ("the brain")
│           ├── teleop_hexapod.py       # Keyboard teleop
│           ├── joystick_hexapod.py     # PyQt5 virtual joystick
│           ├── nav_goal_gui.py         # PyQt5 Nav2 goal picker / map viewer
│           └── trajectory_publisher.py # Builds /trajectory from /odom
└── README.md
```

## Configuration Reference

| File | Governs |
|---|---|
| `config/parameters.yaml` | `controller_manager` update rate; `joint_state_broadcaster` and `hexapod_controller` (position-interface `ForwardCommandController` over all 18 leg joints) |
| `config/bridge.yaml` | Base bridge: `/clock`, `/imu`, `/odom` (from `/model/tiffany/odometry`), and raw TF (`/tf_raw`, `/tf_static_raw`) |
| `config/bridge_camera.yaml` | Camera bridge: image, depth image, point cloud, camera info |
| `config/bridge_lidar.yaml` | LiDAR bridge: `/scan_bridge` (raw, mismatched frame ID — see [Sensors](#sensors)) |
| `config/slam_params.yaml` | SLAM Toolbox: async mapping mode, Ceres solver settings, scan-matching/loop-closure thresholds |
| `config/nav2_params.yaml` | `amcl` (omni motion model), `controller_server` (`RotationShimController` wrapping `DWBLocalPlanner`, tuned to the robot's ~0.15 m/s gait speed), `planner_server` (NavFn/A*), `behavior_server` (spin/backup/wait), `bt_navigator`, `waypoint_follower`, and both global/local costmaps (LiDAR + point-cloud obstacle layers, inflation) |
| `description/hexapod.urdf.xacro` | Robot links/joints, `ros2_control` joint interfaces, IMU/LiDAR/camera sensor definitions, `gz_ros2_control` and `OdometryPublisher` Gazebo plugins |


## Detailed Implementation

### `hexapod_runner.py` — the brain

The core of the project. Runs three ROS nodes in one process under a `MultiThreadedExecutor`:

- **`HexapodRunner`** — the state machine and gait engine. Owns the robot's discrete state (`POWERED_OFF`, `IDLE`, `WALKING`, `TURNING`, `BALANCE`, `POSE`, `PATINHA`, `REBOLAR`) and nav mode, ticks at 50 Hz (`_step`), and layers the failsafe/clearance/stall logic described in [Hexapod-Specific Nav2 Adaptations](#hexapod-specific-nav2-adaptations) on top of the base gait functions.
- **`TFRemapper`** — strips the `tiffany/` frame-ID prefix Gazebo's odometry plugin adds, republishing standard-named TF.
- **`ScanRelay`** — re-frames `/scan_bridge` onto `/scan` and drops scans while the robot's IMU-measured tilt exceeds 15°.

Key methods worth knowing when modifying behavior: `_cmd_vel_cb` (twist → target gait state/angle, per nav mode), `_state_cb` (handles all `/tiffany/state` string commands), `_step` (the main per-tick state machine, including gait-speed ramping and heading smoothing), `_apply_costmap_failsafe`/`_check_and_apply_strafe_escape`/`_apply_clearance_guard` (the three independent safety layers), and `_publish_state_feedback` (the JSON status consumed by both GUIs).

### `nav_goal_gui.py`

A PyQt5 map viewer (`MapCanvas`) plus a `NavGoalNode` that wraps Nav2's `NavigateToPose` action client with the hexapod-specific behaviors in [Hexapod-Specific Nav2 Adaptations](#hexapod-specific-nav2-adaptations): automatic `LATERAL_OPT` toggling, final-approach strafe correction, and backup-and-retry recovery. Also mirrors `/tiffany/state_feedback` into the on-screen state badge and colors the map by local-costmap clearance around the robot.

### `joystick_hexapod.py`

A `JoystickPad` widget (drag-to-vector, normalized to ±1) driving a `JoystickNode` that republishes the last commanded velocity on a timer (so releasing the mouse mid-drag doesn't leave a stale command latched), plus buttons for every discrete `/tiffany/state` command and all four nav modes.

### `teleop_hexapod.py`

A minimal raw-terminal keyboard reader; every non-pose keypress publishes a one-shot velocity or state command, and the loop republishes the last velocity ~50 times/sec while a movement key is held (`select`-based, non-blocking read).

### `trajectory_publisher.py`

Subscribes `/odom`, appends a pose to a `nav_msgs/Path` whenever the robot has moved ≥2 cm since the last sample (to keep the path lightweight), caps the path at 20000 poses, and republishes `/trajectory` for RViz.

---

## Design Decisions

- **A `/cmd_vel`-in, joints-out gait engine, instead of a custom Nav2 controller plugin.** This keeps Nav2, SLAM Toolbox, and every teleop tool completely standard — they all just publish/consume `/cmd_vel`, `/scan`, `/odom`, and TF as they would for any mobile base, and `hexapod_runner.py` is the only place that knows the robot has legs.
- **`Omni 1` (axis-locked) is the mode Nav2 drives in by default**, not the fully free `Omni 2` — the DWB-style controller's trajectory scoring assumes independent, decoupled linear/angular control, and axis-locking avoids diagonal foot-placement ambiguity that would otherwise fight the controller's assumptions.
- **TF remapping instead of configuring every downstream tool for `tiffany/`-prefixed frames.** Gazebo's `OdometryPublisher` plugin namespaces frames by model name (useful for multi-robot sims); rather than reconfigure Nav2, SLAM Toolbox, and RViz to expect that prefix, a small always-on `TFRemapper` node normalizes it once.
- **A tilt cutoff on the LiDAR relay, independent of Nav2/SLAM.** A hexapod's gait naturally pitches/rolls the body far more than a wheeled base; feeding SLAM or the costmaps a scan taken while the body is tipped would corrupt the map, so the relay simply drops those scans rather than trying to compensate for them downstream.
- **A standalone costmap failsafe, separate from Nav2's own recovery behaviors.** It runs at all times once the robot is booted — including during manual driving and idle periods — not just during an active Nav2 goal, since a legged robot can end up standing on a bad cell for reasons Nav2 never sees (e.g. manual teleop).
- **`living_room` as the default world**, with `obstacle_arena` also bundled and offline, and `small_house` supported but not shipped in the repo — this keeps a first-time launch fast and dependency-free (no extra clone required), while still offering a lighter arena for gait testing and a heavier, more realistic multi-room house for those who want it.
- **Auto-patching the AWS Small House world at `setup.bash` source time** rather than shipping a modified copy of a large third-party asset: the upstream world predates Gazebo Harmonic and is missing the physics/sensors/IMU/contact/user-commands/scene-broadcaster/air-pressure system plugins Harmonic needs, and one of its models has a malformed inertia tag. `setup.bash` detects and patches both, idempotently, only if the world is present and not already patched.

## Performance Considerations

- Gazebo's rendering, sensor simulation (camera + LiDAR), and physics load scale with world complexity. `obstacle_arena` (primitive shapes) is the lightest, `living_room` (the default, one furnished room) is moderate, and `small_house` (a full multi-room house) is the heaviest — it may need a more capable GPU/system to keep the simulation running smoothly.
- Disabling `camera:=false` and/or `lidar:=false` reduces sensor simulation and bridging overhead when those sensors aren't needed (e.g. inspecting gait behavior in RViz without navigation).
- No specific hardware minimums are prescribed here; expect the requirement to scale with your chosen world and which sensors are enabled.

## Troubleshooting and Known Limitations

- **`nav2:=true` but nothing loads / map_server has nothing to load** — this means no map was found at `~/map.yaml` (or the path passed via `map:=`). Launch with `nav2:=false` (or leave it unset) to map the space first, save it, then relaunch. The launch file logs an explicit warning when this happens.
- **`lidar:=false` silently disables SLAM/Nav2** — both require `/scan`; a log message explains why neither started.
- **Virtual joystick + active Nav2 goal** — pressing the joystick pad automatically cancels the active Nav2 goal (by design, so manual and autonomous control never fight over `/cmd_vel`); it will not resume automatically.
- **`/scan_bridge` vs `/scan`** — always use `/scan`; the raw bridged topic has a frame ID that doesn't match the TF tree and isn't tilt-filtered.
- **First-time `small_house` launch** — make sure the world has been cloned to `~/aws-robomaker-small-house-world` (see [Worlds](#worlds)) before launching with `world:=small_house`; `setup.bash` patches it automatically once present, but doesn't clone it for you.
- **Continuing a mapping session** — SLAM Toolbox does not resume automatically; you must explicitly serialize the pose-graph before stopping (see [SLAM](#slam-mapping)) for the next launch to pick it up.

## Useful ROS 2 Commands

### Monitoring

```bash
ros2 node list
ros2 topic list
ros2 topic hz /scan
ros2 topic echo /cmd_vel
ros2 topic echo /tiffany/state_feedback
ros2 run tf2_ros tf2_echo map base_link
ros2 run tf2_tools view_frames
```

### Debugging

```bash
ros2 node info /hexapod_runner
ros2 param list /controller_server
ros2 bag record -a -o navigation_data
```

### Clean Build

Remove the build artifacts:

```bash
rm -rf build install log
```