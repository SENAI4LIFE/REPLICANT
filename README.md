# Tiffany — Hexapod Simulation (Gazebo / ROS 2)
Simulation of **Tiffany**, a 3-DOF-per-leg hexapod robot.
Hardware repo: https://github.com/Penguin-Lab/tiffany

---

## Prerequisites
- Ubuntu 24.04
- ROS 2 Jazzy — https://docs.ros.org/en/jazzy/Installation.html
- Gazebo Sim 8 (ships with `ros-jazzy-ros-gz`)
- GPU recommended (Gazebo rendering + camera sensor)

## Setup
Clone the repo:
```bash
git clone https://github.com/SENAI4LIFE/tiffany_gazebo.git && cd tiffany_gazebo
```
Add the ROS 2 apt repository (skip if already configured):
```bash
sudo apt install -y curl && sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null && sudo apt update
```
Install ROS/Gazebo dependencies:
```bash
sudo apt install -y ros-jazzy-desktop ros-jazzy-ros-gz ros-jazzy-gz-ros2-control ros-jazzy-ros2-control ros-jazzy-ros2-controllers ros-jazzy-robot-state-publisher ros-jazzy-xacro ros-jazzy-rviz2 ros-jazzy-slam-toolbox ros-jazzy-navigation2 python3-colcon-common-extensions python3-pyqt5 python3-catkin-pkg python3-lark python3-empy python3-jinja2 python3-yaml python3-typeguard
```
Build the workspace:
```bash
source /opt/ros/jazzy/setup.bash && chmod +x src/hexapod_ws/scripts/*.py && colcon build --symlink-install
```

---

## Run

**Terminal 1 — Simulation**
```bash
source setup.bash && ros2 launch hexapod_ws main.launch.py
```
`nav2:=true` only if `~/map.yaml` already exists, else defaults to live SLAM mapping. `world:=living_room|obstacle_arena|small_house` (default `living_room`). `rviz:=false` to skip RViz.

**Terminal 2 — Control** (pick one, each needs `source setup.bash` first)

Keyboard:
```bash
source setup.bash && ros2 run hexapod_ws teleop_hexapod.py
```
Virtual joystick:
```bash
source setup.bash && ros2 run hexapod_ws joystick_hexapod.py
```
Warning: Do not use the virtual joystick when Nav2 is enabled.

Nav2 destination picker (only when Nav2 is enabled):
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

### World options
- `world:=living_room` (default) — bundled assets, offline.
- `world:=obstacle_arena` — walled arena for gait testing.
- `world:=small_house` — clone once, then `world:=small_house`:
  ```bash
  git clone -b ros2 https://github.com/aws-robotics/aws-robomaker-small-house-world.git ~/aws-robomaker-small-house-world
  ```

### Autonomous Navigation (Nav2)
1. First time: leave `nav2` unset or pass `nav2:=false`, map the space, then save it (see Save the map above) to `~/map.yaml`.
2. From then on, plain `main.launch.py` auto-detects `~/map.yaml` and starts in Nav2 mode. Pass `map:=/other/path.yaml` for a different one.
3. `source setup.bash && ros2 run hexapod_ws nav_goal_gui.py` → **Boot** → click to send a goal (drag to set heading). **Cancel Goal** aborts.

Local costmap uses `/scan` and `/camera/points`.

### Continuing a mapping session
SLAM starts a fresh pose-graph every launch. Save before stopping (use the real home path, not `~`):
```bash
source setup.bash && ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph "{filename: '/home/you/map_slam'}"
```
Next launch auto-resumes `~/map_slam.posegraph`. Pass `continue_mapping:=/other/base/path` for a different graph.

### Comparing against a saved map
`~/map.yaml` is auto-overlaid in RViz as `SavedMap` (behind the live `Map`), for visual comparison only. Pass `saved_map:=/other/path.yaml` for a different one.

### Lidar
360 samples over 360°. Raw topic `/scan_bridge` has a mismatched frame ID; use `/scan` (relayed) instead.

### Camera
RGBD, 640x480 @ 15Hz, bridged on `/camera/image`, `/camera/depth_image`, `/camera/points`, `/camera/camera_info`.

`/camera/points` is shown directly in RViz as `CameraPointCloud`, with a 5s decay time so recently seen points persist briefly instead of vanishing every frame.

---

## Controls

**Keyboard** (`teleop_hexapod.py`)

| Key | Action |
|-----|--------|
| `E` / `Q` | Boot / Shutdown |
| `W`/`S`, `A`/`D` | Walk fwd/back, rotate L/R |
| `↑↓←→` | Same as WASD (outside pose mode) |
| `Z` | Toggle pose mode |
| `R` | Rebolar |
| `B` | Balance mode |
| `P` | Patinha (toggle) |
| `C` / `X` | Turn / Omni nav mode |
| `SPACE` | Stop |

**Joystick** (`joystick_hexapod.py`): drag to move, release to stop. Buttons for Boot, Shutdown, Nav mode, Pose mode, Safe Mode, Stop. A status panel at the top shows Robot state, locomotion mode, control source (Manual/Nav2), Safe Mode, and Nav2 status. While the joystick is actively driving, autonomous locomotion mode changes (from Nav2 or obstacle-recovery behaviors) are held off — the selected mode stays in effect until you release the stick. Safe Mode is off by default; when enabled it uses the LiDAR to limit or block forward movement toward nearby obstacles, without switching locomotion mode or taking control away from you.

**Nav2 picker** (`nav_goal_gui.py`): Boot, then click/drag on the map to send a goal + heading. Cancel Goal aborts.

---

## Useful ROS 2 commands

List active topics:
```bash
ros2 topic list
```

Remove the build
```bash
rm -rf build install log
```