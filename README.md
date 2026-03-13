# Project Description
This project is a simulation/workaround for a 3 degree of freedom (DOF) per leg hexapod robot named Tiffany. <br>
Robot is https://github.com/Penguin-Lab/tiffany.  <br>

# Modeling and Tools
It was firstly prototyped using Autodesk Fusion, then .step exported.<br>
![Fusion Model](./fusion.png)
## Figure 1 - Fusion Model
.step files were inserted, duplicated and mirrowed into a assembly, mate connectors joined parts and set degrees of freedom.
![Onshape Model](./onshape.png)
## Figure 2 - Onshape Model

# Exporting
onshape-to-robot was used to convert the onshape model assembly into .urdf, which is the description of the robot. <br>
https://github.com/Rhoban/onshape-to-robot <br>
https://onshape-to-robot.readthedocs.io/

If you want to use your own hexapod model, you should modify the keys and a config.json to generate your own assets. <br>
To get the API keys, follow the onshape-to-robot documentation <br>
keys file: <br>
```
export ONSHAPE_API=https://cad.onshape.com
export ONSHAPE_ACCESS_KEY=https://onshape-to-robot.readthedocs.io/en/latest/getting_started.html#using-bashrc
export ONSHAPE_SECRET_KEY=https://onshape-to-robot.readthedocs.io/en/latest/getting_started.html#using-bashrc
```
config.json file:
```
{        
    "documentId": "https://onshape-to-robot.readthedocs.io/en/latest/config.html",
    "outputFormat": "urdf",
    "assemblyName": "robot",
    "ignore": {
        "CORPO_CENTER": "collision"
    }
}

```

After setting up the files, get the tool:
```
pip install onshape-to-robot
```
then, source your keys path using:
```
source keys
```
```
onshape-to-robot pybullet (path of the folder with the config.json)
```
# (In Development)
Refining the kinematics inside pybullet before going through ros and gazebo implementation.

## Setting things up
### Cloning the Repository
```bash
git clone https://github.com/SENAI4LIFE/tiffany_sim.git
```

### Create virtual environment
```bash
python3 -m venv venv
```

### Activate environment
```bash
source ~/venv/bin/activate
```
Shortcut
```bash
nano ~/.bashrc
```

Paste at the end of bash:
```bash
alias ..='cd ..'
alias venv='source ~/venv/bin/activate'
alias tiffanypy='cd tiffany_sim/workspace/pybullet'
alias tiffanygz='cd tiffany_sim/workspace/gazebosim'
```

# (PyBullet) 
## Navigate to project directory
```bash
cd ~/tiffany_sim/workspace/pybullet
```

```bash
pip install --upgrade pip
pip install pybullet
pip install numpy
```

### Start Simulation
```bash
python3 main.py
```

# (Gazebosim)
## Prerequisites
- Ubuntu 24.04 (or WSL2 with Ubuntu 24.04)
- ROS 2 Jazzy
- Gazebo Sim 8 (Jazzy)

## Installing ROS 2 Jazzy
Follow the official instructions at https://docs.ros.org/en/jazzy/Installation.html

## Installing Dependencies
```bash
sudo apt install -y \
  ros-jazzy-ros-gz \
  ros-jazzy-gz-ros2-control \
  ros-jazzy-ros2-control \
  ros-jazzy-ros2-controllers \
  ros-jazzy-joint-state-broadcaster \
  ros-jazzy-forward-command-controller \
  ros-jazzy-robot-state-publisher \
  ros-jazzy-xacro \
  ros-jazzy-rviz2 \
  ros-jazzy-rqt-image-view \
  python3-colcon-common-extensions \
  python3-pyqt5
```

## Setting up the workspace
```bash
cd ~/tiffany_sim/workspace/gazebosim
```

Install Python build dependencies inside the venv:
```bash
pip install catkin_pkg lark empy jinja2 pyyaml typeguard
```

## Environment Variables
Add the following to your `~/.bashrc`:
```bash
source /opt/ros/jazzy/setup.bash
export GZ_SIM_RESOURCE_PATH=/opt/ros/jazzy/share:~/tiffany_sim/workspace/gazebosim/install/hexapod_ws/share
export LD_PRELOAD=/usr/lib/x86_64-linux-gnu/libpthread.so.0
```

Then reload:
```bash
source ~/.bashrc
```

## Building
```bash
cd ~/tiffany_sim/workspace/gazebosim
colcon build --symlink-install
source install/setup.bash
```

## Starting the Simulation
### Terminal 1 — Launch Gazebo + Controllers
```bash
source install/setup.bash
ros2 launch hexapod_ws main.launch.py
```
Wait ~15 seconds for all controllers to load.

### Terminal 2 — Teleop
```bash
source install/setup.bash
ros2 run hexapod_ws teleop_hexapod.py
```

## Teleop Controls
| Key | Action |
|-----|--------|
| `E` | Boot robot |
| `R` | Shutdown robot |
| `W` / `S` | Walk forward / backward |
| `A` / `D` | Rotate left / right |
| `Q` / `Z` | Strafe left / right |
| `B` | Rebolar |
| `N` | Balance mode |
| `G` / `P` | Patinha |
| Arrow keys | Body pose (roll/pitch) |
| `SPACE` | Stop |
| `C` | Navigation turn mode |
| `X` | Navigation omni mode |

## Viewing Sensor Data
GUI tools must be run **outside the venv** (`deactivate` first).

### Camera Feed
```bash
ros2 run rqt_image_view rqt_image_view
```
Select `/camera/image_raw` from the dropdown.

### Lidar in RViz
```bash
ros2 run rviz2 rviz2 --ros-args -p use_sim_time:=true
```
In RViz:
1. Set **Fixed Frame** to `core`
2. Click **Add** → **By topic** → `/scan` → **LaserScan**
3. Click **Add** → **TF** to visualize the robot joint frames

## Contact Monitor (optional)
In a third terminal, run the leg contact estimator:
```bash
source install/setup.bash
ros2 run hexapod_ws contact_dummy.py
```
This estimates ground contact force for each leg based on tibia joint position.
