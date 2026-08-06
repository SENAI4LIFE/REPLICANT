WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]:-$0}")" && pwd)"

source /opt/ros/jazzy/setup.bash
source "$WS_DIR/install/setup.bash"
export GZ_SIM_RESOURCE_PATH=/opt/ros/jazzy/share:"$WS_DIR/install/hexapod_ws/share":"$WS_DIR/install/hexapod_ws/share/hexapod_ws/models"
SMALL_HOUSE_DIR="$HOME/aws-robomaker-small-house-world"
if [ -d "$SMALL_HOUSE_DIR/models" ]; then
  export GZ_SIM_RESOURCE_PATH="$GZ_SIM_RESOURCE_PATH:$SMALL_HOUSE_DIR/models"
  SHOERACK_SDF="$SMALL_HOUSE_DIR/models/aws_robomaker_residential_ShoeRack_01/model.sdf"
  if [ -f "$SHOERACK_SDF" ] && ! grep -q "<izz>0.02</izz>" "$SHOERACK_SDF"; then
    perl -0777 -pi -e 's/(<iyy>0\.04<\/iyy>\r?\n\s*<iyz>0<\/iyz>\r?\n\s*)<ixx>0\.02<\/ixx>/$1<izz>0.02<\/izz>/' "$SHOERACK_SDF"
  fi
  SMALL_HOUSE_WORLD="$SMALL_HOUSE_DIR/worlds/small_house.world"
  if [ -f "$SMALL_HOUSE_WORLD" ] && ! grep -q "gz::sim::systems::Sensors" "$SMALL_HOUSE_WORLD"; then
    perl -0777 -pi -e 's/(<world name=.default.>\r?\n)/$1\n    <plugin filename="gz-sim-physics-system"\n            name="gz::sim::systems::Physics"\/>\n\n    <plugin filename="gz-sim-sensors-system"\n            name="gz::sim::systems::Sensors">\n      <render_engine>ogre2<\/render_engine>\n    <\/plugin>\n\n    <plugin filename="gz-sim-imu-system"\n            name="gz::sim::systems::Imu"\/>\n\n    <plugin filename="gz-sim-contact-system"\n            name="gz::sim::systems::Contact"\/>\n\n    <plugin filename="gz-sim-user-commands-system"\n            name="gz::sim::systems::UserCommands"\/>\n\n    <plugin filename="gz-sim-scene-broadcaster-system"\n            name="gz::sim::systems::SceneBroadcaster"\/>\n\n    <plugin filename="gz-sim-air-pressure-system"\n            name="gz::sim::systems::AirPressure"\/>\n\n/' "$SMALL_HOUSE_WORLD"
  fi
fi
export GZ_PARTITION="hexapod_$$"
export LD_PRELOAD=/usr/lib/x86_64-linux-gnu/libpthread.so.0
export MESA_GL_VERSION_OVERRIDE=3.3
export LIBGL_ALWAYS_SOFTWARE=0

alias rviz='ros2 run rviz2 rviz2 --ros-args -p use_sim_time:=true'
