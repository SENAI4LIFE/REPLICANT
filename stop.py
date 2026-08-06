import subprocess

PATTERNS = [
    'gz sim',
    'gzserver',
    'gzclient',
    'ruby.*gz',
    'ros_gz_bridge',
    'ros_gz_sim',
    'robot_state_publisher',
    'controller_manager',
    'async_slam_toolbox_node',
    'rviz2',
    'hexapod_runner.py',
    'joystick_hexapod.py',
    'teleop_hexapod.py',
    'ros2 launch hexapod_ws',
]


def kill_pattern(pattern):
    subprocess.run(['pkill', '-9', '-f', pattern])


def main():
    for pattern in PATTERNS:
        kill_pattern(pattern)


if __name__ == '__main__':
    main()
