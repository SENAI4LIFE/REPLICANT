#!/usr/bin/env python3
import os
import sys
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from world_poses import resolve_pose, as_float_param
from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    TimerAction,
    ExecuteProcess,
    RegisterEventHandler,
    LogInfo,
    DeclareLaunchArgument,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, LifecycleNode
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    pkg_name = 'hexapod_ws'
    pkg_share = get_package_share_directory(pkg_name)

    urdf_path = os.path.join(pkg_share, 'description', 'hexapod.urdf.xacro')
    bridge_config = os.path.join(pkg_share, 'config', 'bridge.yaml')
    camera_bridge_config = os.path.join(pkg_share, 'config', 'bridge_camera.yaml')
    lidar_bridge_config = os.path.join(pkg_share, 'config', 'bridge_lidar.yaml')
    params_file = os.path.join(pkg_share, 'config', 'parameters.yaml')
    slam_params = os.path.join(pkg_share, 'config', 'slam_params.yaml')
    nav2_params = os.path.join(pkg_share, 'config', 'nav2_params.yaml')
    rviz_config = os.path.join(pkg_share, 'rviz', 'hexapod.rviz')
    gz_gui_config = os.path.join(pkg_share, 'config', 'gz_gui_top_down.config')

    rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='Launch RViz2 alongside the simulation',
    )

    camera_arg = DeclareLaunchArgument(
        'camera',
        default_value='true',
        description=(
            'Enable the RGBD camera sensor, its bridge topics and point-cloud '
            'processing. LiDAR, SLAM, Nav2 and odometry work normally either way.'
        ),
    )
    is_camera = IfCondition(LaunchConfiguration('camera'))

    lidar_arg = DeclareLaunchArgument(
        'lidar',
        default_value='true',
        description=(
            'Enable the LiDAR sensor and its bridge/topics. Camera, Gazebo, '
            'odometry and the rest of the simulation work normally either way. '
            'SLAM and Nav2 both require /scan, so neither is started when '
            'lidar:=false.'
        ),
    )
    is_lidar = IfCondition(LaunchConfiguration('lidar'))

    home_dir = os.path.expanduser('~')
    default_map_yaml = os.path.join(home_dir, 'map.yaml')
    default_posegraph_base = os.path.join(home_dir, 'map_slam')

    auto_map_yaml = default_map_yaml if os.path.isfile(default_map_yaml) else ''
    auto_continue_mapping = (
        default_posegraph_base if os.path.isfile(default_posegraph_base + '.posegraph') else ''
    )

    saved_map_arg = DeclareLaunchArgument(
        'saved_map',
        default_value=auto_map_yaml,
        description=(
            'Path to a previously saved map YAML to overlay in RViz as a reference layer, '
            f"published on /saved_map. Auto-detected from {default_map_yaml} if present."
        ),
    )

    has_saved_map = IfCondition(
        PythonExpression(["'", LaunchConfiguration('saved_map'), "' != ''"])
    )

    world_arg = DeclareLaunchArgument(
        'world',
        default_value='living_room',
        description="World to load: 'living_room' (default), 'obstacle_arena', or 'small_house'",
    )

    spawn_x_arg = DeclareLaunchArgument(
        'spawn_x',
        default_value='',
        description=(
            'Override the robot spawn/AMCL initial X. Leave empty to use the '
            'per-world default (living_room=0.0, obstacle_arena=0.0, small_house=0.0).'
        ),
    )

    spawn_y_arg = DeclareLaunchArgument(
        'spawn_y',
        default_value='',
        description=(
            'Override the robot spawn/AMCL initial Y. Leave empty to use the '
            'per-world default (living_room=1.5, obstacle_arena=0.0, small_house=0.0).'
        ),
    )

    spawn_yaw_arg = DeclareLaunchArgument(
        'spawn_yaw',
        default_value='',
        description=(
            'Override the robot spawn/AMCL initial yaw. Leave empty to use the '
            'per-world default (0.0 for every world).'
        ),
    )

    auto_nav2 = 'true' if os.path.isfile(default_map_yaml) else 'false'

    nav2_arg = DeclareLaunchArgument(
        'nav2',
        default_value=auto_nav2,
        description=(
            'Enable Nav2 autonomous navigation (localizes against map instead of live SLAM '
            f"mapping). Defaults to true only if {default_map_yaml} already exists, else false "
            '(so a first-time launch maps instead of sitting on an empty map).'
        ),
    )

    map_arg = DeclareLaunchArgument(
        'map',
        default_value=auto_map_yaml,
        description=(
            'Path to the map YAML Nav2 should localize and navigate against (required when nav2:=true). '
            f"Auto-detected from {default_map_yaml} if present."
        ),
    )

    continue_mapping_arg = DeclareLaunchArgument(
        'continue_mapping',
        default_value=auto_continue_mapping,
        description=(
            'Base path (no extension) of a pose-graph previously saved via the '
            '/slam_toolbox/serialize_map service. When set, SLAM resumes that '
            'graph instead of starting a new one, so the live map stays aligned '
            f"with maps saved from earlier sessions of the same graph. Auto-detected from {default_posegraph_base}.posegraph if present."
        ),
    )

    has_continue_mapping = IfCondition(
        PythonExpression(["'", LaunchConfiguration('continue_mapping'), "' != ''"])
    )

    is_nav2 = IfCondition(
        PythonExpression([
            "'", LaunchConfiguration('nav2'), "' == 'true' and '",
            LaunchConfiguration('lidar'), "' == 'true'",
        ])
    )
    is_not_nav2 = IfCondition(
        PythonExpression([
            "'", LaunchConfiguration('nav2'), "' != 'true' and '",
            LaunchConfiguration('lidar'), "' == 'true'",
        ])
    )

    warn_lidar_disabled = LogInfo(
        msg=(
            '[launch] lidar:=false — /scan is unavailable, so SLAM and Nav2 will '
            'not be started. Camera, Gazebo, odometry and the rest of the '
            'simulation still run normally.'
        ),
        condition=UnlessCondition(LaunchConfiguration('lidar')),
    )

    nav2_missing_map = IfCondition(
        PythonExpression([
            "'", LaunchConfiguration('nav2'), "' == 'true' and '",
            LaunchConfiguration('map'), "' == ''",
        ])
    )
    warn_nav2_no_map = LogInfo(
        msg=(
            f'[launch] nav2:=true but no map at {default_map_yaml} and no map:= given — '
            'map_server has nothing to load. Launch with nav2:=false to build one first '
            '(see README), then relaunch.'
        ),
        condition=nav2_missing_map,
    )

    world_path = LaunchConfiguration('world')
    obstacle_arena_world_path = os.path.join(pkg_share, 'worlds', 'obstacle_arena.sdf')
    living_room_world_path = os.path.join(pkg_share, 'worlds', 'living_room.sdf')
    small_house_world_path = os.path.join(
        os.path.expanduser('~'), 'aws-robomaker-small-house-world', 'worlds', 'small_house.world')

    world_path = PythonExpression([
        f"'{small_house_world_path}' if '", world_path, f"' == 'small_house' else ",
        f"'{living_room_world_path}' if '", world_path, f"' == 'living_room' else '{obstacle_arena_world_path}'"
    ])

    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': ParameterValue(
                Command([
                    'xacro ', urdf_path,
                    ' camera:=', LaunchConfiguration('camera'),
                    ' lidar:=', LaunchConfiguration('lidar'),
                ]),
                value_type=str,
            ),
            'use_sim_time': True,
            'publish_frequency': 50.0,
            'ignore_timestamp': True,
        }],
    )

    gazebo_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('ros_gz_sim'),
            'launch', 'gz_sim.launch.py',
        )]),
        launch_arguments={
            'gz_args': ['-r -s -v4 ', world_path],
            'on_exit_shutdown': 'true',
        }.items(),
    )

    gazebo_gui = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('ros_gz_sim'),
            'launch', 'gz_sim.launch.py',
        )]),
        launch_arguments={
            'gz_args': ['-g -v4 --gui-config ', gz_gui_config],
            'on_exit_shutdown': 'false',
        }.items(),
    )

    robot_pose = resolve_pose(
        world_arg_name='world',
        x_override_arg_name='spawn_x',
        y_override_arg_name='spawn_y',
        yaw_override_arg_name='spawn_yaw',
    )
    spawn_x = robot_pose['x']
    spawn_y = robot_pose['y']
    spawn_yaw = robot_pose['yaw']

    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'tiffany',
            '-x', spawn_x,
            '-y', spawn_y,
            '-Y', spawn_yaw,
            '-z', '0.35',
        ],
        output='screen',
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['--ros-args', '-p', f'config_file:={bridge_config}'],
        output='screen',
    )

    camera_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['--ros-args', '-p', f'config_file:={camera_bridge_config}'],
        output='screen',
        condition=is_camera,
    )

    lidar_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['--ros-args', '-p', f'config_file:={lidar_bridge_config}'],
        output='screen',
        condition=is_lidar,
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': True}],
        output='screen',
        condition=IfCondition(LaunchConfiguration('rviz')),
    )

    jsb_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_state_broadcaster',
            '--controller-manager-timeout', '60',
        ],
        output='screen',
    )

    jsb_spawner_delayed = RegisterEventHandler(
        OnProcessExit(
            target_action=spawn_robot,
            on_exit=[
                LogInfo(msg='[launch] spawn_robot done, waiting 3s for controller_manager'),
                TimerAction(period=3.0, actions=[jsb_spawner]),
            ],
        )
    )

    tiffany_ctrl_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'hexapod_controller',
            '--param-file', params_file,
            '--controller-manager-timeout', '60',
        ],
        output='screen',
    )

    ctrl_spawner_after_jsb = RegisterEventHandler(
        OnProcessExit(
            target_action=jsb_spawner,
            on_exit=[
                LogInfo(msg='[launch] JSB active, starting hexapod_controller'),
                tiffany_ctrl_spawner,
            ],
        )
    )

    tiffany_brain = Node(
        package=pkg_name,
        executable='hexapod_runner.py',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    brain_after_ctrl = RegisterEventHandler(
        OnProcessExit(
            target_action=tiffany_ctrl_spawner,
            on_exit=[
                LogInfo(msg='[launch] hexapod_controller active, starting brain'),
                tiffany_brain,
            ],
        )
    )

    saved_map_server = LifecycleNode(
        package='nav2_map_server',
        executable='map_server',
        name='saved_map_server',
        namespace='',
        output='screen',
        parameters=[{
            'yaml_filename': LaunchConfiguration('saved_map'),
            'topic_name': 'saved_map',
            'frame_id': 'map',
            'use_sim_time': True,
        }],
        condition=has_saved_map,
    )

    saved_map_configure = ExecuteProcess(
        cmd=['bash', '-c',
             'until ros2 lifecycle set /saved_map_server configure; do sleep 1; done'],
        output='screen',
        condition=has_saved_map,
    )

    saved_map_activate = ExecuteProcess(
        cmd=['bash', '-c',
             'until ros2 lifecycle set /saved_map_server activate; do sleep 1; done'],
        output='screen',
        condition=has_saved_map,
    )

    saved_map_configure_after_start = RegisterEventHandler(
        OnProcessStart(
            target_action=saved_map_server,
            on_start=[TimerAction(period=1.0, actions=[saved_map_configure])],
        ),
        condition=has_saved_map,
    )

    saved_map_activate_after_configure = RegisterEventHandler(
        OnProcessExit(
            target_action=saved_map_configure,
            on_exit=[saved_map_activate],
        ),
        condition=has_saved_map,
    )

    slam = LifecycleNode(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        namespace='',
        output='screen',
        parameters=[slam_params, {'use_sim_time': True}],
        condition=is_not_nav2,
    )

    slam_configure = ExecuteProcess(
        cmd=['bash', '-c',
             'until ros2 lifecycle set /slam_toolbox configure; do sleep 1; done'],
        output='screen',
        condition=is_not_nav2,
    )

    slam_activate = ExecuteProcess(
        cmd=['bash', '-c',
             'until ros2 lifecycle set /slam_toolbox activate; do sleep 1; done'],
        output='screen',
        condition=is_not_nav2,
    )

    slam_configure_after_start = RegisterEventHandler(
        OnProcessStart(
            target_action=slam,
            on_start=[
                LogInfo(msg='[launch] SLAM toolbox started, configuring'),
                TimerAction(period=1.0, actions=[slam_configure]),
            ],
        ),
        condition=is_not_nav2,
    )

    slam_activate_after_configure = RegisterEventHandler(
        OnProcessExit(
            target_action=slam_configure,
            on_exit=[
                LogInfo(msg='[launch] SLAM toolbox configured, activating'),
                slam_activate,
            ],
        ),
        condition=is_not_nav2,
    )

    slam_deserialize = ExecuteProcess(
        cmd=['bash', '-c', [
            'ros2 service call /slam_toolbox/deserialize_map '
            'slam_toolbox/srv/DeserializePoseGraph "{filename: \'',
            LaunchConfiguration('continue_mapping'),
            '\', match_type: 1, initial_pose: {x: ',
            spawn_x,
            ', y: ',
            spawn_y,
            ', theta: ',
            spawn_yaw,
            '}}"',
        ]],
        output='screen',
        condition=has_continue_mapping,
    )

    slam_deserialize_after_activate = RegisterEventHandler(
        OnProcessExit(
            target_action=slam_activate,
            on_exit=[
                LogInfo(msg='[launch] SLAM toolbox active, resuming saved pose-graph'),
                TimerAction(period=1.0, actions=[slam_deserialize]),
            ],
        ),
        condition=has_continue_mapping,
    )

    slam_after_brain = RegisterEventHandler(
        OnProcessStart(
            target_action=tiffany_brain,
            on_start=[
                LogInfo(msg='[launch] Brain started, launching SLAM toolbox'),
                TimerAction(period=3.0, actions=[slam]),
            ],
        ),
        condition=is_not_nav2,
    )

    nav2_map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{
            'yaml_filename': LaunchConfiguration('map'),
            'topic_name': 'map',
            'frame_id': 'map',
            'use_sim_time': True,
        }],
    )

    nav2_amcl = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[nav2_params, {
            'initial_pose.x': as_float_param(spawn_x),
            'initial_pose.y': as_float_param(spawn_y),
            'initial_pose.z': 0.0,
            'initial_pose.yaw': as_float_param(spawn_yaw),
        }],
    )

    nav2_controller = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[nav2_params],
    )

    nav2_planner = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[nav2_params],
    )

    nav2_behaviors = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        parameters=[nav2_params],
    )

    nav2_bt_navigator = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[nav2_params],
    )

    nav2_waypoint_follower = Node(
        package='nav2_waypoint_follower',
        executable='waypoint_follower',
        name='waypoint_follower',
        output='screen',
        parameters=[nav2_params],
    )

    nav2_lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'autostart': True,
            'bond_timeout': 0.0,
            'node_names': [
                'map_server',
                'amcl',
                'controller_server',
                'planner_server',
                'behavior_server',
                'bt_navigator',
                'waypoint_follower',
            ],
        }],
    )

    nav2_after_brain = RegisterEventHandler(
        OnProcessStart(
            target_action=tiffany_brain,
            on_start=[
                LogInfo(msg='[launch] Brain started, launching Nav2'),
                TimerAction(period=3.0, actions=[
                    nav2_map_server,
                    nav2_amcl,
                    nav2_controller,
                    nav2_planner,
                    nav2_behaviors,
                    nav2_bt_navigator,
                    nav2_waypoint_follower,
                    nav2_lifecycle_manager,
                ]),
            ],
        ),
        condition=is_nav2,
    )

    trajectory_publisher = Node(
        package=pkg_name,
        executable='trajectory_publisher.py',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    trajectory_after_brain = RegisterEventHandler(
        OnProcessStart(
            target_action=tiffany_brain,
            on_start=[
                LogInfo(msg='[launch] Brain started, launching trajectory publisher'),
                TimerAction(period=1.0, actions=[trajectory_publisher]),
            ],
        )
    )

    gui_after_brain = RegisterEventHandler(
        OnProcessStart(
            target_action=tiffany_brain,
            on_start=[
                LogInfo(msg='[launch] Brain started, launching GUI'),
                TimerAction(period=1.0, actions=[gazebo_gui]),
            ],
        )
    )

    return LaunchDescription([
        rviz_arg,
        camera_arg,
        lidar_arg,
        world_arg,
        spawn_x_arg,
        spawn_y_arg,
        spawn_yaw_arg,
        saved_map_arg,
        nav2_arg,
        map_arg,
        continue_mapping_arg,

        rsp,
        gazebo_server,
        spawn_robot,
        bridge,
        camera_bridge,
        lidar_bridge,
        rviz,
        warn_nav2_no_map,
        warn_lidar_disabled,

        saved_map_server,
        saved_map_configure_after_start,
        saved_map_activate_after_configure,

        jsb_spawner_delayed,
        ctrl_spawner_after_jsb,
        brain_after_ctrl,
        slam_after_brain,
        slam_configure_after_start,
        slam_activate_after_configure,
        slam_deserialize_after_activate,
        nav2_after_brain,
        trajectory_after_brain,
        gui_after_brain,
    ])