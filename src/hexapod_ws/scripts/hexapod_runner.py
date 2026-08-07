#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import Float64MultiArray, String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
from tf2_msgs.msg import TFMessage
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
from tf2_ros import Buffer, TransformListener
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException

import math
import numpy as np
import time
import json

TILT_LIMIT_RAD = math.radians(15.0)
ROBOT_RADIUS_M = 0.24

CLEARANCE_WARN_M     = ROBOT_RADIUS_M + 0.25
CLEARANCE_DANGER_M   = ROBOT_RADIUS_M + 0.10
CLEARANCE_MIN_SPEED_SCALE = 0.35

CORRIDOR_SENSE_RANGE          = 0.6
CORRIDOR_ALIGN_GAIN_DEG_PER_M = 20.0
CORRIDOR_MAX_BIAS_DEG         = 8.0
CORRIDOR_BIAS_SMOOTHING       = 0.15

FORWARD_CONE_HALF_WIDTH_DEG = 35.0

LATERAL_OPT_SENSE_RANGE_M    = 1.0
LATERAL_OPT_DEADBAND_M       = 0.04
LATERAL_OPT_GAIN_MPS_PER_M   = 0.12
LATERAL_OPT_MAX_LATERAL_MPS  = 0.06
LATERAL_OPT_MAX_BIAS_DEG     = 10.0
LATERAL_OPT_SMOOTHING        = 0.10
LATERAL_OPT_COSTMAP_WEIGHT   = 0.5
LATERAL_OPT_COSTMAP_PROBES_M = (0.20, 0.35, 0.50)
LATERAL_OPT_BANDS = (
    (35.0, 12.0, 0.6),
    (60.0, 12.0, 1.0),
    (90.0, 12.0, 0.85),
    (120.0, 12.0, 0.5),
)

CMD_VEL_TIMEOUT_S = 0.5
ZERO_VEL_IDLE_DEBOUNCE_S = 0.35

HEADING_SMOOTHING_GAIN     = 0.35
HEADING_SNAP_THRESHOLD_DEG = 100.0
GAIT_SPEED_RAMP_UP_GAIN    = 0.45
GAIT_SPEED_RAMP_DOWN_GAIN  = 0.22

AUTO_ROTATION_DEADBAND_RAD_S   = 0.03
AUTO_ROTATION_HYSTERESIS_RAD_S = 0.05

STALL_CHECK_INTERVAL_S    = 1.5
STALL_MIN_PROGRESS_M      = 0.03
STALL_MIN_PROGRESS_RAD    = math.radians(4.0)
STALL_TICKS_BEFORE_STRAFE = 4
STRAFE_DURATION_S         = 1.0
STRAFE_COOLDOWN_S         = 4.0

L1 = 0.0256
L2 = 0.0900
L3 = 0.1216

TOTAL_PONTOS          = 25
METADE_PONTOS         = TOTAL_PONTOS // 2
TOTAL_PONTOS_CIRCULAR = 25
STEP_LENGTH           = -0.080
GAIT_TICK             = 0.020

BASE_LINEAR_SPEED  = 0.15
BASE_ANGULAR_SPEED = 1.0
GAIT_SPEED_MIN     = 0.3
GAIT_SPEED_MAX     = 3.0

FAILSAFE_RISK_COST       = 90
FAILSAFE_SAFE_COST       = 10
FAILSAFE_SEARCH_RADIUS_M = 1.5
FAILSAFE_CLEAR_TICKS     = 25
FAILSAFE_COSTMAP_FRAME   = 'odom'
FAILSAFE_ROBOT_FRAME     = 'base_link'

PATINHA_TOTAL  = 50
PATINHA_META   = PATINHA_TOTAL // 2
PATINHA_ROLL   = -10.0
PATINHA_PITCH  = -10.0
PATINHA_DX     = -0.100
PATINHA_DY     =  0.0
PATINHA_DZ     =  0.100

OFFSETS = [0, METADE_PONTOS, 0, METADE_PONTOS, 0, METADE_PONTOS]

SHOULDER_POSITIONS = [
    np.array([ 0.0930, -0.0555,  0.0]),
    np.array([ 0.0000, -0.0650,  0.0]),
    np.array([-0.0950, -0.0550,  0.0]),
    np.array([ 0.0930,  0.0555,  0.0]),
    np.array([ 0.0000,  0.0650,  0.0]),
    np.array([-0.0950,  0.0550,  0.0]),
]

ANGLES_STOW_BY_LEG = [
    ( 0.0,  85.0, -135.0),
    ( 0.0,  85.0, -135.0),
    ( 0.0,  85.0, -135.0),
    ( 0.0,  85.0, -135.0),
    ( 0.0,  85.0, -135.0),
    ( 0.0,  85.0, -135.0),
]

LEG_CONFIGS = [
    (-30.0,  25.0, -100.0, "right"),
    (  0.0,  25.0, -100.0, "right"),
    ( 30.0,  25.0, -100.0, "right"),
    ( 30.0,  25.0, -100.0, "left"),
    (  0.0,  25.0, -100.0, "left"),
    (-30.0,  25.0, -100.0, "left"),
]

def fk(ombro_deg: float, femur_deg: float, tibia_deg: float) -> np.ndarray:
    o = math.radians(ombro_deg)
    f = math.radians(femur_deg)
    t = math.radians(tibia_deg)
    x = -math.sin(o) * (L1 + L3 * math.cos(f + t) + L2 * math.cos(f))
    y =  math.cos(o) * (L1 + L3 * math.cos(f + t) + L2 * math.cos(f))
    z =  L3 * math.sin(f + t) + L2 * math.sin(f)
    return np.array([x, y, z])

def ik(xyz: np.ndarray):
    x, y, z   = float(xyz[0]), float(xyz[1]), float(xyz[2])
    y_prime   = math.sqrt(x*x + y*y) - L1
    Lv        = math.sqrt(z*z + y_prime*y_prime)
    cos_alpha = np.clip((L2**2 + L3**2 - Lv**2) / (2*L2*L3), -1.0, 1.0)
    cos_beta  = np.clip((Lv**2 + L2**2 - L3**2) / (2*Lv*L2), -1.0, 1.0)
    tibia_rad = -math.pi + math.acos(cos_alpha)
    ombro_rad = -math.atan2(x, y)
    femur_rad =  math.acos(cos_beta) + math.atan2(z, y_prime)
    return (ombro_rad, femur_rad, tibia_rad)

def rotation_matrix(roll_deg: float, pitch_deg: float, yaw_deg: float) -> np.ndarray:
    r = math.radians(roll_deg)
    p = math.radians(pitch_deg)
    w = math.radians(yaw_deg)
    Rz = np.array([[math.cos(w), -math.sin(w), 0],
                   [math.sin(w),  math.cos(w), 0],
                   [0,            0,            1]])
    Ry = np.array([[ math.cos(p), 0, math.sin(p)],
                   [0,            1, 0           ],
                   [-math.sin(p), 0, math.cos(p)]])
    Rx = np.array([[1, 0,            0           ],
                   [0, math.cos(r), -math.sin(r)],
                   [0, math.sin(r),  math.cos(r)]])
    return Rz @ Ry @ Rx

def build_bezier_points(xyz_ini: np.ndarray):
    half = STEP_LENGTH / 2.0
    P0 = [xyz_ini[0] - half,           xyz_ini[2]]
    P1 = [P0[0] + half / 2.0,          P0[1] + 2.0 * abs(half)]
    P3 = [P0[0] + STEP_LENGTH,         P0[1]]
    P2 = [P3[0] - half / 2.0,          P0[1] + 2.0 * abs(half)]
    return P0, P1, P2, P3

def _ease_smoothstep(t: float) -> float:
    return t * t * (3.0 - 2.0 * t)

def trajetoria_linear(xyz_ini, k, offset, angle_rad, P0, P1, P2, P3):
    kn = (k + offset) % TOTAL_PONTOS
    if kn < METADE_PONTOS:
        t  = float(kn) / (METADE_PONTOS - 1)
        t  = _ease_smoothstep(t)
        u  = 1.0 - t
        bx = u**3*P0[0] + 3*u**2*t*P1[0] + 3*u*t**2*P2[0] + t**3*P3[0]
        bz = u**3*P0[1] + 3*u**2*t*P1[1] + 3*u*t**2*P2[1] + t**3*P3[1]
        dx = bx - xyz_ini[0]
        x  = xyz_ini[0] + math.cos(angle_rad) * dx
        y  = xyz_ini[1] + math.sin(angle_rad) * dx
        z  = bz
    else:
        t  = float(kn - METADE_PONTOS) / (METADE_PONTOS - 1)
        t  = _ease_smoothstep(t)
        bx = P3[0] + (P0[0] - P3[0]) * t
        dx = bx - xyz_ini[0]
        x  = xyz_ini[0] + math.cos(angle_rad) * dx
        y  = xyz_ini[1] + math.sin(angle_rad) * dx
        z  = xyz_ini[2]
    return np.array([x, y, z])

def mapeia_circular(xyz_ini, xyz_atual, step_len, total_angle, shoulder):
    d_alpha = (total_angle / 2.0) * (xyz_atual[0] - xyz_ini[0]) / step_len
    x = xyz_ini[0] + shoulder[0]
    y = xyz_ini[1] + shoulder[1]
    R = math.sqrt(x*x + y*y)
    alpha   = math.atan2(x, y)
    n_alpha = alpha + d_alpha
    return np.array([R*math.sin(n_alpha) - shoulder[0],
                     R*math.cos(n_alpha) - shoulder[1],
                     xyz_atual[2]])

def bezier_pata(xyz_ini, k, dx, dy, dz, total):
    meta = total // 2
    kn   = k % total
    dx1, dx2 = dx / 4.0, dx / 2.0
    dy1, dy2 = dy / 4.0, dy / 2.0
    dz1, dz2 = dz / 4.0, dz / 2.0
    Px = [xyz_ini[0], xyz_ini[0]+dx1, xyz_ini[0]+dx2, xyz_ini[0]+dx]
    Py = [xyz_ini[1], xyz_ini[1]+dy1, xyz_ini[1]+dy2, xyz_ini[1]+dy]
    Pz = [xyz_ini[2], xyz_ini[2]+dz1+0.006, xyz_ini[2]+dz2+0.010, xyz_ini[2]+dz]
    if kn < meta:
        t = float(kn) / (meta - 1)
        u = 1.0 - t
        x = u**3*Px[0] + 3*u**2*t*Px[1] + 3*u*t**2*Px[2] + t**3*Px[3]
        y = u**3*Py[0] + 3*u**2*t*Py[1] + 3*u*t**2*Py[2] + t**3*Py[3]
        z = u**3*Pz[0] + 3*u**2*t*Pz[1] + 3*u*t**2*Pz[2] + t**3*Pz[3]
    else:
        x, y, z = Px[3], Py[3], Pz[3]
    return np.array([x, y, z])

def circular_roll_pitch_yaw(k, angle_max_deg):
    angle_max_rad = math.radians(angle_max_deg)
    kn        = k % TOTAL_PONTOS_CIRCULAR
    t         = float(kn) / TOTAL_PONTOS_CIRCULAR
    angle_rad = 2.0 * math.pi * t
    roll_deg  = math.cos(angle_rad) * math.degrees(angle_max_rad)
    pitch_deg = math.sin(angle_rad) * math.degrees(angle_max_rad)
    return roll_deg, pitch_deg, 0.0

def _rotacao_pata(ponto, roll_deg, pitch_deg, yaw_deg):
    r  = math.radians(roll_deg)
    pi = math.radians(pitch_deg)
    w  = math.radians(yaw_deg)
    x = (ponto[0]*math.cos(pi)*math.cos(w)
         + ponto[1]*(math.cos(w)*math.sin(pi)*math.sin(r) - math.cos(r)*math.sin(w))
         + ponto[2]*(math.sin(r)*math.sin(w) + math.cos(r)*math.cos(w)*math.sin(pi)))
    y = (ponto[0]*math.cos(pi)*math.sin(w)
         + ponto[1]*(math.cos(r)*math.cos(w) + math.sin(pi)*math.sin(r)*math.sin(w))
         + ponto[2]*(math.cos(r)*math.sin(pi)*math.sin(w) - math.cos(w)*math.sin(r)))
    z = (-ponto[0]*math.sin(pi)
         + ponto[1]*math.cos(pi)*math.sin(r)
         + ponto[2]*math.cos(pi)*math.cos(r))
    return np.array([x, y, z])

def lerp(a, b, t):
    return a + (b - a) * t

def compute_andar(k, angle_rad, xyz_ini, bezier):
    results = []
    for i in range(6):
        P0, P1, P2, P3 = bezier[i]
        current_angle   = angle_rad + (math.pi if i >= 3 else 0.0)
        xyz = trajetoria_linear(xyz_ini[i], k, OFFSETS[i], current_angle, P0, P1, P2, P3)
        results.append(ik(xyz))
    return results

def compute_turn_1(k, angle_deg, xyz_ini, bezier):
    angle_abs = abs(angle_deg)
    angle_max = math.pi / 9.0
    if angle_deg < 0:
        angle_max = -angle_max
    v_mult = 1.0
    w_mult = 1.0
    if angle_abs in (0, 180):
        w_mult = 0.0
    elif angle_abs == 90:
        v_mult = 0.0
    dir_signs   = [1, 1, 1, -1, -1, -1]
    angled_legs = {0, 2, 3, 5}
    results     = []
    for i in range(6):
        P0, P1, P2, P3 = bezier[i]
        sign     = dir_signs[i]
        shoulder = SHOULDER_POSITIONS[i] * np.array([-1.0, sign, 1.0])
        if i in angled_legs:
            leg_angle = math.atan2(-xyz_ini[i][0], xyz_ini[i][1])
            step_len  = P3[0] - xyz_ini[i][0]
            xyz_lin   = trajetoria_linear(xyz_ini[i], k, OFFSETS[i], leg_angle, P0, P1, P2, P3)

            xyz_lin0  = trajetoria_linear(xyz_ini[i], k, OFFSETS[i], 0, P0, P1, P2, P3)
            ombro_ini   = -math.atan2(xyz_ini[i][0], xyz_ini[i][1])
            y_prime_ini = math.sqrt(xyz_ini[i][0] ** 2 + xyz_ini[i][1] ** 2) - L1
            reach       = L1 + y_prime_ini
            d_alpha     = (angle_max * sign / 2.0) * (xyz_lin0[0] - xyz_ini[i][0]) / step_len
            ombro_rot   = ombro_ini + d_alpha
            xyz_rot     = np.array([-math.sin(ombro_rot) * reach,
                                     math.cos(ombro_rot) * reach,
                                     xyz_lin[2]])
            xyz_b = (xyz_lin * v_mult + xyz_rot * w_mult) / (v_mult + w_mult)
        else:
            xyz_lin = trajetoria_linear(xyz_ini[i], k, OFFSETS[i], 0, P0, P1, P2, P3)
            step_len = P3[0] - xyz_ini[i][0]
            xyz_rot = mapeia_circular(xyz_ini[i], xyz_lin, step_len, angle_max * sign, shoulder)
            xyz_b   = (xyz_lin * v_mult + xyz_rot * w_mult) / (v_mult + w_mult)
        results.append(ik(xyz_b))
    return results

def compute_turn_2(k, angle_deg, xyz_ini, bezier, walk_snap_deg=5.0, turn_snap_deg=12.0):
    angle_abs = abs(angle_deg)
    angle_max = math.pi / 9.0
    if angle_deg < 0:
        angle_max = -angle_max
    fold_abs = angle_abs if angle_abs <= 90.0 else 180.0 - angle_abs
    if fold_abs <= walk_snap_deg:
        w_mult = 0.0
    elif fold_abs >= 90.0 - turn_snap_deg:
        w_mult = 1.0
    else:
        w_mult = (fold_abs - walk_snap_deg) / (90.0 - turn_snap_deg - walk_snap_deg)
    v_mult = 1.0 - w_mult
    dir_signs = [1, 1, 1, -1, -1, -1]
    results   = []
    for i in range(6):
        P0, P1, P2, P3 = bezier[i]
        sign        = dir_signs[i]
        shoulder    = SHOULDER_POSITIONS[i] * np.array([-1.0, sign, 1.0])
        walk_angle  = math.pi if i >= 3 else 0.0
        xyz_lin  = trajetoria_linear(xyz_ini[i], k, OFFSETS[i], walk_angle, P0, P1, P2, P3)
        step_len = P3[0] - xyz_ini[i][0]
        xyz_rot  = mapeia_circular(xyz_ini[i], xyz_lin, step_len, angle_max * sign, shoulder)
        xyz_b    = xyz_lin * v_mult + xyz_rot * w_mult
        results.append(ik(xyz_b))
    return results

def compute_ik_corpo(roll_deg, pitch_deg, yaw_deg, xyz_ini):
    R = rotation_matrix(-roll_deg, -pitch_deg, -yaw_deg)
    sig_list = [
        np.array([-1., -1.,  1.]),
        np.array([-1., -1.,  1.]),
        np.array([-1., -1.,  1.]),
        np.array([-1.,  1.,  1.]),
        np.array([-1.,  1.,  1.]),
        np.array([-1.,  1.,  1.]),
    ]
    results = []
    for i in range(6):
        sig        = sig_list[i]
        ombro      = SHOULDER_POSITIONS[i]
        world_foot = ombro + xyz_ini[i] * sig
        new_ombro  = R @ ombro
        xyz        = (world_foot - new_ombro) * sig
        results.append(ik(xyz))
    return results

def compute_rebolar(k, xyz_ini):
    roll_deg, pitch_deg, yaw_deg = circular_roll_pitch_yaw(k, 15)
    return compute_ik_corpo(roll_deg, pitch_deg, yaw_deg, xyz_ini)

def compute_dar_patinha(k, xyz_ini):
    t         = min(1.0, float(k) / max(1, PATINHA_META - 1))
    roll      = -PATINHA_ROLL  * t
    pitch     = -PATINHA_PITCH * t
    sig_right = np.array([-1.,  -1.,  1.])
    sig_left  = np.array([-1.,   1.,  1.])
    sig_list  = [sig_left, sig_left, sig_left, sig_right, sig_right, sig_right]
    results   = []
    for i in range(6):
        if i == 3:
            xyz = bezier_pata(xyz_ini[i], k, PATINHA_DX, PATINHA_DY, PATINHA_DZ, PATINHA_TOTAL)
        else:
            sig   = sig_list[i]
            ombro = SHOULDER_POSITIONS[i]
            ponto = xyz_ini[i] * sig + ombro
            rot   = _rotacao_pata(ponto, roll, pitch, 0.0)
            rotated = (rot - ombro) * sig
            xyz   = np.array([rotated[0], rotated[1], xyz_ini[i][2]])
        results.append(ik(xyz))
    return results

class TFRemapper(Node):
    PREFIX = 'tiffany/'

    def __init__(self):
        super().__init__('tf_remapper')

        be_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=100,
        )
        static_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=100,
        )
        static_raw_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=100,
        )
        self.pub = self.create_publisher(TFMessage, '/tf', 100)
        self.pub_static = self.create_publisher(TFMessage, '/tf_static', static_qos)
        self.sub = self.create_subscription(TFMessage, '/tf_raw', self._cb, be_qos)
        self.sub_static = self.create_subscription(TFMessage, '/tf_static_raw', self._cb_static, static_raw_qos)
        self.get_logger().info('TF remapper active')

    def _strip(self, frame: str) -> str:
        return frame[len(self.PREFIX):] if frame.startswith(self.PREFIX) else frame

    def _cb(self, msg: TFMessage):
        for t in msg.transforms:
            t.header.frame_id = self._strip(t.header.frame_id)
            t.child_frame_id  = self._strip(t.child_frame_id)
        self.pub.publish(msg)

    def _cb_static(self, msg: TFMessage):
        for t in msg.transforms:
            t.header.frame_id = self._strip(t.header.frame_id)
            t.child_frame_id  = self._strip(t.child_frame_id)
        self.pub_static.publish(msg)

class ScanRelay(Node):
    def __init__(self):
        super().__init__('scan_relay')
        sub_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        pub_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.pub = self.create_publisher(LaserScan, '/scan', pub_qos)
        self.sub = self.create_subscription(LaserScan, '/scan_bridge', self._cb, sub_qos)
        self.imu_sub = self.create_subscription(Imu, '/imu', self._imu_cb, 10)
        self.tilt = 0.0
        self.get_logger().info('Scan relay active')

    def _imu_cb(self, msg: Imu):
        q = msg.orientation
        sinr = 2.0 * (q.w * q.x + q.y * q.z)
        cosr = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
        roll = math.atan2(sinr, cosr)
        sinp = 2.0 * (q.w * q.y - q.z * q.x)
        pitch = math.asin(np.clip(sinp, -1.0, 1.0))
        self.tilt = max(abs(roll), abs(pitch))

    def _cb(self, msg: LaserScan):
        if self.tilt > TILT_LIMIT_RAD:
            return
        if msg.header.frame_id.startswith('tiffany/'):
            msg.header.frame_id = msg.header.frame_id[len('tiffany/'):]
        self.pub.publish(msg)

class HexapodRunner(Node):
    def __init__(self):
        super().__init__('hexapod_runner')

        self.joint_pub = self.create_publisher(
            Float64MultiArray, '/hexapod_controller/commands', 10)

        self.cmd_sub = self.create_subscription(
            Twist, '/cmd_vel', self._cmd_vel_cb, 10)
        self.state_sub = self.create_subscription(
            String, '/tiffany/state', self._state_cb, 10)
        self.state_feedback_pub = self.create_publisher(
            String, '/tiffany/state_feedback', 10)
        self.imu_sub = self.create_subscription(
            Imu, '/imu', self._imu_cb, 10)

        costmap_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
        )
        self.costmap_sub = self.create_subscription(
            OccupancyGrid, '/local_costmap/costmap', self._costmap_cb, costmap_qos)

        scan_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self._scan_cb, scan_qos)
        self.corridor_bias_deg = 0.0
        self.target_corridor_bias_deg = 0.0
        self.min_obstacle_dist_m = None
        self.forward_obstacle_dist_m = None
        self.clearance_throttling = False

        self.lateral_opt_enabled = False
        self.lateral_clearance_bias_deg = 0.0
        self.target_lateral_clearance_bias_deg = 0.0
        self.scan_left_clearance_m = None
        self.scan_right_clearance_m = None

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.costmap = None
        self.failsafe_active = False
        self.failsafe_clear_ticks = 0

        self.xyz_ini, self.bezier = self._init_leg_state()

        self.state      = 'POWERED_OFF'
        self.prev_state = None

        self.k         = 0
        self.patinha_k = 0

        self.nav_mode = 'TURN_1'

        self.pose_roll  = 0.0
        self.pose_pitch = 0.0
        self.POSE_MAX   = 25.0
        self.POSE_STEP  = 1.5

        self.angle_joystick = 0.0
        self.angle_joystick_target = 0.0
        self._auto_rotation_sign = 0.0

        self.gait_speed = 1.0
        self.gait_speed_target = 1.0
        self.k_accum    = 0.0

        self.smoothed_rpy = [0.0, 0.0, 0.0]

        self.last_joints   = None
        self.idle_from_xyz = None
        self.idle_k        = 0
        self.idle_total    = 25

        self.pending_move_state = None
        self.pending_move_angle = 0.0

        self.pending_gait_state = None
        self.pending_gait_angle = 0.0
        self.waiting_for_sync = False

        self.last_cmd_vel_time = None
        self.cmd_vel_nav_active = False
        self.zero_vel_since = None
        self.stop_requested = False

        self.manual_active = False
        self.safe_mode = False

        self.stall_last_check_time = None
        self.stall_last_xy = None
        self.stall_last_yaw = None
        self.stall_ticks = 0
        self.strafe_active = False
        self.strafe_end_time = None
        self.strafe_resume_state = None
        self.strafe_resume_nav_mode = None
        self.strafe_direction = 1.0
        self.strafe_cooldown_until = None

        self.ready = False

        self.transition_from  = None
        self.transition_k     = 0
        self.transition_total = 10

        self.feedback_tick = 0

        self.create_timer(0.02, self._step)
        self._publish_stow_smooth()
        self.get_logger().info('HexapodRunner ready. State: POWERED_OFF')
        self.get_logger().info('Send /tiffany/state = "BOOT" to start.')

    def _init_leg_state(self):
        xyz_ini = []
        bezier  = []
        for cfg in LEG_CONFIGS:
            coxa_h, femur_h, tibia_h, _ = cfg
            xyz = fk(coxa_h, femur_h, tibia_h)
            xyz_ini.append(xyz)
            bezier.append(build_bezier_points(xyz))
        return xyz_ini, bezier

    def _joints_from_results(self, results):
        flat = []
        for coxa_r, femur_r, tibia_r in results:
            flat.extend([coxa_r, -femur_r, -tibia_r])
        return flat

    def _publish_joints(self, results):
        msg = Float64MultiArray()
        msg.data = self._joints_from_results(results)
        self.joint_pub.publish(msg)
        self.last_joints = msg.data

    def _publish_joints_blended(self, results):
        target = self._joints_from_results(results)
        if self.transition_from is not None:
            t = float(self.transition_k) / self.transition_total
            if t >= 1.0:
                self.transition_from = None
            else:
                t = _ease_smoothstep(t)
                target = [lerp(self.transition_from[i], target[i], t) for i in range(len(target))]
                self.transition_k += 1
        msg = Float64MultiArray()
        msg.data = target
        self.joint_pub.publish(msg)
        self.last_joints = msg.data

    def _publish_stow(self):
        results = [ik(fk(*ANGLES_STOW_BY_LEG[i])) for i in range(6)]
        self._publish_joints(results)

    def _publish_stow_smooth(self, steps=50, dt=0.008):
        stow_results = [ik(fk(*ANGLES_STOW_BY_LEG[i])) for i in range(6)]
        target = self._joints_from_results(stow_results)
        start  = [0.0] * len(target)
        for k in range(steps):
            t = float(k) / (steps - 1)
            t = _ease_smoothstep(t)
            msg = Float64MultiArray()
            msg.data = [lerp(start[j], target[j], t) for j in range(len(target))]
            self.joint_pub.publish(msg)
            self.last_joints = msg.data
            time.sleep(dt)

    def _publish_home(self):
        results = [ik(self.xyz_ini[i]) for i in range(6)]
        self._publish_joints(results)

    def _current_xyz_from_last_joints(self):
        xyz = []
        for i in range(6):
            coxa_r  = self.last_joints[i * 3]
            femur_r = -self.last_joints[i * 3 + 1]
            tibia_r = -self.last_joints[i * 3 + 2]
            xyz.append(fk(math.degrees(coxa_r), math.degrees(femur_r), math.degrees(tibia_r)))
        return xyz

    def _run_boot_sequence(self):
        steps = 50
        stow  = [fk(*ANGLES_STOW_BY_LEG[i]) for i in range(6)]

        for k in range(steps):
            t = float(k) / (steps - 1)
            results = []
            for i in range(6):
                xyz = np.array([lerp(stow[i][0], self.xyz_ini[i][0], t),
                                lerp(stow[i][1], self.xyz_ini[i][1], t),
                                stow[i][2]])
                results.append(ik(xyz))
            self._publish_joints(results)
            time.sleep(0.008)

        above = [np.array([self.xyz_ini[i][0], self.xyz_ini[i][1], stow[i][2]])
                 for i in range(6)]
        for k in range(steps):
            t = float(k) / (steps - 1)
            results = []
            for i in range(6):
                xyz = np.array([self.xyz_ini[i][0],
                                self.xyz_ini[i][1],
                                lerp(above[i][2], self.xyz_ini[i][2], t)])
                results.append(ik(xyz))
            self._publish_joints(results)
            time.sleep(0.008)

        self.k     = 0
        self.state = 'IDLE'
        self.ready = True
        self.get_logger().info('Boot complete. State: IDLE')

    def _run_shutdown_sequence(self):
        steps = 50
        stow  = [fk(*ANGLES_STOW_BY_LEG[i]) for i in range(6)]
        above = [np.array([self.xyz_ini[i][0], self.xyz_ini[i][1], stow[i][2]])
                 for i in range(6)]

        for k in range(steps):
            t = float(k) / (steps - 1)
            results = []
            for i in range(6):
                xyz = np.array([self.xyz_ini[i][0],
                                self.xyz_ini[i][1],
                                lerp(self.xyz_ini[i][2], above[i][2], t)])
                results.append(ik(xyz))
            self._publish_joints(results)
            time.sleep(0.008)

        for k in range(steps):
            t = float(k) / (steps - 1)
            results = []
            for i in range(6):
                xyz = np.array([lerp(self.xyz_ini[i][0], stow[i][0], t),
                                lerp(self.xyz_ini[i][1], stow[i][1], t),
                                above[i][2]])
                results.append(ik(xyz))
            self._publish_joints(results)
            time.sleep(0.008)

        self.state = 'POWERED_OFF'
        self.get_logger().info('Shutdown complete. State: POWERED_OFF')

    def _imu_cb(self, msg: Imu):
        q = msg.orientation
        sinr = 2.0 * (q.w * q.x + q.y * q.z)
        cosr = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
        roll = math.atan2(sinr, cosr)
        sinp = 2.0 * (q.w * q.y - q.z * q.x)
        pitch = math.asin(np.clip(sinp, -1.0, 1.0))
        alpha = 0.1
        self.smoothed_rpy[0] += (roll  - self.smoothed_rpy[0]) * alpha
        self.smoothed_rpy[1] += (pitch - self.smoothed_rpy[1]) * alpha

    def _scan_cb(self, msg: LaserScan):
        def sector_min(lo_deg, hi_deg):
            lo = math.radians(lo_deg)
            hi = math.radians(hi_deg)
            best = float('inf')
            angle = msg.angle_min
            for r in msg.ranges:
                if lo <= angle <= hi and msg.range_min < r < msg.range_max:
                    if r < best:
                        best = r
                angle += msg.angle_increment
            return best

        left  = sector_min(60.0, 100.0)
        right = sector_min(-100.0, -60.0)

        if left < CORRIDOR_SENSE_RANGE and right < CORRIDOR_SENSE_RANGE:
            error = left - right
            bias  = CORRIDOR_ALIGN_GAIN_DEG_PER_M * error
            self.target_corridor_bias_deg = max(
                -CORRIDOR_MAX_BIAS_DEG, min(CORRIDOR_MAX_BIAS_DEG, bias))
        else:
            self.target_corridor_bias_deg = 0.0

        closest = float('inf')
        for r in msg.ranges:
            if msg.range_min < r < msg.range_max and r < closest:
                closest = r
        self.min_obstacle_dist_m = closest if closest != float('inf') else None

        self.scan_left_clearance_m = self._scan_side_clearance(msg, 1.0)
        self.scan_right_clearance_m = self._scan_side_clearance(msg, -1.0)

        travel_deg = self.angle_joystick if self.state in ('WALKING',) else 0.0
        forward = sector_min(
            travel_deg - FORWARD_CONE_HALF_WIDTH_DEG,
            travel_deg + FORWARD_CONE_HALF_WIDTH_DEG,
        )
        self.forward_obstacle_dist_m = forward if forward != float('inf') else None

    def _band_min(self, msg, center_deg, half_width_deg):
        lo = math.radians(center_deg - half_width_deg)
        hi = math.radians(center_deg + half_width_deg)
        best = float('inf')
        angle = msg.angle_min
        for r in msg.ranges:
            if lo <= angle <= hi and msg.range_min < r < msg.range_max:
                if r < best:
                    best = r
            angle += msg.angle_increment
        return best

    def _scan_side_clearance(self, msg, sign):
        total = 0.0
        weight_sum = 0.0
        for center_deg, half_width_deg, weight in LATERAL_OPT_BANDS:
            dist = self._band_min(msg, sign * center_deg, half_width_deg)
            if dist == float('inf'):
                continue
            capped = min(dist, LATERAL_OPT_SENSE_RANGE_M)
            total += capped * weight
            weight_sum += weight
        if weight_sum == 0.0:
            return None
        return total / weight_sum

    def _costmap_lateral_clearance(self):
        if self.costmap is None:
            return None, None
        pose = self._costmap_robot_pose()
        if pose is None:
            return None, None
        rx, ry, ryaw = pose

        def side_score(sign):
            total = 0.0
            count = 0
            for probe in LATERAL_OPT_COSTMAP_PROBES_M:
                wx = rx + probe * math.cos(ryaw + sign * math.pi / 2.0)
                wy = ry + probe * math.sin(ryaw + sign * math.pi / 2.0)
                cost = self._costmap_cost_at(wx, wy)
                if cost is None or cost < 0:
                    continue
                freeness = max(0.0, 1.0 - cost / 100.0)
                total += freeness * probe
                count += 1
            if count == 0:
                return None
            return total / count

        return side_score(1.0), side_score(-1.0)

    def _sample_lateral_clearance(self):
        scan_left = self.scan_left_clearance_m
        scan_right = self.scan_right_clearance_m
        cm_left, cm_right = self._costmap_lateral_clearance()

        if scan_left is None and cm_left is None:
            return None, None
        if cm_left is None:
            return scan_left, scan_right
        if scan_left is None:
            return cm_left, cm_right

        w = LATERAL_OPT_COSTMAP_WEIGHT
        left = (1.0 - w) * scan_left + w * cm_left
        right = (1.0 - w) * scan_right + w * cm_right
        return left, right

    def _apply_lateral_clearance_optimization(self):
        if (not self.lateral_opt_enabled or self.manual_active
                or self.strafe_active or self.failsafe_active
                or self.state != 'WALKING'):
            self.target_lateral_clearance_bias_deg = 0.0
            return

        left_clear, right_clear = self._sample_lateral_clearance()
        if left_clear is None or right_clear is None:
            self.target_lateral_clearance_bias_deg = 0.0
            return

        diff = left_clear - right_clear
        if abs(diff) < LATERAL_OPT_DEADBAND_M:
            self.target_lateral_clearance_bias_deg = 0.0
            return

        lateral_speed = diff * LATERAL_OPT_GAIN_MPS_PER_M
        lateral_speed = max(-LATERAL_OPT_MAX_LATERAL_MPS,
                             min(LATERAL_OPT_MAX_LATERAL_MPS, lateral_speed))
        forward_speed = max(0.05, BASE_LINEAR_SPEED * self.gait_speed)
        bias_deg = math.degrees(math.atan2(lateral_speed, forward_speed))
        self.target_lateral_clearance_bias_deg = max(
            -LATERAL_OPT_MAX_BIAS_DEG, min(LATERAL_OPT_MAX_BIAS_DEG, bias_deg))

    def _state_cb(self, msg: String):
        cmd = msg.data.upper().strip()

        if cmd == 'MANUAL_ON':
            self.manual_active = True
            return
        elif cmd == 'MANUAL_OFF':
            self.manual_active = False
            return
        elif cmd == 'SAFE_ON':
            self.safe_mode = True
            return
        elif cmd == 'SAFE_OFF':
            self.safe_mode = False
            return
        elif cmd == 'LATERAL_OPT_ON':
            self.lateral_opt_enabled = True
            return
        elif cmd == 'LATERAL_OPT_OFF':
            self.lateral_opt_enabled = False
            self.target_lateral_clearance_bias_deg = 0.0
            self.lateral_clearance_bias_deg = 0.0
            return

        if cmd != 'BOOT' and not self.ready:
            return

        if cmd == 'BOOT' and self.state == 'POWERED_OFF':
            self._run_boot_sequence()

        elif cmd == 'SHUTDOWN' and self.state != 'POWERED_OFF':
            self.ready = False
            self._run_shutdown_sequence()

        elif cmd in ('IDLE', 'BALANCE', 'REBOLAR', 'PATINHA') and self.state != 'POWERED_OFF':
            if cmd == 'PATINHA':
                if self.state == 'PATINHA':
                    self.state = 'IDLE'
                    self.nav_mode = 'TURN_1'
                    self.get_logger().info('State → IDLE')
                    return
                self.patinha_k = 0
            self.state = cmd
            if cmd == 'IDLE':
                self.nav_mode = 'TURN_1'
                self.cmd_vel_nav_active = False
                self.pending_move_state = None
                self.stop_requested = False
                self.waiting_for_sync = False
                self.pending_gait_state = None
            self.get_logger().info(f'State → {cmd}')

        elif cmd == 'NAV_OMNI':
            self.nav_mode = 'OMNI'
            if self.state in ('POSE', 'REBOLAR', 'PATINHA'):
                self.state = 'IDLE'
        elif cmd == 'NAV_TURN_2':
            self.nav_mode = 'TURN_2'
            if self.state in ('POSE', 'REBOLAR', 'PATINHA'):
                self.state = 'IDLE'
        elif cmd == 'NAV_TURN_1':
            self.nav_mode = 'TURN_1'
            if self.state in ('POSE', 'REBOLAR', 'PATINHA'):
                self.state = 'IDLE'

        elif cmd.startswith('POSE '):
            try:
                _, r, p = cmd.split()
                self.pose_roll  = float(r)
                self.pose_pitch = float(p)
                self.state = 'POSE'
            except ValueError:
                pass

    def _stable_auto_rotation_az(self, az):
        if abs(az) < AUTO_ROTATION_DEADBAND_RAD_S:
            self._auto_rotation_sign = 0.0
            return 0.0

        candidate_sign = 1.0 if az > 0.0 else -1.0

        if self._auto_rotation_sign == 0.0:
            self._auto_rotation_sign = candidate_sign
        elif candidate_sign != self._auto_rotation_sign:
            flip_threshold = AUTO_ROTATION_DEADBAND_RAD_S + AUTO_ROTATION_HYSTERESIS_RAD_S
            if abs(az) < flip_threshold:
                candidate_sign = self._auto_rotation_sign
            else:
                self._auto_rotation_sign = candidate_sign

        return candidate_sign * abs(az)

    def _cmd_vel_cb(self, msg: Twist):
        if not self.ready:
            return

        self.last_cmd_vel_time = time.monotonic()

        if self.strafe_active:
            return

        lx, ly, az = self._apply_safe_mode_limit(
            msg.linear.x, msg.linear.y, msg.angular.z)

        if self.manual_active:
            self._auto_rotation_sign = 0.0
        else:
            az = self._stable_auto_rotation_az(az)

        if abs(lx) > 0.01 or abs(ly) > 0.01 or abs(az) > 0.01:
            self.cmd_vel_nav_active = True
            self.zero_vel_since = None
            self.stop_requested = False

            lin_mag = math.hypot(lx, ly)
            ang_mag = abs(az)
            lin_ratio = lin_mag / BASE_LINEAR_SPEED
            ang_ratio = ang_mag / BASE_ANGULAR_SPEED
            if lin_mag > 0.01 and ang_mag > 0.01:
                ratio = max(lin_ratio, ang_ratio)
            elif lin_mag > 0.01:
                ratio = lin_ratio
            elif ang_mag > 0.01:
                ratio = ang_ratio
            else:
                ratio = 1.0
            self.gait_speed_target = min(GAIT_SPEED_MAX, max(GAIT_SPEED_MIN, ratio))

            target_state = 'WALKING'
            target_angle = self.angle_joystick

            if self.nav_mode == 'TURN_2':
                if abs(lx) > 0.01 or abs(az) > 0.01:
                    target_angle = math.degrees(math.atan2(az, -lx))
                    target_state = 'TURNING'
            elif self.nav_mode == 'TURN_1':
                if abs(lx) > 0.01:
                    target_angle = 180.0 if lx > 0 else 0.0
                    target_state = 'WALKING'
                elif abs(az) > 0.01:
                    target_angle = 180.0 if az < 0 else 0.0
                    target_state = 'TURNING'
            else:
                if abs(lx) > 0.01 or abs(ly) > 0.01:
                    target_angle = math.degrees(math.atan2(-ly, -lx))
                    target_state = 'WALKING'
                elif abs(az) > 0.01:
                    target_angle = 180.0 if az < 0 else 0.0
                    target_state = 'TURNING'
                else:
                    target_state = 'WALKING'

            in_gait = self.state in ('WALKING', 'TURNING')
            target_gait = target_state in ('WALKING', 'TURNING')

            if in_gait and target_gait:
                if target_state == self.state:
                    self.angle_joystick_target = target_angle
                    self.waiting_for_sync = False
                    self.pending_gait_state = None
                else:
                    self.pending_gait_state = target_state
                    self.pending_gait_angle = target_angle
                    self.waiting_for_sync = True
            elif not in_gait:
                if self.state in ('PATINHA', 'POSE') or self.pending_move_state is not None:
                    self.pending_move_state = target_state
                    self.pending_move_angle = target_angle
                    if self.state != 'IDLE':
                        self.state = 'IDLE'
                else:
                    self.angle_joystick = target_angle
                    self.angle_joystick_target = target_angle
                    self.state = target_state
                    self.waiting_for_sync = False
                    self.pending_gait_state = None
        else:
            self.cmd_vel_nav_active = False
            self.pending_move_state = None
            self.zero_vel_since = None
            self.stop_requested = False
            if not self.manual_active:
                self._auto_rotation_sign = 0.0
            if self.state in ('WALKING', 'TURNING'):
                self.state = 'IDLE'
                self.waiting_for_sync = False
                self.pending_gait_state = None

    def _apply_safe_mode_limit(self, lx, ly, az):
        if not self.safe_mode:
            return lx, ly, az

        if math.hypot(lx, ly) <= 0.01:
            return lx, ly, az

        dist = self.forward_obstacle_dist_m if lx > 0.01 else self.min_obstacle_dist_m
        if dist is None:
            return lx, ly, az

        if dist <= CLEARANCE_DANGER_M:
            return 0.0, 0.0, az

        if dist < CLEARANCE_WARN_M:
            t = (dist - CLEARANCE_DANGER_M) / (CLEARANCE_WARN_M - CLEARANCE_DANGER_M)
            lx *= t
            ly *= t

        return lx, ly, az

    def _advance_steps(self):
        self.k_accum += self.gait_speed
        step = int(self.k_accum)
        self.k_accum -= step
        return step

    def _costmap_cb(self, msg: OccupancyGrid):
        self.costmap = msg

    def _costmap_robot_pose(self):
        try:
            tf = self.tf_buffer.lookup_transform(
                FAILSAFE_COSTMAP_FRAME, FAILSAFE_ROBOT_FRAME, rclpy.time.Time())
        except (LookupException, ConnectivityException, ExtrapolationException):
            return None
        x = tf.transform.translation.x
        y = tf.transform.translation.y
        q = tf.transform.rotation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        return x, y, yaw

    def _costmap_cost_at(self, wx, wy):
        info = self.costmap.info
        mx = int((wx - info.origin.position.x) / info.resolution)
        my = int((wy - info.origin.position.y) / info.resolution)
        if mx < 0 or my < 0 or mx >= info.width or my >= info.height:
            return None
        return self.costmap.data[my * info.width + mx]

    def _find_escape_target(self, rx, ry):
        info = self.costmap.info
        max_cells = int(FAILSAFE_SEARCH_RADIUS_M / info.resolution)
        rmx = int((rx - info.origin.position.x) / info.resolution)
        rmy = int((ry - info.origin.position.y) / info.resolution)
        for ring in range(1, max_cells + 1):
            for dmx in range(-ring, ring + 1):
                for dmy in range(-ring, ring + 1):
                    if max(abs(dmx), abs(dmy)) != ring:
                        continue
                    mx = rmx + dmx
                    my = rmy + dmy
                    if mx < 0 or my < 0 or mx >= info.width or my >= info.height:
                        continue
                    val = self.costmap.data[my * info.width + mx]
                    if 0 <= val <= FAILSAFE_SAFE_COST:
                        wx = info.origin.position.x + (mx + 0.5) * info.resolution
                        wy = info.origin.position.y + (my + 0.5) * info.resolution
                        return wx, wy
        return None

    def _apply_costmap_failsafe(self):
        if not self.ready or self.costmap is None:
            return False

        pose = self._costmap_robot_pose()
        if pose is None:
            return False
        rx, ry, ryaw = pose
        cost = self._costmap_cost_at(rx, ry)

        if not self.failsafe_active:
            if cost is None or cost < FAILSAFE_RISK_COST:
                return False
            self.failsafe_active = True
            self.failsafe_clear_ticks = 0

        target = self._find_escape_target(rx, ry)
        if target is None:
            gx, gy = -1.0, 0.0
        else:
            gx, gy = target[0] - rx, target[1] - ry

        dist = math.hypot(gx, gy)
        if dist < 1e-3:
            dist = 1e-3
        lx_body = (gx * math.cos(-ryaw) - gy * math.sin(-ryaw)) / dist
        ly_body = (gx * math.sin(-ryaw) + gy * math.cos(-ryaw)) / dist
        target_angle = math.degrees(math.atan2(-ly_body, -lx_body))

        self.gait_speed = GAIT_SPEED_MIN
        self.gait_speed_target = GAIT_SPEED_MIN
        if self.state in ('PATINHA', 'POSE') or self.pending_move_state is not None:
            self.pending_move_state = 'WALKING'
            self.pending_move_angle = target_angle
            if self.state != 'IDLE':
                self.state = 'IDLE'
        else:
            self.angle_joystick = target_angle
            self.state = 'WALKING'

        self.angle_joystick_target = target_angle
        if cost is not None and cost < FAILSAFE_RISK_COST:
            self.failsafe_clear_ticks += 1
        else:
            self.failsafe_clear_ticks = 0

        if self.failsafe_clear_ticks >= FAILSAFE_CLEAR_TICKS:
            self.failsafe_active = False
            self.failsafe_clear_ticks = 0

        return True

    def _check_and_apply_strafe_escape(self):
        now = time.monotonic()

        if self.strafe_active:
            if now >= self.strafe_end_time:
                self.strafe_active = False
                self.state = self.strafe_resume_state or 'IDLE'
                if not self.manual_active:
                    self.nav_mode = self.strafe_resume_nav_mode or self.nav_mode
                self.stall_ticks = 0
                self.stall_last_check_time = None
                self.stall_last_xy = None
                self.stall_last_yaw = None
                self.strafe_cooldown_until = now + STRAFE_COOLDOWN_S
            return True

        if self.strafe_cooldown_until is not None and now < self.strafe_cooldown_until:
            return False

        if not self.ready or not self.cmd_vel_nav_active:
            self.stall_ticks = 0
            self.stall_last_check_time = None
            self.stall_last_xy = None
            self.stall_last_yaw = None
            return False

        if self.state not in ('WALKING', 'TURNING'):
            self.stall_ticks = 0
            self.stall_last_check_time = None
            self.stall_last_xy = None
            self.stall_last_yaw = None
            return False

        pose = self._costmap_robot_pose()
        if pose is None:
            return False
        rx, ry, ryaw = pose

        if self.stall_last_check_time is None:
            self.stall_last_check_time = now
            self.stall_last_xy = (rx, ry)
            self.stall_last_yaw = ryaw
            return False

        if now - self.stall_last_check_time < STALL_CHECK_INTERVAL_S:
            return False

        dx = rx - self.stall_last_xy[0]
        dy = ry - self.stall_last_xy[1]
        moved = math.hypot(dx, dy)
        dyaw = abs(math.atan2(math.sin(ryaw - self.stall_last_yaw),
                               math.cos(ryaw - self.stall_last_yaw)))

        self.stall_last_check_time = now
        self.stall_last_xy = (rx, ry)
        self.stall_last_yaw = ryaw

        if self.clearance_throttling:
            self.stall_ticks = 0
            return False

        if moved >= STALL_MIN_PROGRESS_M or dyaw >= STALL_MIN_PROGRESS_RAD:
            self.stall_ticks = 0
            return False

        self.stall_ticks += 1
        if self.stall_ticks < STALL_TICKS_BEFORE_STRAFE:
            return False

        self.stall_ticks = 0

        left_cost = None
        right_cost = None
        if self.costmap is not None:
            probe = ROBOT_RADIUS_M + 0.10
            lx_world = rx + probe * math.cos(ryaw + math.pi / 2.0)
            ly_world = ry + probe * math.sin(ryaw + math.pi / 2.0)
            rx_world = rx + probe * math.cos(ryaw - math.pi / 2.0)
            ry_world = ry + probe * math.sin(ryaw - math.pi / 2.0)
            left_cost = self._costmap_cost_at(lx_world, ly_world)
            right_cost = self._costmap_cost_at(rx_world, ry_world)

        if left_cost is not None and right_cost is not None:
            self.strafe_direction = 1.0 if left_cost <= right_cost else -1.0
        else:
            self.strafe_direction = -self.strafe_direction

        self.strafe_resume_state = self.state
        self.strafe_resume_nav_mode = self.nav_mode
        self.strafe_active = True
        self.strafe_end_time = now + STRAFE_DURATION_S

        if not self.manual_active:
            self.nav_mode = 'OMNI'
        self.gait_speed = GAIT_SPEED_MIN
        self.angle_joystick = 90.0 if self.strafe_direction > 0 else -90.0
        self.angle_joystick_target = self.angle_joystick
        self.state = 'WALKING'
        self.pending_move_state = None

        direction_label = 'left' if self.strafe_direction > 0 else 'right'
        self.get_logger().info(
            f'Stall detected while on route, strafing {direction_label} to clear obstruction')

        return True

    def _is_primarily_rotating(self):
        if self.state != 'TURNING':
            return False
        if self.nav_mode != 'TURN_2':
            return True
        angle_abs = abs(self.angle_joystick)
        fold_abs = angle_abs if angle_abs <= 90.0 else 180.0 - angle_abs
        return fold_abs >= 45.0

    def _apply_clearance_guard(self):
        self.clearance_throttling = False

        if self.state not in ('WALKING', 'TURNING'):
            return

        primarily_rotating = self._is_primarily_rotating()

        if primarily_rotating:
            dist = self.min_obstacle_dist_m
        else:
            dist = self.forward_obstacle_dist_m

        if dist is None:
            return

        if dist >= CLEARANCE_WARN_M:
            return

        self.clearance_throttling = True

        span = max(1e-3, CLEARANCE_WARN_M - CLEARANCE_DANGER_M)
        t = (dist - CLEARANCE_DANGER_M) / span
        t = max(0.0, min(1.0, t))
        scale = CLEARANCE_MIN_SPEED_SCALE + (1.0 - CLEARANCE_MIN_SPEED_SCALE) * t

        if primarily_rotating:
            scale *= 0.85

        self.gait_speed = max(GAIT_SPEED_MIN * scale, self.gait_speed * scale)

    def _step(self):
        if self.cmd_vel_nav_active and self.last_cmd_vel_time is not None:
            if time.monotonic() - self.last_cmd_vel_time > CMD_VEL_TIMEOUT_S:
                self.cmd_vel_nav_active = False
                self.pending_move_state = None
                if self.state in ('WALKING', 'TURNING'):
                    self.stop_requested = True

        if not self._check_and_apply_strafe_escape():
            self._apply_costmap_failsafe()

        self._apply_clearance_guard()

        self.corridor_bias_deg += (
            self.target_corridor_bias_deg - self.corridor_bias_deg
        ) * CORRIDOR_BIAS_SMOOTHING

        self._apply_lateral_clearance_optimization()
        self.lateral_clearance_bias_deg += (
            self.target_lateral_clearance_bias_deg - self.lateral_clearance_bias_deg
        ) * LATERAL_OPT_SMOOTHING

        if self.state in ('WALKING', 'TURNING') and not self.strafe_active:
            diff = self.angle_joystick_target - self.angle_joystick
            diff = math.atan2(math.sin(math.radians(diff)), math.cos(math.radians(diff)))
            diff = math.degrees(diff)
            if abs(diff) >= HEADING_SNAP_THRESHOLD_DEG:
                self.angle_joystick = self.angle_joystick_target
            else:
                self.angle_joystick += diff * HEADING_SMOOTHING_GAIN
        else:
            self.angle_joystick = self.angle_joystick_target

        if not self.strafe_active and not self.failsafe_active:
            speed_diff = self.gait_speed_target - self.gait_speed
            gain = GAIT_SPEED_RAMP_UP_GAIN if speed_diff > 0.0 else GAIT_SPEED_RAMP_DOWN_GAIN
            self.gait_speed += speed_diff * gain
            self.gait_speed = max(GAIT_SPEED_MIN, min(GAIT_SPEED_MAX, self.gait_speed))

        state = self.state

        if state in ('WALKING', 'TURNING') and self.prev_state not in ('WALKING', 'TURNING'):
            self.k = 0
            self.k_accum = 0.0
            self.stop_requested = False
            self.transition_from = list(self.last_joints) if self.last_joints is not None else None
            self.transition_k = 0

        if self.waiting_for_sync and self.state in ('WALKING', 'TURNING') and (self.k == 0 or self.k == METADE_PONTOS):
            self.state = self.pending_gait_state
            self.angle_joystick = self.pending_gait_angle
            self.angle_joystick_target = self.pending_gait_angle
            self.waiting_for_sync = False
            self.pending_gait_state = None
            self.transition_from = None
            self.transition_k = 0

        if state == 'WALKING':
            angle_rad = math.radians(
                self.angle_joystick + self.corridor_bias_deg
                + self.lateral_clearance_bias_deg)
            results   = compute_andar(self.k, angle_rad, self.xyz_ini, self.bezier)
            self._publish_joints_blended(results)
            self.k = (self.k + self._advance_steps()) % TOTAL_PONTOS
            if self.stop_requested and self.k == 0:
                self.stop_requested = False
                self.state = 'IDLE'
                if not self.manual_active:
                    self.nav_mode = 'TURN_1'
                self.waiting_for_sync = False
                self.pending_gait_state = None
                self.pending_gait_angle = 0.0

        elif state == 'TURNING':
            if self.nav_mode == 'TURN_2':
                results = compute_turn_2(
                    self.k, self.angle_joystick, self.xyz_ini, self.bezier)
            else:
                results = compute_turn_1(
                    self.k, self.angle_joystick, self.xyz_ini, self.bezier)
            self._publish_joints_blended(results)
            step = self._advance_steps()
            if abs(self.angle_joystick) > 90:
                self.k = (self.k - step) % TOTAL_PONTOS
            else:
                self.k = (self.k + step) % TOTAL_PONTOS
            if self.stop_requested and self.k == 0:
                self.stop_requested = False
                self.state = 'IDLE'
                if not self.manual_active:
                    self.nav_mode = 'TURN_1'
                self.waiting_for_sync = False
                self.pending_gait_state = None
                self.pending_gait_angle = 0.0

        elif state == 'BALANCE':
            roll_deg  = math.degrees(self.smoothed_rpy[0])
            pitch_deg = math.degrees(self.smoothed_rpy[1])
            results   = compute_ik_corpo(roll_deg, pitch_deg, 0.0, self.xyz_ini)
            self._publish_joints(results)

        elif state == 'POSE':
            results = compute_ik_corpo(
                self.pose_roll, self.pose_pitch, 0.0, self.xyz_ini)
            self._publish_joints(results)

        elif state == 'PATINHA':
            results = compute_dar_patinha(self.patinha_k, self.xyz_ini)
            self._publish_joints(results)
            if self.patinha_k < PATINHA_META - 1:
                self.patinha_k += 1

        elif state == 'REBOLAR':
            results = compute_rebolar(self.k, self.xyz_ini)
            self._publish_joints(results)
            self.k = (self.k + 1) % TOTAL_PONTOS_CIRCULAR

        elif state == 'IDLE':
            if self.prev_state != 'IDLE':
                if self.last_joints is None:
                    self._publish_home()
                else:
                    self.idle_from_xyz = self._current_xyz_from_last_joints()
                    self.idle_k = 0
            if self.idle_from_xyz is not None:
                t = float(self.idle_k) / (self.idle_total - 1)
                t = _ease_smoothstep(t)
                results = []
                for i in range(6):
                    xyz = np.array([lerp(self.idle_from_xyz[i][0], self.xyz_ini[i][0], t),
                                    lerp(self.idle_from_xyz[i][1], self.xyz_ini[i][1], t),
                                    lerp(self.idle_from_xyz[i][2], self.xyz_ini[i][2], t)])
                    results.append(ik(xyz))
                self._publish_joints(results)
                self.idle_k += 1
                if self.idle_k >= self.idle_total:
                    self.idle_from_xyz = None

            if self.idle_from_xyz is None and self.pending_move_state is not None:
                self.angle_joystick     = self.pending_move_angle
                self.angle_joystick_target = self.pending_move_angle
                self.state              = self.pending_move_state
                self.pending_move_state = None

        self.prev_state = state

        self.feedback_tick += 1
        if self.feedback_tick >= 5:
            self.feedback_tick = 0
            self._publish_state_feedback()

    def _publish_state_feedback(self):
        payload = {
            'state': self.state,
            'ready': self.ready,
            'failsafe_active': self.failsafe_active,
            'nav_mode': self.nav_mode,
            'gait_speed': round(self.gait_speed, 2),
            'roll_deg': round(math.degrees(self.smoothed_rpy[0]), 1),
            'pitch_deg': round(math.degrees(self.smoothed_rpy[1]), 1),
            'corridor_bias_deg': round(self.corridor_bias_deg, 1),
            'lateral_opt_enabled': self.lateral_opt_enabled,
            'lateral_clearance_bias_deg': round(self.lateral_clearance_bias_deg, 1),
            'strafe_active': self.strafe_active,
            'min_obstacle_dist_m': (
                round(self.min_obstacle_dist_m, 3)
                if self.min_obstacle_dist_m is not None else None
            ),
            'forward_obstacle_dist_m': (
                round(self.forward_obstacle_dist_m, 3)
                if self.forward_obstacle_dist_m is not None else None
            ),
            'clearance_throttling': self.clearance_throttling,
            'waiting_for_sync': self.waiting_for_sync,
            'manual_active': self.manual_active,
            'safe_mode': self.safe_mode,
        }
        msg = String()
        msg.data = json.dumps(payload)
        self.state_feedback_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)

    hexapod    = HexapodRunner()
    tf_remap   = TFRemapper()
    scan_relay = ScanRelay()

    executor = MultiThreadedExecutor()
    executor.add_node(hexapod)
    executor.add_node(tf_remap)
    executor.add_node(scan_relay)

    try:
        executor.spin()
    except (KeyboardInterrupt, Exception):
        pass
    finally:
        executor.shutdown()
        hexapod.destroy_node()
        tf_remap.destroy_node()
        scan_relay.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()