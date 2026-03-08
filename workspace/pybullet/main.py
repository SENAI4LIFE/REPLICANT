import pybullet as p
import pybullet_data
import time
import math
import numpy as np
import bootcamp

SIM_FREQ      = 120
DT            = 1.0 / SIM_FREQ
TOTAL_PONTOS  = 25
METADE_PONTOS = TOTAL_PONTOS // 2

TOTAL_PONTOS_CIRCULAR = 25

L1 = 2.56
L2 = 9.00
L3 = 11.96

_PI  = math.pi
_RAD = _PI / 180.0
_DEG = 180.0 / _PI

LEG_CONFIGS = [
    (0,  1,  2,  -30, 25, -100, "right"),
    (3,  4,  5,    0, 25, -100, "right"),
    (6,  7,  8,   30, 25, -100, "right"),
    (9,  10, 11,  30, 25, -100, "left"),
    (12, 13, 14,   0, 25, -100, "left"),
    (15, 16, 17, -30, 25, -100, "left"),
]

SHOULDER_POSITIONS = [
    np.array([ 9.30, -5.55, 0.0]),
    np.array([ 0.00, -6.50, 0.0]),
    np.array([-9.50, -5.50, 0.0]),
    np.array([ 9.30,  5.55, 0.0]),
    np.array([ 0.00,  6.50, 0.0]),
    np.array([-9.50,  5.50, 0.0]),
]

OFFSETS = [0, METADE_PONTOS, 0, METADE_PONTOS, 0, METADE_PONTOS]

STEP_LENGTH = -15.0

ANGLES_STOW_BY_LEG = [
    (-30, 90, -135),
    (  0, 90, -135),
    ( 30, 90, -135),
    ( 30, 90, -135),
    (  0, 90, -135),
    (-30, 90, -135),
]

def fk(ombro_deg, femur_deg, tibia_deg):
    o = ombro_deg * _RAD
    f = femur_deg * _RAD
    t = tibia_deg * _RAD
    x = -math.sin(o) * (L1 + L3 * math.cos(f + t) + L2 * math.cos(f))
    y =  math.cos(o) * (L1 + L3 * math.cos(f + t) + L2 * math.cos(f))
    z =  L3 * math.sin(f + t) + L2 * math.sin(f)
    return np.array([x, y, z])

def ik(xyz):
    x, y, z  = float(xyz[0]), float(xyz[1]), float(xyz[2])
    y_prime  = math.sqrt(x*x + y*y) - L1
    Lv       = math.sqrt(z*z + y_prime*y_prime)
    cos_alpha = np.clip((L2*L2 + L3*L3 - Lv*Lv) / (2*L2*L3), -1.0, 1.0)
    cos_beta  = np.clip((Lv*Lv + L2*L2 - L3*L3) / (2*Lv*L2), -1.0, 1.0)
    tibia_rad  = -_PI + math.acos(cos_alpha)
    ombro_rad  = -math.atan2(x, y)
    femur_rad  =  math.acos(cos_beta) + math.atan2(z, y_prime)
    return (ombro_rad * _DEG, femur_rad * _DEG, tibia_rad * _DEG)

def rotation_matrix(roll_deg, pitch_deg, yaw_deg):
    r = roll_deg  * _RAD
    p = pitch_deg * _RAD
    w = yaw_deg   * _RAD
    Rz = np.array([[math.cos(w), -math.sin(w), 0],
                   [math.sin(w),  math.cos(w), 0],
                   [0,            0,            1]])
    Ry = np.array([[ math.cos(p), 0, math.sin(p)],
                   [0,            1, 0           ],
                   [-math.sin(p), 0, math.cos(p)]])
    Rx = np.array([[1, 0,           0           ],
                   [0, math.cos(r), -math.sin(r)],
                   [0, math.sin(r),  math.cos(r)]])
    return Rz @ Ry @ Rx

def build_bezier_points(xyz_ini):
    half = STEP_LENGTH / 2.0
    P0 = [xyz_ini[0] - half,       xyz_ini[2]]
    P1 = [P0[0] + half / 2.0,      P0[1] + 2.0 * abs(half)]
    P3 = [P0[0] + STEP_LENGTH,     P0[1]]
    P2 = [P3[0] - half / 2.0,      P0[1] + 2.0 * abs(half)]
    return P0, P1, P2, P3

def trajetoria_linear(xyz_ini, k, offset, angle_rad, P0, P1, P2, P3):
    kn = (k + offset) % TOTAL_PONTOS
    if kn < METADE_PONTOS:
        t  = float(kn) / (METADE_PONTOS - 1)
        u  = 1.0 - t
        bx = u**3*P0[0] + 3*u**2*t*P1[0] + 3*u*t**2*P2[0] + t**3*P3[0]
        bz = u**3*P0[1] + 3*u**2*t*P1[1] + 3*u*t**2*P2[1] + t**3*P3[1]
        dx = bx - xyz_ini[0]
        x  = xyz_ini[0] + math.cos(angle_rad) * dx
        y  = xyz_ini[1] + math.sin(angle_rad) * dx
        z  = bz
    else:
        t  = float(kn - METADE_PONTOS) / (METADE_PONTOS - 1)
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
    Pz = [xyz_ini[2], xyz_ini[2]+dz1+6.0, xyz_ini[2]+dz2+10.0, xyz_ini[2]+dz]
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
    angle_max_rad = angle_max_deg * _RAD
    kn = k % TOTAL_PONTOS_CIRCULAR
    t  = float(kn) / TOTAL_PONTOS_CIRCULAR
    angle_rad = 2.0 * _PI * t
    roll_deg  = math.cos(angle_rad) * angle_max_rad * _DEG
    pitch_deg = math.sin(angle_rad) * angle_max_rad * _DEG
    return roll_deg, pitch_deg, 0.0

def compute_rebolar(k, xyz_ini):
    roll_deg, pitch_deg, yaw_deg = circular_roll_pitch_yaw(k, 10)
    return compute_ik_corpo(roll_deg, pitch_deg, yaw_deg, xyz_ini)

def setup_simulation():
    p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
    p.configureDebugVisualizer(p.COV_ENABLE_MOUSE_PICKING, 1)
    p.configureDebugVisualizer(p.COV_ENABLE_SHADOWS, 0)
    p.configureDebugVisualizer(p.COV_ENABLE_RGB_BUFFER_PREVIEW, 0)
    p.configureDebugVisualizer(p.COV_ENABLE_TINY_RENDERER, 0)
    p.resetDebugVisualizerCamera(1.0, 270, -25, [-1.0, 0, 1.0])
    p.setGravity(0, 0, -9.81)
    p.setPhysicsEngineParameter(
        numSolverIterations=8,
        numSubSteps=1,
        fixedTimeStep=DT,
        enableConeFriction=0,
    )
    p.loadURDF("plane.urdf", [0, 0, 0])
    bootcamp.CreateTerrain()
    robot = p.loadURDF("robot.urdf", [0, 0, 0])
    for j in range(p.getNumJoints(robot)):
        p.changeDynamics(robot, j,
                         lateralFriction=5.0,
                         jointDamping=0.1,
                         restitution=0.0,
                         maxJointVelocity=6.0)
    return robot

def set_leg(robot, cfg, ombro_deg, femur_deg, tibia_deg):
    jO, jF, jT = cfg[0], cfg[1], cfg[2]
    mv = 4.0
    p.setJointMotorControl2(robot, jO, p.POSITION_CONTROL,  ombro_deg * _RAD, maxVelocity=mv)
    p.setJointMotorControl2(robot, jF, p.POSITION_CONTROL, -femur_deg * _RAD, maxVelocity=mv)
    p.setJointMotorControl2(robot, jT, p.POSITION_CONTROL, -tibia_deg * _RAD, maxVelocity=mv)

def init_leg_state():
    xyz_ini = []
    bezier  = []
    for cfg in LEG_CONFIGS:
        _, _, _, coxa_home, femur_home, tibia_home, _ = cfg
        xyz = fk(coxa_home, femur_home, tibia_home)
        xyz_ini.append(xyz)
        bezier.append(build_bezier_points(xyz))
    return xyz_ini, bezier

def get_body_tilt(robot, smoothed_rpy):
    _, orn = p.getBasePositionAndOrientation(robot)
    rpy    = p.getEulerFromQuaternion(orn)
    for i in range(3):
        smoothed_rpy[i] += (rpy[i] - smoothed_rpy[i]) * 0.2
    return smoothed_rpy[0] * _DEG, smoothed_rpy[1] * _DEG

def compute_andar(k, angle_rad, xyz_ini, bezier):
    results = []
    for i, cfg in enumerate(LEG_CONFIGS):
        P0, P1, P2, P3 = bezier[i]
        current_angle = angle_rad + (_PI if i >= 3 else 0.0)
        xyz = trajetoria_linear(xyz_ini[i], k, OFFSETS[i], current_angle, P0, P1, P2, P3)
        results.append(ik(xyz))
    return results

def compute_andar_circular(k, angle_deg, xyz_ini, bezier):
    angle_abs = abs(angle_deg)
    angle_max = _PI / 9.0
    if angle_deg < 0:
        angle_max = -angle_max
    v_mult = 1.0
    w_mult = 1.0
    if angle_abs in (0, 180):
        w_mult = 0.0
    elif angle_abs == 90:
        v_mult = 0.0

    dir_signs = [1, 1, 1, -1, -1, -1]
    results   = []
    for i, cfg in enumerate(LEG_CONFIGS):
        P0, P1, P2, P3 = bezier[i]
        step_len = P3[0] - xyz_ini[i][0]
        xyz_lin  = trajetoria_linear(xyz_ini[i], k, OFFSETS[i], 0, P0, P1, P2, P3)
        sign     = dir_signs[i]
        shoulder = SHOULDER_POSITIONS[i] * np.array([-1.0, sign, 1.0])
        xyz_rot  = mapeia_circular(xyz_ini[i], xyz_lin, step_len, angle_max * sign, shoulder)
        xyz_b    = (xyz_lin * v_mult + xyz_rot * w_mult) / (v_mult + w_mult)
        results.append(ik(xyz_b))
    return results

def compute_ik_corpo(roll_deg, pitch_deg, yaw_deg, xyz_ini):
    results  = []
    R        = rotation_matrix(-roll_deg, -pitch_deg, -yaw_deg)
    sig_list = [
        np.array([-1.,-1., 1.]),
        np.array([-1.,-1., 1.]),
        np.array([-1.,-1., 1.]),
        np.array([-1., 1., 1.]),
        np.array([-1., 1., 1.]),
        np.array([-1., 1., 1.]),
    ]
    for i in range(6):
        sig   = sig_list[i]
        ombro = SHOULDER_POSITIONS[i]
        ponto = xyz_ini[i] * sig + ombro
        rot   = R @ ponto
        xyz   = (rot - ombro) * sig
        results.append(ik(xyz))
    return results

def run_boot_sequence(robot, xyz_ini):
    total = 100
    meta  = total // 2
    stow  = [fk(*ANGLES_STOW_BY_LEG[i]) for i in range(6)]

    for k in range(meta):
        for i, cfg in enumerate(LEG_CONFIGS):
            dx  = xyz_ini[i][0] - stow[i][0]
            dy  = xyz_ini[i][1] - stow[i][1]
            xyz = bezier_pata(stow[i], k, dx, dy, 0, total)
            o, f, t = ik(xyz)
            set_leg(robot, cfg, o, f, t)
        p.stepSimulation()
        time.sleep(DT * 3)

    above = [np.array([xyz_ini[i][0], xyz_ini[i][1], stow[i][2]]) for i in range(6)]
    for k in range(meta):
        for i, cfg in enumerate(LEG_CONFIGS):
            dz  = xyz_ini[i][2] - above[i][2]
            xyz = bezier_pata(above[i], k, 0, 0, dz, total)
            o, f, t = ik(xyz)
            set_leg(robot, cfg, o, f, t)
        p.stepSimulation()
        time.sleep(DT * 3)

def run_shutdown_sequence(robot, xyz_ini):
    total = 100
    meta  = total // 2
    stow  = [fk(*ANGLES_STOW_BY_LEG[i]) for i in range(6)]
    above = [np.array([xyz_ini[i][0], xyz_ini[i][1], stow[i][2]]) for i in range(6)]

    for k in range(meta):
        for i, cfg in enumerate(LEG_CONFIGS):
            dz  = above[i][2] - xyz_ini[i][2]
            xyz = bezier_pata(xyz_ini[i], k, 0, 0, dz, total)
            o, f, t = ik(xyz)
            set_leg(robot, cfg, o, f, t)
        p.stepSimulation()
        time.sleep(DT * 3)

    for k in range(meta):
        for i, cfg in enumerate(LEG_CONFIGS):
            dx  = stow[i][0] - xyz_ini[i][0]
            dy  = stow[i][1] - xyz_ini[i][1]
            xyz = bezier_pata(above[i], k, dx, dy, 0, total)
            o, f, t = ik(xyz)
            set_leg(robot, cfg, o, f, t)
        p.stepSimulation()
        time.sleep(DT * 3)

def main():
    robot = setup_simulation()

    xyz_ini, bezier = init_leg_state()

    state          = "POWERED_OFF"
    k              = 0
    nav_mode       = "OMNI"
    cam_track      = False
    smoothed_rpy   = [0.0, 0.0, 0.0]
    angle_joystick = 0.0
    last_key       = None
    pose_roll      = 0.0
    pose_pitch     = 0.0
    POSE_MAX       = 15.0
    POSE_STEP      = 1.5

    KEY_E = ord('e')
    KEY_R = ord('r')
    KEY_F = ord('f')
    KEY_C = ord('c')
    KEY_X = ord('x')
    KEY_B = ord('b')
    KEY_Z = ord('z')

    UP    = p.B3G_UP_ARROW
    DOWN  = p.B3G_DOWN_ARROW
    LEFT  = p.B3G_LEFT_ARROW
    RIGHT = p.B3G_RIGHT_ARROW

    for i, cfg in enumerate(LEG_CONFIGS):
        o, f, t = ik(fk(*ANGLES_STOW_BY_LEG[i]))
        set_leg(robot, cfg, o, f, t)

    while True:
        keys = p.getKeyboardEvents()

        if keys.get(KEY_F, 0) & p.KEY_WAS_TRIGGERED:
            cam_track = not cam_track
        if keys.get(KEY_C, 0) & p.KEY_WAS_TRIGGERED:
            nav_mode = "TURN"
        if keys.get(KEY_X, 0) & p.KEY_WAS_TRIGGERED:
            nav_mode = "OMNI"

        if cam_track:
            pos, _ = p.getBasePositionAndOrientation(robot)
            p.resetDebugVisualizerCamera(1.0, 270, -25, [pos[0]-1.0, pos[1], pos[2]+1.0])

        if last_key and not (last_key in keys and keys[last_key] & p.KEY_IS_DOWN):
            last_key = None
        if not last_key:
            for kk in keys:
                if keys[kk] & p.KEY_IS_DOWN:
                    last_key = kk
                    break

        if last_key == KEY_E and state == "POWERED_OFF":
            run_boot_sequence(robot, xyz_ini)
            k     = 0
            state = "IDLE"

        elif last_key == KEY_R and state in ("IDLE", "WALKING", "TURNING", "BALANCE", "REBOLAR", "POSE"):
            run_shutdown_sequence(robot, xyz_ini)
            state = "POWERED_OFF"

        up_dn = (UP   in keys and keys[UP]   & p.KEY_IS_DOWN,
                 DOWN in keys and keys[DOWN]  & p.KEY_IS_DOWN)
        lr    = (LEFT  in keys and keys[LEFT]  & p.KEY_IS_DOWN,
                 RIGHT in keys and keys[RIGHT] & p.KEY_IS_DOWN)

        moving = any(up_dn) or any(lr)

        z_held = KEY_Z in keys and keys[KEY_Z] & p.KEY_IS_DOWN

        if state in ("IDLE", "WALKING", "TURNING", "BALANCE", "REBOLAR", "POSE"):
            if last_key == KEY_B:
                state = "REBOLAR"
            elif z_held:
                state = "POSE"
                if up_dn[0]:
                    pose_pitch = max(-POSE_MAX, pose_pitch - POSE_STEP)
                elif up_dn[1]:
                    pose_pitch = min( POSE_MAX, pose_pitch + POSE_STEP)
                if lr[0]:
                    pose_roll  = max(-POSE_MAX, pose_roll  - POSE_STEP)
                elif lr[1]:
                    pose_roll  = min( POSE_MAX, pose_roll  + POSE_STEP)
            elif moving:
                if nav_mode == "OMNI":
                    if   up_dn[0]: angle_joystick = 180.0
                    elif up_dn[1]: angle_joystick = 0.0
                    elif lr[0]:    angle_joystick = -90.0
                    elif lr[1]:    angle_joystick = 90.0
                    state = "WALKING"
                else:
                    if   up_dn[0]: angle_joystick = 180.0
                    elif up_dn[1]: angle_joystick = 0.0
                    elif lr[0]:    angle_joystick = -90.0
                    elif lr[1]:    angle_joystick = 90.0
                    state = "TURNING"
            else:
                if state not in ("IDLE", "POWERED_OFF", "REBOLAR", "POSE"):
                    state = "IDLE"
                if state == "POSE" and not z_held:
                    pose_roll  = 0.0
                    pose_pitch = 0.0
                    state = "IDLE"

        if state == "WALKING":
            angle_rad = angle_joystick * _RAD
            results   = compute_andar(k, angle_rad, xyz_ini, bezier)
            for i, (o, f, t) in enumerate(results):
                set_leg(robot, LEG_CONFIGS[i], o, f, t)
            k = (k + 1) % TOTAL_PONTOS

        elif state == "TURNING":
            results = compute_andar_circular(k, angle_joystick, xyz_ini, bezier)
            for i, (o, f, t) in enumerate(results):
                set_leg(robot, LEG_CONFIGS[i], o, f, t)
            if abs(angle_joystick) > 90:
                k = (k - 1) % TOTAL_PONTOS
            else:
                k = (k + 1) % TOTAL_PONTOS

        elif state == "BALANCE":
            roll_deg, pitch_deg = get_body_tilt(robot, smoothed_rpy)
            results = compute_ik_corpo(roll_deg, pitch_deg, 0.0, xyz_ini)
            for i, (o, f, t) in enumerate(results):
                set_leg(robot, LEG_CONFIGS[i], o, f, t)

        elif state == "POSE":
            results = compute_ik_corpo(pose_roll, pose_pitch, 0.0, xyz_ini)
            for i, (o, f, t) in enumerate(results):
                set_leg(robot, LEG_CONFIGS[i], o, f, t)

        elif state == "REBOLAR":
            results = compute_rebolar(k, xyz_ini)
            for i, (o, f, t) in enumerate(results):
                set_leg(robot, LEG_CONFIGS[i], o, f, t)
            k = (k + 1) % TOTAL_PONTOS_CIRCULAR

        elif state == "IDLE":
            for i, cfg in enumerate(LEG_CONFIGS):
                o, f, t = ik(xyz_ini[i])
                set_leg(robot, cfg, o, f, t)

        p.stepSimulation()
        time.sleep(DT * 2)

main()
