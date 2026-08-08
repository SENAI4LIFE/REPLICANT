#!/usr/bin/env python3
import math
import sys
import signal
import json
import time

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy

from nav2_msgs.action import NavigateToPose, BackUp
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import String

from tf2_ros import Buffer, TransformListener
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException

from PyQt5.QtCore import Qt, QPoint, QRectF, QTimer
from PyQt5.QtGui import QImage, QPainter, QPen, QBrush, QColor, QTransform
from PyQt5.QtWidgets import (
    QApplication,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QMainWindow,
    QPushButton,
    QVBoxLayout,
    QWidget,
)


STATE_COLORS = {
    'POWERED_OFF': '#555555',
    'IDLE':        '#3B82F6',
    'WALKING':     '#22C55E',
    'TURNING':     '#14B8A6',
    'BALANCE':     '#A855F7',
    'POSE':        '#F97316',
    'PATINHA':     '#EC4899',
    'REBOLAR':     '#D946EF',
}
STATE_UNKNOWN_COLOR = '#374151'

ROBOT_RADIUS_M = 0.24
SAFE_CLEARANCE_M = ROBOT_RADIUS_M + 0.15
CLEARANCE_DANGER_M = ROBOT_RADIUS_M + 0.10
CLEARANCE_WARN_M = ROBOT_RADIUS_M + 0.25

FINAL_APPROACH_RADIUS_M = 0.35
FINAL_APPROACH_ARRIVE_M = 0.10
FINAL_APPROACH_SPIN_STALL_S = 2.5
FINAL_APPROACH_SPEED_MPS = 0.06
FINAL_APPROACH_MAX_TIME_S = 6.0

NAV_STATUS_COLORS = {
    'idle':        '#374151',
    'sending':     '#F59E0B',
    'navigating':  '#3B82F6',
    'succeeded':   '#22C55E',
    'canceled':    '#F97316',
    'aborted':     '#EF4444',
    'rejected':    '#EF4444',
    'unavailable': '#EF4444',
}


def _badge_style(color):
    return (
        f'background-color: {color}; color: white; font-weight: bold;'
        'padding: 4px 10px; border-radius: 6px;'
    )


def _button_style(kind='normal'):
    styles = {
        'normal': (
            'QPushButton {background-color: #3A3A3A; color: #EAEAEA; '
            'border: 1px solid #555555; border-radius: 5px; padding: 6px 10px;}'
            'QPushButton:hover {background-color: #454545;}'
            'QPushButton:pressed {background-color: #2A2A2A;}'
        ),
        'safety': (
            'QPushButton {background-color: #7C4A12; color: #FFE8CC; '
            'border: 1px solid #F59E0B; border-radius: 5px; padding: 6px 10px;}'
            'QPushButton:hover {background-color: #8F5714;}'
            'QPushButton:pressed {background-color: #5E3A0E;}'
        ),
        'shutdown': (
            'QPushButton {background-color: #7A1F1F; color: #FFE0E0; '
            'border: 1px solid #EF4444; border-radius: 5px; padding: 6px 10px;}'
            'QPushButton:hover {background-color: #8E2626;}'
            'QPushButton:pressed {background-color: #5E1818;}'
        ),
        'boot': (
            'QPushButton {background-color: #14532D; color: #DFFCE8; '
            'border: 1px solid #22C55E; border-radius: 5px; padding: 6px 10px;}'
            'QPushButton:hover {background-color: #1A6B39;}'
            'QPushButton:pressed {background-color: #0F3D21;}'
        ),
    }
    return styles.get(kind, styles['normal'])


_NAV_BUTTON_STYLE = (
    'QPushButton {background-color: #3A3A3A; color: #EAEAEA; '
    'border: 1px solid #555555; border-radius: 5px; padding: 6px 10px;}'
    'QPushButton:hover {background-color: #454545;}'
    'QPushButton:pressed {background-color: #2A2A2A;}'
    'QPushButton:checked {background-color: #1D4ED8; color: white; '
    'font-weight: bold; border: 2px solid #93C5FD;}'
)


MAP_VIEW_ROTATION_DEG = -90


class MapCanvas(QWidget):

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumSize(500, 500)
        self.grid = None
        self.width_cells = 0
        self.height_cells = 0
        self.resolution = 0.05
        self.origin_x = 0.0
        self.origin_y = 0.0
        self.image = None
        self.draw_rect = None
        self.robot_pose = None
        self.min_obstacle_dist_m = None
        self.plan_points = []
        self.press_point = None
        self.drag_point = None
        self.on_goal = None
        self.goal_point = None

    def set_map(self, msg):
        self.width_cells = msg.info.width
        self.height_cells = msg.info.height
        self.resolution = msg.info.resolution
        self.origin_x = msg.info.origin.position.x
        self.origin_y = msg.info.origin.position.y

        data = np.array(msg.data, dtype=np.int16).reshape(
            self.height_cells, self.width_cells)

        rgb = np.zeros((self.height_cells, self.width_cells, 3), dtype=np.uint8)
        rgb[data == -1] = (120, 120, 120)
        free = (data >= 0) & (data < 50)
        occ = data >= 50
        rgb[free] = (255, 255, 255)
        rgb[occ] = (20, 20, 20)
        rgb = np.ascontiguousarray(np.flipud(rgb))

        h, w, _ = rgb.shape
        qimg = QImage(rgb.data, w, h, 3 * w, QImage.Format_RGB888)
        self.image = qimg.copy()
        self.update()

    def set_robot_pose(self, pose):
        self.robot_pose = pose
        self.update()

    def set_min_obstacle_dist(self, dist_m):
        self.min_obstacle_dist_m = dist_m
        self.update()

    def _clearance_color(self):
        dist = self.min_obstacle_dist_m
        if dist is None:
            return QColor(120, 170, 255)
        if dist <= CLEARANCE_DANGER_M:
            return QColor(239, 68, 68)
        if dist >= CLEARANCE_WARN_M:
            return QColor(34, 197, 94)
        span = max(1e-3, CLEARANCE_WARN_M - CLEARANCE_DANGER_M)
        t = (dist - CLEARANCE_DANGER_M) / span
        r = int(239 + (34 - 239) * t)
        g = int(68 + (197 - 68) * t)
        b = int(68 + (94 - 68) * t)
        return QColor(r, g, b)

    def set_plan(self, poses):
        self.plan_points = [(p.pose.position.x, p.pose.position.y) for p in poses]
        self.update()

    def clear_plan(self):
        self.plan_points = []
        self.goal_point = None
        self.update()

    def _world_to_widget(self, x, y):
        if self.draw_rect is None or self.image is None:
            return None
        col = (x - self.origin_x) / self.resolution
        row = (y - self.origin_y) / self.resolution
        img_row = self.height_cells - 1 - row
        rx, ry, rw, rh = self.draw_rect
        scale = rw / self.image.width()
        px = rx + col * scale
        py = ry + img_row * scale
        return QPoint(int(px), int(py))

    def _widget_to_world(self, pos):
        if self.draw_rect is None or self.image is None:
            return None
        rx, ry, rw, rh = self.draw_rect
        if not (rx <= pos.x() <= rx + rw and ry <= pos.y() <= ry + rh):
            return None
        scale = self.image.width() / rw
        col = (pos.x() - rx) * scale
        img_row = (pos.y() - ry) * scale
        row = self.height_cells - 1 - img_row
        x = self.origin_x + col * self.resolution
        y = self.origin_y + row * self.resolution
        return (x, y)

    def _view_transform(self):
        if self.draw_rect is None:
            return QTransform()
        rx, ry, rw, rh = self.draw_rect
        center = QPoint(int(rx + rw / 2.0), int(ry + rh / 2.0))
        t = QTransform()
        t.translate(center.x(), center.y())
        t.rotate(MAP_VIEW_ROTATION_DEG)
        t.translate(-center.x(), -center.y())
        return t

    def _view_to_map_pos(self, pos):
        if self.draw_rect is None:
            return pos
        inverted, invertible = self._view_transform().inverted()
        if not invertible:
            return pos
        return inverted.map(pos)

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        painter.fillRect(self.rect(), QColor(32, 32, 32))

        if self.image is None:
            painter.setPen(QColor(200, 200, 200))
            painter.drawText(self.rect(), Qt.AlignCenter, 'Waiting for /map ...')
            return

        w, h = self.width(), self.height()
        iw, ih = self.image.width(), self.image.height()
        scale = min(w / iw, h / ih)
        dw, dh = iw * scale, ih * scale
        dx, dy = (w - dw) / 2.0, (h - dh) / 2.0
        self.draw_rect = (dx, dy, dw, dh)
        painter.setTransform(self._view_transform(), True)
        painter.drawImage(QRectF(dx, dy, dw, dh), self.image)

        if len(self.plan_points) > 1:
            widget_points = [self._world_to_widget(x, y) for x, y in self.plan_points]
            widget_points = [p for p in widget_points if p is not None]
            if len(widget_points) > 1:
                painter.setPen(QPen(QColor(255, 180, 40), 3))
                for a, b in zip(widget_points, widget_points[1:]):
                    painter.drawLine(a, b)

        if self.goal_point is not None:
            gx, gy = self.goal_point
            center = self._world_to_widget(gx, gy)
            if center is not None:
                painter.setBrush(QBrush(QColor(255, 80, 80)))
                painter.setPen(QPen(QColor(255, 220, 220), 2))
                painter.drawEllipse(center, 7, 7)

        if self.robot_pose is not None:
            x, y, yaw = self.robot_pose
            center = self._world_to_widget(x, y)
            if center is not None:
                if self.draw_rect is not None and self.image is not None:
                    _, _, rw, _ = self.draw_rect
                    px_per_m = (rw / self.image.width()) / self.resolution
                    clearance_px = max(2, int(round(ROBOT_RADIUS_M * px_per_m)))
                    clearance_color = self._clearance_color()
                    fill = QColor(clearance_color)
                    fill.setAlpha(60)
                    painter.setBrush(QBrush(fill))
                    painter.setPen(QPen(clearance_color, 2))
                    painter.drawEllipse(center, clearance_px, clearance_px)

                tip = self._world_to_widget(
                    x + 0.3 * math.cos(yaw), y + 0.3 * math.sin(yaw))
                painter.setBrush(QBrush(QColor(60, 140, 255)))
                painter.setPen(QPen(QColor(220, 235, 255), 2))
                painter.drawEllipse(center, 8, 8)
                if tip is not None:
                    painter.drawLine(center, tip)

        if self.press_point is not None and self.drag_point is not None:
            painter.setPen(QPen(QColor(80, 220, 120), 3))
            painter.drawLine(self.press_point, self.drag_point)
            painter.setBrush(QBrush(QColor(80, 220, 120)))
            painter.drawEllipse(self.press_point, 6, 6)

    def mousePressEvent(self, event):
        if event.button() == Qt.LeftButton:
            pos = self._view_to_map_pos(event.pos())
            world = self._widget_to_world(pos)
            if world is not None:
                self.press_point = pos
                self.drag_point = pos
                self.update()

    def mouseMoveEvent(self, event):
        if self.press_point is not None:
            self.drag_point = self._view_to_map_pos(event.pos())
            self.update()

    def mouseReleaseEvent(self, event):
        if event.button() != Qt.LeftButton or self.press_point is None:
            return
        start_world = self._widget_to_world(self.press_point)
        end_world = self._widget_to_world(self._view_to_map_pos(event.pos()))
        self.press_point = None
        self.drag_point = None
        self.update()

        if start_world is None:
            return

        x, y = start_world
        yaw = 0.0
        if end_world is not None:
            dx = end_world[0] - x
            dy = end_world[1] - y
            if math.hypot(dx, dy) > 0.05:
                yaw = math.atan2(dy, dx)

        self.goal_point = (x, y)
        self.update()

        if self.on_goal:
            self.on_goal(x, y, yaw)


class NavGoalNode(Node):

    def __init__(self):
        super().__init__('nav_goal_gui')

        map_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.map_sub = self.create_subscription(
            OccupancyGrid, '/map', self._map_cb, map_qos)

        costmap_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.costmap_sub = self.create_subscription(
            OccupancyGrid, '/local_costmap/costmap', self._costmap_cb, costmap_qos)

        self.plan_sub = self.create_subscription(
            Path, '/plan', self._plan_cb, 10)

        self.feedback_sub = self.create_subscription(
            String, '/tiffany/state_feedback', self._feedback_state_cb, 10)

        self.state_pub = self.create_publisher(String, '/tiffany/state', 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.nav_status_pub = self.create_publisher(String, '/tiffany/nav2_status', 10)
        self.action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.backup_client = ActionClient(self, BackUp, 'backup')

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.on_map = None
        self.on_status = None
        self.on_plan = None
        self.on_result = None
        self.on_hexapod_state = None
        self.on_distance = None
        self.on_safe_retry = None
        self._goal_handle = None
        self._last_goal_xy = None
        self._auto_retry_used = False
        self._goal_generation = 0

        self.last_feedback_time = None
        self.nav_active = False
        self.nav_start_time = None
        self.distance_remaining = None
        self.last_map = None
        self.last_costmap = None
        self.last_goal_yaw = 0.0

        self.last_hexapod_state = None
        self.last_nav_mode = 'OMNI_1'
        self.manual_active = False
        self.lateral_opt_active = False
        self._recovery_active = False
        self._spin_near_goal_since = None
        self._spin_near_goal_start_dist = None
        self._final_strafe_active = False
        self._final_strafe_end_time = None
        self._final_strafe_prev_nav_mode = 'OMNI_1'

        self.create_timer(0.1, self._final_approach_check)

    def _map_cb(self, msg):
        self.last_map = msg
        if self.on_map:
            self.on_map(msg)

    def _costmap_cb(self, msg):
        self.last_costmap = msg

    def _plan_cb(self, msg):
        if self.on_plan:
            self.on_plan(msg.poses)

    def _feedback_state_cb(self, msg):
        self.last_feedback_time = time.monotonic()
        try:
            data = json.loads(msg.data)
        except (json.JSONDecodeError, TypeError):
            return
        self.last_hexapod_state = data.get('state')
        self.last_nav_mode = data.get('nav_mode', self.last_nav_mode)
        manual_active = data.get('manual_active', self.manual_active)
        if manual_active and not self.manual_active and self.nav_active:
            self.cancel_goal()
        self.manual_active = manual_active
        self._refresh_lateral_opt()
        if self.on_hexapod_state:
            self.on_hexapod_state(data)

    def send_state(self, state):
        msg = String()
        msg.data = state
        self.state_pub.publish(msg)

    def _emit_status(self, key, text):
        if self.on_status:
            self.on_status(key, text)
        msg = String()
        msg.data = key
        self.nav_status_pub.publish(msg)

    def _set_lateral_opt(self, active):
        if active == self.lateral_opt_active:
            return
        self.lateral_opt_active = active
        self.send_state('LATERAL_OPT_ON' if active else 'LATERAL_OPT_OFF')

    def _refresh_lateral_opt(self):
        should_be_active = (
            self.nav_active
            and not self.manual_active
            and not self._recovery_active
            and not self._final_strafe_active
        )
        self._set_lateral_opt(should_be_active)

    def current_pose(self):
        try:
            tf = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
        except (LookupException, ConnectivityException, ExtrapolationException):
            return None
        q = tf.transform.rotation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                          1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        return (tf.transform.translation.x, tf.transform.translation.y, yaw)

    def _final_approach_check(self):
        if self._final_strafe_active:
            self._final_strafe_tick()
            return

        if not self.nav_active or self._goal_handle is None or self._last_goal_xy is None:
            self._spin_near_goal_since = None
            self._spin_near_goal_start_dist = None
            return

        if self.distance_remaining is None or self.distance_remaining > FINAL_APPROACH_RADIUS_M:
            self._spin_near_goal_since = None
            self._spin_near_goal_start_dist = None
            return

        pose = self.current_pose()
        if pose is None:
            self._spin_near_goal_since = None
            self._spin_near_goal_start_dist = None
            return

        gx, gy = self._last_goal_xy
        actual_dist = math.hypot(gx - pose[0], gy - pose[1])
        if actual_dist > FINAL_APPROACH_RADIUS_M:
            self._spin_near_goal_since = None
            self._spin_near_goal_start_dist = None
            return

        if actual_dist <= FINAL_APPROACH_ARRIVE_M:
            self._spin_near_goal_since = None
            self._spin_near_goal_start_dist = None
            return

        if self.last_hexapod_state != 'TURNING':
            self._spin_near_goal_since = None
            self._spin_near_goal_start_dist = None
            return

        now = time.monotonic()
        if self._spin_near_goal_since is None:
            self._spin_near_goal_since = now
            self._spin_near_goal_start_dist = actual_dist
            return

        if now - self._spin_near_goal_since >= FINAL_APPROACH_SPIN_STALL_S:
            start_dist = self._spin_near_goal_start_dist
            self._spin_near_goal_since = None
            self._spin_near_goal_start_dist = None
            progress = (start_dist - actual_dist) if start_dist is not None else 0.0
            if progress < FINAL_APPROACH_ARRIVE_M:
                self._start_final_strafe()

    def _start_final_strafe(self):
        if self.manual_active:
            return

        self._goal_generation += 1
        if self._goal_handle is not None:
            self._goal_handle.cancel_goal_async()
        self._goal_handle = None
        self.nav_active = False

        self._final_strafe_prev_nav_mode = self.last_nav_mode or 'TURN_1'
        self.send_state('NAV_OMNI_1')
        self._final_strafe_active = True
        self._final_strafe_end_time = time.monotonic() + FINAL_APPROACH_MAX_TIME_S
        self._refresh_lateral_opt()

        self._emit_status('sending', 'Close to goal, strafing in directly...')

    def _final_strafe_tick(self):
        now = time.monotonic()
        pose = self.current_pose()
        target = self._last_goal_xy

        if self.manual_active:
            self._stop_final_strafe(succeeded=False)
            return

        if pose is None or target is None or now >= self._final_strafe_end_time:
            self._stop_final_strafe(succeeded=False)
            return

        x, y, yaw = pose
        gx, gy = target[0] - x, target[1] - y
        dist = math.hypot(gx, gy)

        if dist <= FINAL_APPROACH_ARRIVE_M:
            self._stop_final_strafe(succeeded=True)
            return

        cos_y, sin_y = math.cos(-yaw), math.sin(-yaw)
        lx_body = (gx * cos_y - gy * sin_y) / dist
        ly_body = (gx * sin_y + gy * cos_y) / dist

        twist = Twist()
        twist.linear.x = lx_body * FINAL_APPROACH_SPEED_MPS
        twist.linear.y = ly_body * FINAL_APPROACH_SPEED_MPS
        self.cmd_vel_pub.publish(twist)

    def _stop_final_strafe(self, succeeded):
        self._final_strafe_active = False
        self._refresh_lateral_opt()
        self.cmd_vel_pub.publish(Twist())
        if not self.manual_active:
            self.send_state(f'NAV_{self._final_strafe_prev_nav_mode}')
        if succeeded:
            self._emit_status('succeeded', 'Reached goal via final strafe')
        else:
            self._emit_status('idle', 'Final strafe stopped')
        if self.on_result:
            self.on_result()

    def find_safe_point(self, x, y, clearance_m=SAFE_CLEARANCE_M, search_radius_m=2.0):
        msg = self.last_costmap
        use_costmap = msg is not None
        if msg is None:
            msg = self.last_map
        if msg is None:
            return None

        info = msg.info
        res = info.resolution
        w = info.width
        h = info.height
        ox = info.origin.position.x
        oy = info.origin.position.y

        grid = np.array(msg.data, dtype=np.int16).reshape(h, w)

        def to_cell(px, py):
            return (int(round((px - ox) / res)), int(round((py - oy) / res)))

        def in_bounds(c, r):
            return 0 <= c < w and 0 <= r < h
        occ_threshold = 30 if use_costmap else 50
        if use_costmap:
            effective_clearance = max(ROBOT_RADIUS_M, clearance_m * 0.5)
        else:
            effective_clearance = clearance_m

        def is_safe(c, r):
            clearance_cells = max(1, int(round(effective_clearance / res)))
            c0, c1 = max(0, c - clearance_cells), min(w, c + clearance_cells + 1)
            r0, r1 = max(0, r - clearance_cells), min(h, r + clearance_cells + 1)
            patch = grid[r0:r1, c0:c1]
            if patch.size == 0:
                return False
            if np.any(patch >= occ_threshold):
                return False
            if not use_costmap and np.any(patch == -1):
                return False
            return True

        cx, cy = to_cell(x, y)
        max_ring = max(1, int(round(search_radius_m / res)))

        if in_bounds(cx, cy) and is_safe(cx, cy):
            return (x, y)

        for ring in range(1, max_ring + 1):
            candidates = []
            for dc in range(-ring, ring + 1):
                for dr in range(-ring, ring + 1):
                    if max(abs(dc), abs(dr)) != ring:
                        continue
                    c, r = cx + dc, cy + dr
                    if not in_bounds(c, r):
                        continue
                    if is_safe(c, r):
                        candidates.append((c, r))
            if candidates:
                c, r = min(
                    candidates,
                    key=lambda cr: (cr[0] - cx) ** 2 + (cr[1] - cy) ** 2,
                )
                sx = ox + c * res
                sy = oy + r * res
                return (sx, sy)

        if use_costmap:
            saved_costmap = self.last_costmap
            self.last_costmap = None
            try:
                return self.find_safe_point(x, y, clearance_m, search_radius_m)
            finally:
                self.last_costmap = saved_costmap

        return None

    def send_goal(self, x, y, yaw, is_retry=False):
        self.last_goal_yaw = yaw
        self._last_goal_xy = (x, y)
        if not is_retry:
            self._auto_retry_used = False
        self._spin_near_goal_since = None
        self._spin_near_goal_start_dist = None
        self._final_strafe_active = False
        self._goal_generation += 1
        my_generation = self._goal_generation
        goal = NavigateToPose.Goal()
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.orientation.z = math.sin(yaw / 2.0)
        pose.pose.orientation.w = math.cos(yaw / 2.0)
        goal.pose = pose

        if not self.action_client.server_is_ready():
            self._emit_status('unavailable', 'Nav2 action server not available yet')
            return

        self._emit_status('sending', f'Sending goal ({x:.2f}, {y:.2f})')

        if not self.manual_active:
            self.send_state('NAV_OMNI_1')

        self.nav_active = True
        self.nav_start_time = time.monotonic()
        self.distance_remaining = None
        self._refresh_lateral_opt()
        if self.on_distance:
            self.on_distance(None, None)

        future = self.action_client.send_goal_async(
            goal, feedback_callback=self._feedback_cb)
        future.add_done_callback(
            lambda f: self._goal_response_cb(f, my_generation))

    def cancel_goal(self):
        self._goal_generation += 1
        my_generation = self._goal_generation
        self._spin_near_goal_since = None
        self._spin_near_goal_start_dist = None
        self._final_strafe_active = False
        handle = self._goal_handle
        if handle is not None:
            cancel_future = handle.cancel_goal_async()
            cancel_future.add_done_callback(
                lambda _f: self._cancel_confirmed(my_generation))
        self._goal_handle = None
        self.nav_active = False
        self.nav_start_time = None
        self.distance_remaining = None
        self._refresh_lateral_opt()
        stop = Twist()
        self.cmd_vel_pub.publish(stop)
        if self.on_distance:
            self.on_distance(None, None)

    def _cancel_confirmed(self, generation):
        if generation != self._goal_generation:
            return
        self.cmd_vel_pub.publish(Twist())

    def recover_and_go_safe(self):
        self._goal_generation += 1
        my_generation = self._goal_generation

        self._recovery_active = True
        self._refresh_lateral_opt()

        self._emit_status('sending', 'Backing away from obstacle...')

        if not self.backup_client.server_is_ready():
            self._after_backup(my_generation)
            return

        goal = BackUp.Goal()
        goal.target.x = -(ROBOT_RADIUS_M + 0.05)
        goal.speed = 0.08
        goal.time_allowance.sec = 5

        future = self.backup_client.send_goal_async(goal)

        def _backup_response(f):
            if my_generation != self._goal_generation:
                return
            handle = f.result()
            if handle is None or not handle.accepted:
                self._after_backup(my_generation)
                return
            rf = handle.get_result_async()
            rf.add_done_callback(lambda _rf: self._after_backup(my_generation))

        future.add_done_callback(_backup_response)

    def _after_backup(self, generation):
        if generation != self._goal_generation:
            return
        self._emit_status('sending', 'Obstacle ahead, looking for a way around...')
        self._go_to_safe_point(generation)

    def _go_to_safe_point(self, generation):
        if generation != self._goal_generation:
            return
        pose = self.current_pose()
        if pose is None:
            self._emit_status('idle', 'No robot pose available yet')
            return
        x, y, _ = pose
        safe = self.find_safe_point(x, y)
        if safe is None:
            self._emit_status('idle', 'No safe nearby spot found')
            return
        if self.on_safe_retry:
            self.on_safe_retry(safe[0], safe[1])
        self.send_goal(safe[0], safe[1], self.last_goal_yaw)

    def _goal_response_cb(self, future, generation):
        goal_handle = future.result()
        if generation != self._goal_generation:
            if goal_handle is not None and goal_handle.accepted:
                goal_handle.cancel_goal_async()
            return
        if goal_handle is None or not goal_handle.accepted:
            self.nav_active = False
            self._refresh_lateral_opt()
            self._emit_status('rejected', 'Goal rejected')
            return
        self._goal_handle = goal_handle
        self._refresh_lateral_opt()
        self._emit_status('navigating', 'Goal accepted, navigating...')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            lambda f: self._result_cb(f, generation))

    def _feedback_cb(self, feedback_msg):
        remaining = feedback_msg.feedback.distance_remaining
        self.distance_remaining = remaining
        elapsed = None
        if self.nav_start_time is not None:
            elapsed = time.monotonic() - self.nav_start_time
        if self.on_distance:
            self.on_distance(remaining, elapsed)
        self._emit_status('navigating', f'Navigating... {remaining:.2f} m remaining')

    def _result_cb(self, future, generation):
        if generation != self._goal_generation:
            return
        status = future.result().status
        names = {
            4: ('succeeded', 'Succeeded'),
            5: ('canceled', 'Canceled'),
            6: ('aborted', 'Aborted'),
        }
        key, label = names.get(status, ('idle', f'Finished (status {status})'))
        self.nav_active = False
        self._goal_handle = None

        if key == 'aborted' and not self._auto_retry_used and self._last_goal_xy is not None:
            self._auto_retry_used = True
            self._emit_status(
                'sending',
                'Goal aborted near obstacle, backing off and retrying...')
            self.recover_and_go_safe()
            return

        self._recovery_active = False
        self._refresh_lateral_opt()
        self._emit_status(key, label)
        if self.on_result:
            self.on_result()


class NavGoalWindow(QMainWindow):

    def __init__(self, node):
        super().__init__()
        self.node = node
        self.setWindowTitle('Tiffany Nav2 Destination')
        self.setStyleSheet('QMainWindow {background-color: #202020;} '
                           'QLabel {color: #E5E5E5;} '
                           'QGroupBox {color: #B8B8B8; font-weight: bold; '
                           'border: 1px solid #444444; border-radius: 6px; '
                           'margin-top: 10px; padding-top: 8px;} '
                           'QGroupBox::title {subcontrol-origin: margin; '
                           'left: 8px; padding: 0 4px;}')

        central = QWidget()
        layout = QVBoxLayout(central)
        layout.setSpacing(8)
        layout.setContentsMargins(10, 10, 10, 10)

        badge_row = QHBoxLayout()
        self.hexapod_badge = QLabel('HEXAPOD: NO DATA')
        self.hexapod_badge.setStyleSheet(_badge_style(STATE_UNKNOWN_COLOR))
        self.nav_badge = QLabel('NAV2: IDLE')
        self.nav_badge.setStyleSheet(_badge_style(NAV_STATUS_COLORS['idle']))
        badge_row.addWidget(self.hexapod_badge)
        badge_row.addWidget(self.nav_badge)
        badge_row.addStretch(1)
        layout.addLayout(badge_row)

        top_row = QHBoxLayout()
        boot_btn = QPushButton('Boot')
        boot_btn.setStyleSheet(_button_style('boot'))
        boot_btn.clicked.connect(lambda: self.node.send_state('BOOT'))
        shutdown_btn = QPushButton('Shutdown')
        shutdown_btn.setStyleSheet(_button_style('shutdown'))
        shutdown_btn.clicked.connect(lambda: self.node.send_state('SHUTDOWN'))
        cancel_btn = QPushButton('Cancel Goal')
        cancel_btn.setStyleSheet(_button_style('normal'))
        cancel_btn.clicked.connect(self._on_cancel)
        safe_spot_btn = QPushButton('Move to Safe Spot')
        safe_spot_btn.setStyleSheet(_button_style('safety'))
        safe_spot_btn.setToolTip('Back away, then drive to a clear spot at a different angle')
        safe_spot_btn.clicked.connect(self._on_move_to_safe_spot)
        top_row.addWidget(boot_btn)
        top_row.addWidget(shutdown_btn)
        top_row.addWidget(cancel_btn)
        top_row.addWidget(safe_spot_btn)
        layout.addLayout(top_row)

        mode_row = QHBoxLayout()
        for label, cmd in (('Idle', 'IDLE'), ('Balance', 'BALANCE'),
                           ('Patinha', 'PATINHA'), ('Rebolar', 'REBOLAR')):
            btn = QPushButton(label)
            btn.setStyleSheet(_button_style('normal'))
            btn.clicked.connect(lambda _, c=cmd: self.node.send_state(c))
            mode_row.addWidget(btn)
        layout.addLayout(mode_row)

        self.canvas = MapCanvas()
        self.canvas.on_goal = self._on_goal
        layout.addWidget(self.canvas, stretch=1)

        info_box = QGroupBox('Telemetry')
        info_grid = QGridLayout(info_box)

        self.pos_value = QLabel('--')
        self.nav_mode_value = QLabel('--')
        self.gait_speed_value = QLabel('--')
        self.tilt_value = QLabel('--')
        self.failsafe_value = QLabel('--')
        self.distance_value = QLabel('--')
        self.elapsed_value = QLabel('--')
        self.link_value = QLabel('--')
        self.corridor_value = QLabel('--')
        self.strafe_value = QLabel('--')
        self.clearance_value = QLabel('--')

        rows = [
            ('Robot pose (x, y, yaw):', self.pos_value),
            ('Nav mode:', self.nav_mode_value),
            ('Gait speed:', self.gait_speed_value),
            ('Tilt (roll, pitch):', self.tilt_value),
            ('Failsafe:', self.failsafe_value),
            ('Distance remaining:', self.distance_value),
            ('Elapsed nav time:', self.elapsed_value),
            ('Hexapod link:', self.link_value),
            ('Corridor align:', self.corridor_value),
            ('Strafe escape:', self.strafe_value),
            ('Obstacle clearance:', self.clearance_value),
        ]
        for r, (caption, value_label) in enumerate(rows):
            info_grid.addWidget(QLabel(caption), r, 0)
            info_grid.addWidget(value_label, r, 1)
        layout.addWidget(info_box)

        self.status_label = QLabel(
            'Click a point on the map to set a destination, drag to set heading.')
        layout.addWidget(self.status_label)

        self.setCentralWidget(central)
        self.resize(760, 940)

        self.suppress_plan = False

        self.node.on_map = self.canvas.set_map
        self.node.on_status = self._on_nav_status
        self.node.on_plan = self._on_plan
        self.node.on_result = self._on_nav_result
        self.node.on_hexapod_state = self._on_hexapod_state
        self.node.on_distance = self._on_distance
        self.node.on_safe_retry = self._on_safe_retry

    def _on_hexapod_state(self, data):
        state = data.get('state', 'UNKNOWN')
        color = STATE_COLORS.get(state, STATE_UNKNOWN_COLOR)
        self.hexapod_badge.setText(f'HEXAPOD: {state}')
        self.hexapod_badge.setStyleSheet(_badge_style(color))

        self.nav_mode_value.setText(str(data.get('nav_mode', '--')))
        self.gait_speed_value.setText(f"{data.get('gait_speed', 0.0):.2f}x")

        roll = data.get('roll_deg', 0.0)
        pitch = data.get('pitch_deg', 0.0)
        self.tilt_value.setText(f'{roll:.1f}\u00b0, {pitch:.1f}\u00b0')

        failsafe = data.get('failsafe_active', False)
        self.failsafe_value.setText('ACTIVE' if failsafe else 'clear')
        self.failsafe_value.setStyleSheet(
            'color: #EF4444; font-weight: bold;' if failsafe else 'color: #22C55E;')

        bias = data.get('corridor_bias_deg', 0.0)
        if abs(bias) > 0.5:
            self.corridor_value.setText(f'{bias:+.1f}\u00b0')
            self.corridor_value.setStyleSheet('color: #F59E0B; font-weight: bold;')
        else:
            self.corridor_value.setText('inactive')
            self.corridor_value.setStyleSheet('color: #9CA3AF;')

        self.link_value.setText('connected')
        self.link_value.setStyleSheet('color: #22C55E;')

        clearance = data.get('min_obstacle_dist_m')
        self.canvas.set_min_obstacle_dist(clearance)
        if clearance is None:
            self.clearance_value.setText('--')
            self.clearance_value.setStyleSheet('color: #9CA3AF;')
        else:
            self.clearance_value.setText(f'{clearance:.2f} m')
            if clearance <= CLEARANCE_DANGER_M:
                self.clearance_value.setStyleSheet('color: #EF4444; font-weight: bold;')
            elif clearance >= CLEARANCE_WARN_M:
                self.clearance_value.setStyleSheet('color: #22C55E;')
            else:
                self.clearance_value.setStyleSheet('color: #F59E0B; font-weight: bold;')

        strafing = data.get('strafe_active', False)
        self.strafe_value.setText('ACTIVE' if strafing else 'inactive')
        self.strafe_value.setStyleSheet(
            'color: #F59E0B; font-weight: bold;' if strafing else 'color: #9CA3AF;')

    def _on_nav_status(self, key, text):
        color = NAV_STATUS_COLORS.get(key, NAV_STATUS_COLORS['idle'])
        self.nav_badge.setText(f'NAV2: {text.upper()}')
        self.nav_badge.setStyleSheet(_badge_style(color))
        self.status_label.setText(text)

    def _on_distance(self, remaining, elapsed):
        self.distance_value.setText('--' if remaining is None else f'{remaining:.2f} m')
        self.elapsed_value.setText('--' if elapsed is None else f'{elapsed:.1f} s')

    def _on_plan(self, poses):
        if not self.suppress_plan:
            self.canvas.set_plan(poses)

    def _on_nav_result(self):
        self.suppress_plan = True
        self.canvas.clear_plan()

    def _on_cancel(self):
        self.node.cancel_goal()
        self.suppress_plan = True
        self.canvas.clear_plan()
        self.canvas.goal_point = None
        self.canvas.update()
        self.distance_value.setText('--')
        self.elapsed_value.setText('--')
        self._on_nav_status('canceled', 'Canceled')

    def _on_goal(self, x, y, yaw):
        self.suppress_plan = False
        self.node.send_goal(x, y, yaw)

    def _on_safe_retry(self, x, y):
        self.canvas.goal_point = (x, y)
        self.canvas.update()

    def _on_move_to_safe_spot(self):
        self.suppress_plan = False
        self.node.recover_and_go_safe()

    def refresh_pose(self):
        pose = self.node.current_pose()
        if pose is not None:
            self.canvas.set_robot_pose(pose)
            self.pos_value.setText(
                f'{pose[0]:.2f}, {pose[1]:.2f}, {math.degrees(pose[2]):.1f}\u00b0')

    def refresh_link_status(self):
        last = self.node.last_feedback_time
        if last is None or (time.monotonic() - last) > 1.0:
            self.hexapod_badge.setText('HEXAPOD: NO DATA')
            self.hexapod_badge.setStyleSheet(_badge_style(STATE_UNKNOWN_COLOR))
            self.link_value.setText('no link')
            self.link_value.setStyleSheet('color: #EF4444; font-weight: bold;')


def main(args=None):
    rclpy.init(args=args)
    node = NavGoalNode()

    app = QApplication(sys.argv)
    app.setQuitOnLastWindowClosed(True)

    window = NavGoalWindow(node)
    window.show()

    def handle_sigint(*_):
        app.quit()

    signal.signal(signal.SIGINT, handle_sigint)

    spin_timer = QTimer()
    spin_timer.timeout.connect(lambda: rclpy.spin_once(node, timeout_sec=0.0))
    spin_timer.start(20)

    pose_timer = QTimer()
    pose_timer.timeout.connect(window.refresh_pose)
    pose_timer.start(100)

    link_timer = QTimer()
    link_timer.timeout.connect(window.refresh_link_status)
    link_timer.start(500)

    app.exec_()

    window.close()
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()