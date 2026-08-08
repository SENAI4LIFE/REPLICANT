#!/usr/bin/env python3
import math
import signal
import sys
import json

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String

from PyQt5.QtCore import Qt, QPointF, QTimer
from PyQt5.QtGui import QBrush, QColor, QPainter, QPen
from PyQt5.QtWidgets import (
    QApplication,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QMainWindow,
    QPushButton,
    QSlider,
    QVBoxLayout,
    QWidget,
)


POSE_MAX = 15.0

NAV_MODE_LABELS = {
    'OMNI_1': 'Omni 1',
    'OMNI_2': 'Omni 2',
    'TURN_1': 'Turn 1',
    'TURN_2': 'Turn 2',
}

NAV2_STATUS_LABELS = {
    'idle': 'Idle',
    'sending': 'Navigating',
    'navigating': 'Navigating',
    'succeeded': 'Goal reached',
    'canceled': 'Cancelled',
    'rejected': 'Idle',
    'aborted': 'Idle',
    'unavailable': 'Idle',
}

ROBOT_STATE_COLORS = {
    'Booted': '#22C55E',
    'Shutdown': '#6B7280',
}

CONTROL_SOURCE_COLORS = {
    'Manual': '#F59E0B',
    'Nav2': '#3B82F6',
    'SLAM': '#22C55E',
}

NAV2_STATUS_COLORS = {
    'idle': '#374151',
    'sending': '#F59E0B',
    'navigating': '#3B82F6',
    'succeeded': '#22C55E',
    'canceled': '#F97316',
    'aborted': '#EF4444',
    'rejected': '#EF4444',
    'unavailable': '#EF4444',
}

SAFE_MODE_COLORS = {
    True: '#EF4444',
    False: '#6B7280',
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

_KB_MOVE_STYLE = (
    'QPushButton {background-color: #3A3A3A; color: #EAEAEA; '
    'border: 1px solid #555555; border-radius: 6px;}'
    'QPushButton:hover {background-color: #454545;}'
    'QPushButton:pressed {background-color: #2A2A2A;}'
)

_KB_DIAGONAL_STYLE = (
    'QPushButton {background-color: #33415C; color: #DCE6FF; '
    'border: 1px solid #4C6284; border-radius: 6px;}'
    'QPushButton:hover {background-color: #3D4E70;}'
    'QPushButton:pressed {background-color: #283349;}'
    'QPushButton:disabled {background-color: #262626; color: #6B7280; '
    'border: 1px solid #3A3A3A;}'
)

_KB_STOP_STYLE = (
    'QPushButton {background-color: #7A1F1F; color: #FFE0E0; '
    'border: 1px solid #EF4444; border-radius: 6px; font-weight: bold;}'
    'QPushButton:hover {background-color: #8E2626;}'
    'QPushButton:pressed {background-color: #5E1818;}'
)

DIAGONAL_MODES = ('OMNI_2', 'TURN_2')


class JoystickPad(QWidget):

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumSize(260, 260)
        self._handle = QPointF(0.0, 0.0)
        self._dragging = False
        self.on_move = None
        self.on_press = None
        self.on_release = None

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        side = min(self.width(), self.height())
        cx, cy = self.width() / 2.0, self.height() / 2.0
        radius = side / 2.0 - 12

        painter.setBrush(QBrush(QColor(45, 45, 45)))
        painter.setPen(QPen(QColor(90, 90, 90), 2))
        painter.drawEllipse(QPointF(cx, cy), radius, radius)

        painter.setPen(QPen(QColor(100, 100, 100), 1, Qt.DashLine))
        painter.drawLine(int(cx - radius), int(cy), int(cx + radius), int(cy))
        painter.drawLine(int(cx), int(cy - radius), int(cx), int(cy + radius))

        hx = cx + self._handle.x() * radius
        hy = cy + self._handle.y() * radius
        painter.setBrush(QBrush(QColor(90, 170, 255)))
        painter.setPen(QPen(QColor(210, 230, 255), 2))
        painter.drawEllipse(QPointF(hx, hy), 22, 22)

    def mousePressEvent(self, event):
        if event.button() == Qt.LeftButton:
            self._dragging = True
            if self.on_press:
                self.on_press()
            self._update_handle(event.pos())

    def mouseMoveEvent(self, event):
        if self._dragging:
            self._update_handle(event.pos())

    def mouseReleaseEvent(self, event):
        if event.button() == Qt.LeftButton:
            self._dragging = False
            self._handle = QPointF(0.0, 0.0)
            self.update()
            if self.on_release:
                self.on_release()

    def _update_handle(self, pos):
        side = min(self.width(), self.height())
        cx, cy = self.width() / 2.0, self.height() / 2.0
        radius = side / 2.0 - 12

        dx = (pos.x() - cx) / radius
        dy = (pos.y() - cy) / radius
        dist = math.hypot(dx, dy)
        if dist > 1.0:
            dx /= dist
            dy /= dist

        self._handle = QPointF(dx, dy)
        self.update()
        if self.on_move:
            self.on_move(-dy, -dx)


class JoystickNode(Node):

    def __init__(self):
        super().__init__('joystick_hexapod')
        self.vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.state_pub = self.create_publisher(String, '/tiffany/state', 10)
        self.feedback_sub = self.create_subscription(
            String, '/tiffany/state_feedback', self._feedback_cb, 10)
        self.nav2_status_sub = self.create_subscription(
            String, '/tiffany/nav2_status', self._nav2_status_cb, 10)
        self.nav_mode = 'OMNI_2'
        self.confirmed_nav_mode = None
        self.robot_ready = False
        self.nav2_status = None
        self.on_feedback = None
        self.on_nav2_status = None
        self.current_lx = 0.0
        self.current_ly = 0.0
        self.current_az = 0.0

    def _feedback_cb(self, msg):
        try:
            data = json.loads(msg.data)
        except (json.JSONDecodeError, TypeError):
            return
        mode = data.get('nav_mode')
        if mode in ('OMNI_1', 'OMNI_2', 'TURN_1', 'TURN_2'):
            self.confirmed_nav_mode = mode
        self.robot_ready = bool(data.get('ready', False))
        if self.on_feedback:
            self.on_feedback(data)

    def _nav2_status_cb(self, msg):
        self.nav2_status = msg.data
        if self.on_nav2_status:
            self.on_nav2_status(msg.data)

    def send_state(self, state):
        msg = String()
        msg.data = state
        self.state_pub.publish(msg)

    def set_target_velocity(self, lx=0.0, ly=0.0, az=0.0):
        self.current_lx = lx
        self.current_ly = ly
        self.current_az = az

    def publish_velocity(self):
        msg = Twist()
        msg.linear.x = self.current_lx
        msg.linear.y = self.current_ly
        msg.angular.z = self.current_az
        self.vel_pub.publish(msg)

    def send_velocity(self, lx=0.0, ly=0.0, az=0.0):
        self.set_target_velocity(lx, ly, az)
        self.publish_velocity()

    def send_pose(self, roll, pitch):
        self.send_state(f'POSE {roll:.1f} {pitch:.1f}')

    def set_nav_mode(self, mode):
        self.nav_mode = mode
        self.send_state(f'NAV_{mode}')
        return mode

    def set_manual_active(self, active):
        self.send_state('MANUAL_ON' if active else 'MANUAL_OFF')

    def set_safe_mode(self, active):
        self.send_state('SAFE_ON' if active else 'SAFE_OFF')


class JoystickWindow(QMainWindow):

    def __init__(self, node):
        super().__init__()
        self.node = node
        self.pose_mode = False
        self.manual_active = False
        self.safe_mode = False
        self.setWindowTitle('Tiffany Virtual Joystick')
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

        status_box = QGroupBox('Status')
        status_row = QHBoxLayout(status_box)
        self.status_labels = {}
        for key, title in (
            ('robot_state', 'Robot'),
            ('nav_mode', 'Mode'),
            ('control_source', 'Control'),
            ('safe_mode', 'Safe Mode'),
            ('nav2_status', 'Nav2'),
        ):
            label = QLabel(f'{title}: --')
            label.setStyleSheet(_badge_style('#374151'))
            self.status_labels[key] = label
            status_row.addWidget(label)
        status_row.addStretch(1)
        layout.addWidget(status_box)

        controls_box = QGroupBox('Robot Controls')
        top_row = QHBoxLayout(controls_box)
        boot_btn = QPushButton('Boot')
        boot_btn.setStyleSheet(_button_style('boot'))
        boot_btn.clicked.connect(self._on_boot)
        shutdown_btn = QPushButton('Shutdown')
        shutdown_btn.setStyleSheet(_button_style('shutdown'))
        shutdown_btn.clicked.connect(self._on_shutdown)
        self.pose_btn = QPushButton('Pose mode: OFF')
        self.pose_btn.setStyleSheet(_button_style('normal'))
        self.pose_btn.clicked.connect(self._toggle_pose)
        self.safe_btn = QPushButton('Safe Mode: OFF')
        self.safe_btn.setStyleSheet(_button_style('safety'))
        self.safe_btn.clicked.connect(self._toggle_safe_mode)
        top_row.addWidget(boot_btn)
        top_row.addWidget(shutdown_btn)
        top_row.addWidget(self.pose_btn)
        top_row.addWidget(self.safe_btn)
        layout.addWidget(controls_box)

        nav_box = QGroupBox('Navigation Mode')
        nav_row = QHBoxLayout(nav_box)
        self.nav_buttons = {}
        for mode, label in (
            ('OMNI_1', 'Omni 1'),
            ('OMNI_2', 'Omni 2'),
            ('TURN_1', 'Turn 1'),
            ('TURN_2', 'Turn 2'),
        ):
            btn = QPushButton(label)
            btn.setCheckable(True)
            btn.setStyleSheet(_NAV_BUTTON_STYLE)
            btn.clicked.connect(lambda _checked, m=mode: self._set_nav_mode(m))
            nav_row.addWidget(btn)
            self.nav_buttons[mode] = btn
        self.nav_buttons['OMNI_2'].setChecked(True)
        layout.addWidget(nav_box)

        speed_box = QGroupBox('Speed')
        speed_row = QHBoxLayout(speed_box)
        speed_row.addWidget(QLabel('Max speed:'))
        self.speed_slider = QSlider(Qt.Horizontal)
        self.speed_slider.setRange(10, 100)
        self.speed_slider.setValue(50)
        self.speed_label = QLabel('50%')
        self.speed_label.setStyleSheet('color: #93C5FD; font-weight: bold;')
        self.speed_slider.valueChanged.connect(
            lambda v: self.speed_label.setText(f'{v}%'))
        speed_row.addWidget(self.speed_slider)
        speed_row.addWidget(self.speed_label)
        layout.addWidget(speed_box)

        joystick_box = QGroupBox('Virtual Joystick')
        joystick_layout = QVBoxLayout(joystick_box)
        self.pad = JoystickPad()
        self.pad.on_press = self._on_press
        self.pad.on_move = self._on_move
        self.pad.on_release = self._on_release
        joystick_layout.addWidget(self.pad, stretch=1)

        self.values_label = QLabel('lx=0.00  ly=0.00  az=0.00')
        self.values_label.setAlignment(Qt.AlignCenter)
        self.values_label.setStyleSheet('color: #93C5FD; font-family: monospace;')
        joystick_layout.addWidget(self.values_label)
        layout.addWidget(joystick_box, stretch=1)

        keyboard_box = QGroupBox('Keyboard Controls')
        keyboard_layout = QVBoxLayout(keyboard_box)
        keyboard_layout.setSpacing(6)

        grid_wrap = QHBoxLayout()
        keyboard_grid = QGridLayout()
        keyboard_grid.setSpacing(4)

        SQUARE = 52

        self.kb_nw_btn = QPushButton('\u2196\nQ')
        self.kb_forward_btn = QPushButton('\u2191\nW')
        self.kb_ne_btn = QPushButton('\u2197\nE')
        self.kb_left_btn = QPushButton('\u2190\nA')
        self.kb_stop_btn = QPushButton('\u25A0\nSPACE')
        self.kb_right_btn = QPushButton('\u2192\nD')
        self.kb_sw_btn = QPushButton('\u2199\nZ')
        self.kb_backward_btn = QPushButton('\u2193\nS')
        self.kb_se_btn = QPushButton('\u2198\nC')

        self.diagonal_buttons = {
            'NW': self.kb_nw_btn,
            'NE': self.kb_ne_btn,
            'SW': self.kb_sw_btn,
            'SE': self.kb_se_btn,
        }

        for btn in (self.kb_forward_btn, self.kb_left_btn, self.kb_right_btn,
                    self.kb_backward_btn):
            btn.setStyleSheet(_KB_MOVE_STYLE)
            btn.setFixedSize(SQUARE, SQUARE)

        for btn in self.diagonal_buttons.values():
            btn.setStyleSheet(_KB_DIAGONAL_STYLE)
            btn.setFixedSize(SQUARE, SQUARE)

        self.kb_stop_btn.setStyleSheet(_KB_STOP_STYLE)
        self.kb_stop_btn.setFixedSize(SQUARE, SQUARE)

        self.kb_forward_btn.pressed.connect(lambda: self._on_kb_direction(1.0, 0.0))
        self.kb_forward_btn.released.connect(self._on_kb_release)
        self.kb_backward_btn.pressed.connect(lambda: self._on_kb_direction(-1.0, 0.0))
        self.kb_backward_btn.released.connect(self._on_kb_release)
        self.kb_left_btn.pressed.connect(lambda: self._on_kb_direction(0.0, 1.0))
        self.kb_left_btn.released.connect(self._on_kb_release)
        self.kb_right_btn.pressed.connect(lambda: self._on_kb_direction(0.0, -1.0))
        self.kb_right_btn.released.connect(self._on_kb_release)
        self.kb_stop_btn.clicked.connect(self._on_kb_stop)

        self.kb_nw_btn.pressed.connect(lambda: self._on_kb_diagonal('NW'))
        self.kb_nw_btn.released.connect(self._on_kb_release)
        self.kb_ne_btn.pressed.connect(lambda: self._on_kb_diagonal('NE'))
        self.kb_ne_btn.released.connect(self._on_kb_release)
        self.kb_sw_btn.pressed.connect(lambda: self._on_kb_diagonal('SW'))
        self.kb_sw_btn.released.connect(self._on_kb_release)
        self.kb_se_btn.pressed.connect(lambda: self._on_kb_diagonal('SE'))
        self.kb_se_btn.released.connect(self._on_kb_release)

        keyboard_grid.addWidget(self.kb_nw_btn, 0, 0)
        keyboard_grid.addWidget(self.kb_forward_btn, 0, 1)
        keyboard_grid.addWidget(self.kb_ne_btn, 0, 2)
        keyboard_grid.addWidget(self.kb_left_btn, 1, 0)
        keyboard_grid.addWidget(self.kb_stop_btn, 1, 1)
        keyboard_grid.addWidget(self.kb_right_btn, 1, 2)
        keyboard_grid.addWidget(self.kb_sw_btn, 2, 0)
        keyboard_grid.addWidget(self.kb_backward_btn, 2, 1)
        keyboard_grid.addWidget(self.kb_se_btn, 2, 2)

        grid_wrap.addStretch(1)
        grid_wrap.addLayout(keyboard_grid)
        grid_wrap.addStretch(1)
        keyboard_layout.addLayout(grid_wrap)

        balance_row = QHBoxLayout()
        self.kb_balance_btn = QPushButton('Balance  (B)')
        self.kb_balance_btn.setStyleSheet(_button_style('normal'))
        self.kb_balance_btn.setMinimumHeight(36)
        self.kb_balance_btn.clicked.connect(self._on_balance)
        balance_row.addWidget(self.kb_balance_btn)
        keyboard_layout.addLayout(balance_row)

        layout.addWidget(keyboard_box)

        anim_box = QGroupBox('Animations')
        anim_row = QHBoxLayout(anim_box)
        rebolar_btn = QPushButton('Rebolar')
        rebolar_btn.setStyleSheet(_button_style('normal'))
        rebolar_btn.clicked.connect(self._on_rebolar)
        patinha_btn = QPushButton('Patinha')
        patinha_btn.setStyleSheet(_button_style('normal'))
        patinha_btn.clicked.connect(self._on_patinha)
        anim_row.addWidget(rebolar_btn)
        anim_row.addWidget(patinha_btn)
        layout.addWidget(anim_box)

        self.setCentralWidget(central)
        self.resize(440, 760)

        self.node.on_feedback = self._on_feedback
        self.node.on_nav2_status = self._on_nav2_status
        self._refresh_status()
        self.setFocusPolicy(Qt.StrongFocus)
        self.setFocus()

    def keyPressEvent(self, event):
        if event.isAutoRepeat():
            return
        key = event.key()
        directions = {
            Qt.Key_W: (1.0, 0.0),
            Qt.Key_S: (-1.0, 0.0),
            Qt.Key_A: (0.0, 1.0),
            Qt.Key_D: (0.0, -1.0),
        }
        diagonals = {
            Qt.Key_Q: 'NW',
            Qt.Key_E: 'NE',
            Qt.Key_Z: 'SW',
            Qt.Key_C: 'SE',
        }
        if key in directions:
            self._on_kb_direction(*directions[key])
            event.accept()
            return
        if key in diagonals:
            self._on_kb_diagonal(diagonals[key])
            event.accept()
            return
        if key == Qt.Key_Space:
            self._on_kb_stop()
            event.accept()
            return
        if key == Qt.Key_B:
            self._on_balance()
            event.accept()
            return
        super().keyPressEvent(event)

    def keyReleaseEvent(self, event):
        if event.isAutoRepeat():
            return
        if event.key() in (
            Qt.Key_W, Qt.Key_S, Qt.Key_A, Qt.Key_D,
            Qt.Key_Q, Qt.Key_E, Qt.Key_Z, Qt.Key_C,
            Qt.Key_Space,
        ):
            self._on_kb_release()
            event.accept()
            return
        super().keyReleaseEvent(event)

    def _on_boot(self):
        self.node.send_velocity(0.0, 0.0, 0.0)
        self.node.send_state('BOOT')
        self._set_nav_mode(self.node.nav_mode)

    def _on_shutdown(self):
        self.node.send_velocity(0.0, 0.0, 0.0)
        self.node.send_state('SHUTDOWN')

    def _on_rebolar(self):
        self.node.send_velocity(0.0, 0.0, 0.0)
        self.node.send_state('REBOLAR')

    def _on_patinha(self):
        self.node.send_velocity(0.0, 0.0, 0.0)
        self.node.send_state('PATINHA')

    def _on_balance(self):
        self.node.send_velocity(0.0, 0.0, 0.0)
        self.node.send_state('BALANCE')

    def _set_nav_mode(self, mode):
        self.node.send_velocity(0.0, 0.0, 0.0)
        self.node.set_nav_mode(mode)
        for m, btn in self.nav_buttons.items():
            btn.setChecked(m == mode)
        self._refresh_status()

    def _toggle_pose(self):
        self.pose_mode = not self.pose_mode
        if self.pose_mode:
            self.node.send_velocity(0.0, 0.0, 0.0)
            self.pose_btn.setText('Pose mode: ON')
        else:
            self.node.send_state('IDLE')
            self.pose_btn.setText('Pose mode: OFF')

    def _toggle_safe_mode(self):
        self.safe_mode = not self.safe_mode
        self.node.set_safe_mode(self.safe_mode)
        self.safe_btn.setText(f'Safe Mode: {"ON" if self.safe_mode else "OFF"}')
        if not self.safe_mode:
            self.node.send_state('SAFE_OFF')
        self._refresh_status()

    def _on_press(self):
        self.manual_active = True
        self.node.set_manual_active(True)
        self._refresh_status()

    def _on_feedback(self, data):
        self._refresh_status()

    def _on_nav2_status(self, status):
        self._refresh_status()

    def _refresh_status(self):
        robot_state = 'Booted' if self.node.robot_ready else 'Shutdown'
        if not self.node.robot_ready:
            self.safe_mode = False
        mode = (self.node.confirmed_nav_mode or self.node.nav_mode) if self.node.robot_ready else None
        if self.manual_active:
            control_source = 'Manual'
        elif self.node.nav2_status in ('sending', 'navigating'):
            control_source = 'Nav2'
        else:
            control_source = 'Idle'
        nav2_key = self.node.nav2_status
        nav2_status = NAV2_STATUS_LABELS.get(nav2_key, 'Idle')

        self.status_labels['robot_state'].setText(f'Robot: {robot_state}')
        self.status_labels['robot_state'].setStyleSheet(
            _badge_style(ROBOT_STATE_COLORS.get(robot_state, '#374151')))

        self.status_labels['nav_mode'].setText(
            f'Mode: {NAV_MODE_LABELS.get(mode, "OFF") if mode is not None else "OFF"}')
        self.status_labels['nav_mode'].setStyleSheet(_badge_style('#3B82F6'))

        self.status_labels['control_source'].setText(f'Control: {control_source}')
        self.status_labels['control_source'].setStyleSheet(
            _badge_style(CONTROL_SOURCE_COLORS.get(control_source, '#374151')))

        self.status_labels['safe_mode'].setText(
            f'Safe Mode: {"ON" if self.safe_mode else "OFF"}')
        self.status_labels['safe_mode'].setStyleSheet(
            _badge_style(SAFE_MODE_COLORS[self.safe_mode]))

        self.status_labels['nav2_status'].setText(f'Nav2: {nav2_status}')
        self.status_labels['nav2_status'].setStyleSheet(
            _badge_style(NAV2_STATUS_COLORS.get(nav2_key, NAV2_STATUS_COLORS['idle'])))

        self._update_diagonal_availability()

    def _current_nav_mode(self):
        return self.node.confirmed_nav_mode or self.node.nav_mode

    def _update_diagonal_availability(self):
        enabled = self._current_nav_mode() in DIAGONAL_MODES
        for btn in self.diagonal_buttons.values():
            btn.setEnabled(enabled)

    def _on_move(self, linear, angular):
        if self.pose_mode:
            roll = -angular * POSE_MAX
            pitch = -linear * POSE_MAX
            self.node.send_pose(roll, pitch)
            self.values_label.setText(f'roll={roll:.1f}°  pitch={pitch:.1f}°')
            return

        scale = self.speed_slider.value() / 100.0 * 0.3
        angular_scale = self.speed_slider.value() / 100.0 * 2.0

        mode = self.node.confirmed_nav_mode or self.node.nav_mode
        if mode in ('OMNI_1', 'OMNI_2'):
            lx = linear * scale
            ly = angular * scale
            self.node.set_target_velocity(lx=lx, ly=ly)
            self.values_label.setText(f'lx={lx:.2f}  ly={ly:.2f}')
        elif mode == 'TURN_1':
            lx = linear * scale
            az = angular * angular_scale
            self.node.set_target_velocity(lx=lx, az=az)
            self.values_label.setText(f'lx={lx:.2f}  az={az:.2f}')
        elif mode == 'TURN_2':
            lx = linear * scale
            ly = angular * scale
            az = angular * angular_scale
            self.node.set_target_velocity(lx=lx, ly=ly, az=az)
            self.values_label.setText(f'lx={lx:.2f}  ly={ly:.2f}  az={az:.2f}')
        else:
            lx = linear * scale
            az = angular * angular_scale
            if abs(lx) >= abs(az):
                az = 0.0
            else:
                lx = 0.0
            self.node.set_target_velocity(lx=lx, az=az)
            self.values_label.setText(f'lx={lx:.2f}  az={az:.2f}')

    def _on_release(self):
        self.manual_active = False
        self.node.set_manual_active(False)
        self._refresh_status()
        if self.pose_mode:
            return
        self.node.send_velocity(0.0, 0.0, 0.0)
        self.values_label.setText('lx=0.00  ly=0.00  az=0.00')

    def _on_kb_direction(self, linear, angular):
        self._on_press()
        self._on_move(linear, angular)

    def _on_kb_release(self):
        self._on_release()

    def _on_kb_stop(self):
        self.node.send_velocity(0.0, 0.0, 0.0)
        self.node.send_state('IDLE')
        self.manual_active = False
        self.node.set_manual_active(False)
        self._refresh_status()
        self.values_label.setText('lx=0.00  ly=0.00  az=0.00')
        self.node.send_state('IDLE')

    def _on_kb_diagonal(self, direction):
        mode = self._current_nav_mode()
        if mode not in DIAGONAL_MODES:
            return

        lx_sign = 1.0 if direction in ('NW', 'NE') else -1.0
        ly_sign = 1.0 if direction in ('NW', 'SW') else -1.0
        az_sign = 1.0 if direction in ('NW', 'SW') else -1.0

        self._on_press()

        if mode in ('OMNI_1', 'OMNI_2'):
            scale = self.speed_slider.value() / 100.0 * 0.3
            lx = lx_sign * scale
            ly = ly_sign * scale
            self.node.set_target_velocity(lx=lx, ly=ly)
            self.values_label.setText(f'lx={lx:.2f}  ly={ly:.2f}')
        elif mode == 'TURN_2':
            lx = lx_sign * 0.15
            ly = ly_sign * 0.02
            az = az_sign * 0.15
            self.node.set_target_velocity(lx=lx, ly=ly, az=az)
            self.values_label.setText(f'lx={lx:.2f}  ly={ly:.2f}  az={az:.2f}')

    def tick(self):
        rclpy.spin_once(self.node, timeout_sec=0.0)
        if self.manual_active and not self.pose_mode:
            self.node.publish_velocity()


def main(args=None):
    rclpy.init(args=args)
    node = JoystickNode()

    app = QApplication(sys.argv)
    app.setQuitOnLastWindowClosed(True)

    window = JoystickWindow(node)
    window.show()

    def handle_sigint(*_):
        app.quit()

    signal.signal(signal.SIGINT, handle_sigint)

    timer = QTimer()
    timer.timeout.connect(window.tick)
    timer.start(20)

    app.exec_()

    window.close()
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()