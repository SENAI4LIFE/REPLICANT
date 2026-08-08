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

        central = QWidget()
        layout = QVBoxLayout(central)

        status_row = QHBoxLayout()
        self.status_labels = {}
        for key, title in (
            ('robot_state', 'Robot'),
            ('nav_mode', 'Mode'),
            ('control_source', 'Control'),
            ('safe_mode', 'Safe Mode'),
            ('nav2_status', 'Nav2'),
        ):
            label = QLabel(f'{title}: --')
            self.status_labels[key] = label
            status_row.addWidget(label)
        layout.addLayout(status_row)

        top_row = QHBoxLayout()
        boot_btn = QPushButton('Boot')
        boot_btn.clicked.connect(self._on_boot)
        shutdown_btn = QPushButton('Shutdown')
        shutdown_btn.clicked.connect(self._on_shutdown)
        self.pose_btn = QPushButton('Pose mode: OFF')
        self.pose_btn.clicked.connect(self._toggle_pose)
        self.safe_btn = QPushButton('Safe Mode: OFF')
        self.safe_btn.clicked.connect(self._toggle_safe_mode)
        top_row.addWidget(boot_btn)
        top_row.addWidget(shutdown_btn)
        top_row.addWidget(self.pose_btn)
        top_row.addWidget(self.safe_btn)
        layout.addLayout(top_row)

        nav_row = QHBoxLayout()
        nav_row.addWidget(QLabel('Nav mode:'))
        self.nav_buttons = {}
        for mode, label in (
            ('OMNI_1', 'Omni 1'),
            ('OMNI_2', 'Omni 2'),
            ('TURN_1', 'Turn 1'),
            ('TURN_2', 'Turn 2'),
        ):
            btn = QPushButton(label)
            btn.setCheckable(True)
            btn.clicked.connect(lambda _checked, m=mode: self._set_nav_mode(m))
            nav_row.addWidget(btn)
            self.nav_buttons[mode] = btn
        self.nav_buttons['OMNI_2'].setChecked(True)
        layout.addLayout(nav_row)

        speed_row = QHBoxLayout()
        speed_row.addWidget(QLabel('Max speed:'))
        self.speed_slider = QSlider(Qt.Horizontal)
        self.speed_slider.setRange(10, 100)
        self.speed_slider.setValue(50)
        self.speed_label = QLabel('50%')
        self.speed_slider.valueChanged.connect(
            lambda v: self.speed_label.setText(f'{v}%'))
        speed_row.addWidget(self.speed_slider)
        speed_row.addWidget(self.speed_label)
        layout.addLayout(speed_row)

        self.pad = JoystickPad()
        self.pad.on_press = self._on_press
        self.pad.on_move = self._on_move
        self.pad.on_release = self._on_release
        layout.addWidget(self.pad, stretch=1)

        self.values_label = QLabel('lx=0.00  ly=0.00  az=0.00')
        layout.addWidget(self.values_label)

        anim_row = QHBoxLayout()
        rebolar_btn = QPushButton('Rebolar')
        rebolar_btn.clicked.connect(self._on_rebolar)
        patinha_btn = QPushButton('Patinha')
        patinha_btn.clicked.connect(self._on_patinha)
        anim_row.addWidget(rebolar_btn)
        anim_row.addWidget(patinha_btn)
        layout.addLayout(anim_row)

        self.setCentralWidget(central)
        self.resize(420, 520)

        self.node.on_feedback = self._on_feedback
        self.node.on_nav2_status = self._on_nav2_status
        self._refresh_status()

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
        mode = self.node.confirmed_nav_mode or self.node.nav_mode
        control_source = 'Manual' if self.manual_active else 'Nav2'
        nav2_status = NAV2_STATUS_LABELS.get(self.node.nav2_status, 'Idle')

        self.status_labels['robot_state'].setText(f'Robot: {robot_state}')
        self.status_labels['nav_mode'].setText(
            f'Mode: {NAV_MODE_LABELS.get(mode, mode)}')
        self.status_labels['control_source'].setText(f'Control: {control_source}')
        self.status_labels['safe_mode'].setText(
            f'Safe Mode: {"ON" if self.safe_mode else "OFF"}')
        self.status_labels['nav2_status'].setText(f'Nav2: {nav2_status}')

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

    def tick(self):
        rclpy.spin_once(self.node, timeout_sec=0.0)
        if not self.pose_mode:
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