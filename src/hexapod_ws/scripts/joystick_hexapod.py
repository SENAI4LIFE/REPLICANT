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


class JoystickPad(QWidget):

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumSize(260, 260)
        self._handle = QPointF(0.0, 0.0)
        self._dragging = False
        self.on_move = None
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
        self.nav_mode = 'OMNI'
        self.confirmed_nav_mode = None

    def _feedback_cb(self, msg):
        try:
            data = json.loads(msg.data)
        except (json.JSONDecodeError, TypeError):
            return
        mode = data.get('nav_mode')
        if mode in ('OMNI', 'TURN_1', 'TURN_2'):
            self.confirmed_nav_mode = mode

    def send_state(self, state):
        msg = String()
        msg.data = state
        self.state_pub.publish(msg)

    def send_velocity(self, lx=0.0, ly=0.0, az=0.0):
        msg = Twist()
        msg.linear.x = lx
        msg.linear.y = ly
        msg.angular.z = az
        self.vel_pub.publish(msg)

    def send_pose(self, roll, pitch):
        self.send_state(f'POSE {roll:.1f} {pitch:.1f}')

    def set_nav_mode(self, mode):
        self.nav_mode = mode
        self.send_state(f'NAV_{mode}')
        return mode


class JoystickWindow(QMainWindow):

    def __init__(self, node):
        super().__init__()
        self.node = node
        self.pose_mode = False
        self.setWindowTitle('Tiffany Virtual Joystick')

        central = QWidget()
        layout = QVBoxLayout(central)

        top_row = QHBoxLayout()
        boot_btn = QPushButton('Boot')
        boot_btn.clicked.connect(self._on_boot)
        shutdown_btn = QPushButton('Shutdown')
        shutdown_btn.clicked.connect(lambda: self.node.send_state('SHUTDOWN'))
        self.pose_btn = QPushButton('Pose mode: OFF')
        self.pose_btn.clicked.connect(self._toggle_pose)
        top_row.addWidget(boot_btn)
        top_row.addWidget(shutdown_btn)
        top_row.addWidget(self.pose_btn)
        layout.addLayout(top_row)

        nav_row = QHBoxLayout()
        nav_row.addWidget(QLabel('Nav mode:'))
        self.nav_buttons = {}
        for mode, label in (
            ('OMNI', 'Omni'),
            ('TURN_1', 'Turn 1'),
            ('TURN_2', 'Turn 2'),
        ):
            btn = QPushButton(label)
            btn.setCheckable(True)
            btn.clicked.connect(lambda _checked, m=mode: self._set_nav_mode(m))
            nav_row.addWidget(btn)
            self.nav_buttons[mode] = btn
        self.nav_buttons['OMNI'].setChecked(True)
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
        self.pad.on_move = self._on_move
        self.pad.on_release = self._on_release
        layout.addWidget(self.pad, stretch=1)

        self.values_label = QLabel('lx=0.00  ly=0.00  az=0.00')
        layout.addWidget(self.values_label)

        anim_row = QHBoxLayout()
        rebolar_btn = QPushButton('Rebolar')
        rebolar_btn.clicked.connect(lambda: self.node.send_state('REBOLAR'))
        patinha_btn = QPushButton('Patinha')
        patinha_btn.clicked.connect(lambda: self.node.send_state('PATINHA'))
        anim_row.addWidget(rebolar_btn)
        anim_row.addWidget(patinha_btn)
        layout.addLayout(anim_row)

        self.setCentralWidget(central)
        self.resize(360, 480)

    def _on_boot(self):
        self.node.send_state('BOOT')
        self._set_nav_mode(self.node.nav_mode)

    def _set_nav_mode(self, mode):
        self.node.set_nav_mode(mode)
        for m, btn in self.nav_buttons.items():
            btn.setChecked(m == mode)

    def _toggle_pose(self):
        self.pose_mode = not self.pose_mode
        if self.pose_mode:
            self.node.send_velocity(0.0, 0.0, 0.0)
            self.pose_btn.setText('Pose mode: ON')
        else:
            self.node.send_state('IDLE')
            self.pose_btn.setText('Pose mode: OFF')

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
        if mode == 'OMNI':
            lx = linear * scale
            ly = angular * scale
            self.node.send_velocity(lx=lx, ly=ly)
            self.values_label.setText(f'lx={lx:.2f}  ly={ly:.2f}')
        elif mode == 'TURN_1':
            lx = linear * scale
            az = angular * angular_scale
            self.node.send_velocity(lx=lx, az=az)
            self.values_label.setText(f'lx={lx:.2f}  az={az:.2f}')
        else:
            lx = linear * scale
            az = angular * angular_scale
            self.node.send_velocity(lx=lx, az=az)
            self.values_label.setText(f'lx={lx:.2f}  az={az:.2f}')

    def _on_release(self):
        if self.pose_mode:
            return
        self.node.send_velocity(0.0, 0.0, 0.0)
        self.values_label.setText('lx=0.00  ly=0.00  az=0.00')


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
    timer.timeout.connect(lambda: rclpy.spin_once(node, timeout_sec=0.0))
    timer.start(20)

    app.exec_()

    window.close()
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()