"""
teleop_gui_node.py
PySide6 GUI for teleoperation system status and control.

Shows:
  - Node online/offline status
  - Pedal A/B/C live state
  - Manus calibration panel (recalibrate button + phase progress)
"""

import sys
import threading

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_srvs.srv import Trigger

from PySide6.QtWidgets import (
    QApplication, QWidget, QVBoxLayout, QHBoxLayout,
    QGroupBox, QLabel, QPushButton, QProgressBar, QGridLayout,
)
from PySide6.QtCore import Qt, QTimer, Signal, QObject
from PySide6.QtGui import QFont

CALIB_DURATION = 4.0  # seconds per phase — must match manus_inspire

NODES_TO_WATCH = [
    ('pedal_node',           'pedal_ros2'),
    ('vive_tracker_node',    'vive_ros2'),
    ('manus_data_publisher', 'manus_ros2'),
    ('vive_rby1_node',       'vive_rby1'),
    ('manus_inspire',        'manus_inspire'),
]


# ---------------------------------------------------------------------------
# ROS2 node (runs in background thread)
# ---------------------------------------------------------------------------

class TeleopGuiNode(Node):

    def __init__(self):
        super().__init__('teleop_gui')
        self._pedal_state = [0, 0, 0]
        self._pedal_cbs = []
        self._node_status_cbs = []

        self.create_subscription(Joy, '/teleop/pedal', self._cb_pedal, 10)
        self.create_timer(1.0, self._poll_nodes)
        self._calib_client = self.create_client(Trigger, '/manus_inspire/calibrate')

    def _cb_pedal(self, msg):
        state = list(msg.buttons[:3]) + [0] * max(0, 3 - len(msg.buttons))
        self._pedal_state = state[:3]
        for cb in self._pedal_cbs:
            cb(self._pedal_state)

    def _poll_nodes(self):
        names = {n for n, _ in self.get_node_names_and_namespaces()}
        status = {node: (node in names) for node, _ in NODES_TO_WATCH}
        for cb in self._node_status_cbs:
            cb(status)

    def call_calibrate(self, done_cb):
        if not self._calib_client.wait_for_service(timeout_sec=1.0):
            done_cb(False, 'Service not available')
            return
        future = self._calib_client.call_async(Trigger.Request())
        future.add_done_callback(
            lambda f: done_cb(f.result().success, f.result().message))


# ---------------------------------------------------------------------------
# Qt signals bridge (thread-safe)
# ---------------------------------------------------------------------------

class Signals(QObject):
    pedal_updated       = Signal(list)
    node_status_updated = Signal(dict)
    calib_status        = Signal(str)


# ---------------------------------------------------------------------------
# Main window
# ---------------------------------------------------------------------------

class TeleopGuiWindow(QWidget):

    def __init__(self, ros_node: TeleopGuiNode, signals: Signals):
        super().__init__()
        self._node = ros_node
        self._sig  = signals

        signals.pedal_updated.connect(self._on_pedal)
        signals.node_status_updated.connect(self._on_nodes)
        signals.calib_status.connect(self._on_calib_status)

        self._build_ui()

    # ------------------------------------------------------------------
    # UI construction
    # ------------------------------------------------------------------

    def _build_ui(self):
        self.setWindowTitle('Teleop Control')
        self.setMinimumWidth(340)

        root = QVBoxLayout()
        root.setSpacing(8)

        root.addWidget(self._build_node_panel())
        root.addWidget(self._build_pedal_panel())
        root.addWidget(self._build_calib_panel())

        self.setLayout(root)

    def _build_node_panel(self):
        group = QGroupBox('Node Status')
        grid  = QGridLayout()
        grid.setSpacing(4)

        self._node_dots = {}
        for i, (node, pkg) in enumerate(NODES_TO_WATCH):
            dot = QLabel('●')
            dot.setFont(QFont('Monospace', 14))
            dot.setStyleSheet('color: #888;')
            grid.addWidget(dot,            i, 0, Qt.AlignCenter)
            grid.addWidget(QLabel(f'{pkg} / {node}'), i, 1)
            self._node_dots[node] = dot

        group.setLayout(grid)
        return group

    def _build_pedal_panel(self):
        group  = QGroupBox('Pedal')
        layout = QHBoxLayout()

        self._pedal_btns = []
        for label in ['Engage', 'Record', 'Spare']:
            btn = QPushButton(label)
            btn.setEnabled(False)
            btn.setFixedHeight(36)
            btn.setStyleSheet('background-color: #ccc; color: #444;')
            layout.addWidget(btn)
            self._pedal_btns.append(btn)

        group.setLayout(layout)
        return group

    def _build_calib_panel(self):
        group  = QGroupBox('Manus Calibration')
        layout = QVBoxLayout()

        self._calib_label = QLabel('Status: READY')
        self._calib_bar   = QProgressBar()
        self._calib_bar.setRange(0, 100)
        self._calib_bar.setValue(0)
        self._calib_bar.setVisible(False)
        self._calib_btn   = QPushButton('Recalibrate')
        self._calib_btn.setFixedHeight(36)
        self._calib_btn.clicked.connect(self._on_recalibrate)

        layout.addWidget(self._calib_label)
        layout.addWidget(self._calib_bar)
        layout.addWidget(self._calib_btn)
        group.setLayout(layout)
        return group

    # ------------------------------------------------------------------
    # Signal handlers
    # ------------------------------------------------------------------

    def _on_pedal(self, state):
        for btn, pressed in zip(self._pedal_btns, state):
            color = '#A6D256' if pressed else '#ccc'
            btn.setStyleSheet(f'background-color: {color}; color: #333;')

    def _on_nodes(self, status):
        for node, alive in status.items():
            if node in self._node_dots:
                color = '#A6D256' if alive else '#ED325A'
                self._node_dots[node].setStyleSheet(f'color: {color};')

    def _on_calib_status(self, text):
        self._calib_label.setText(f'Status: {text}')

    # ------------------------------------------------------------------
    # Calibration flow
    # ------------------------------------------------------------------

    def _on_recalibrate(self):
        self._calib_btn.setEnabled(False)
        self._calib_bar.setVisible(True)
        self._calib_bar.setValue(0)
        self._sig.calib_status.emit('Calling service...')

        def done(success, msg):
            if not success:
                self._sig.calib_status.emit(f'FAILED: {msg}')
                self._calib_btn.setEnabled(True)
                return
            self._sig.calib_status.emit('Phase 1/2: Open hands fully...')
            self._calib_elapsed = 0.0
            self._calib_phase   = 1
            self._calib_tick_timer = QTimer()
            self._calib_tick_timer.timeout.connect(self._tick_calib)
            self._calib_tick_timer.start(100)

        threading.Thread(
            target=self._node.call_calibrate, args=(done,), daemon=True
        ).start()

    def _tick_calib(self):
        self._calib_elapsed += 0.1
        done_phases = (self._calib_phase - 1) * CALIB_DURATION
        pct = int((done_phases + self._calib_elapsed) / (CALIB_DURATION * 2) * 100)
        self._calib_bar.setValue(min(pct, 100))

        if self._calib_elapsed >= CALIB_DURATION:
            self._calib_elapsed = 0.0
            if self._calib_phase == 1:
                self._calib_phase = 2
                self._sig.calib_status.emit('Phase 2/2: Close fists fully...')
            else:
                self._calib_tick_timer.stop()
                self._calib_bar.setValue(100)
                self._sig.calib_status.emit('COMPLETE')
                self._calib_btn.setEnabled(True)


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main(args=None):
    rclpy.init(args=args)
    ros_node = TeleopGuiNode()
    signals  = Signals()

    # Wire ROS callbacks → Qt signals (thread-safe)
    ros_node._pedal_cbs.append(       lambda s: signals.pedal_updated.emit(s))
    ros_node._node_status_cbs.append( lambda s: signals.node_status_updated.emit(s))

    # Spin ROS in background thread
    spin_thread = threading.Thread(
        target=rclpy.spin, args=(ros_node,), daemon=True)
    spin_thread.start()

    app    = QApplication(sys.argv)
    window = TeleopGuiWindow(ros_node, signals)
    window.show()

    ret = app.exec()

    ros_node.destroy_node()
    rclpy.shutdown()
    sys.exit(ret)


if __name__ == '__main__':
    main()
