"""
teleop_gui_node.py
PySide6 GUI for teleoperation system status and control.

Shows:
  - Node online/offline status
  - Pedal A/B/C live state
  - Recording panel (state, task_id dropdown, episode, Start/End button)
  - Manus calibration panel (recalibrate button + phase progress)
"""

import sys
import threading

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Int32, String
from std_srvs.srv import Trigger

from PySide6.QtWidgets import (
    QApplication, QWidget, QVBoxLayout, QHBoxLayout,
    QGroupBox, QLabel, QPushButton, QProgressBar, QGridLayout,
    QRadioButton, QButtonGroup, QSpinBox,
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

REC_STATE_STYLE = {
    'IDLE':      ('⬤ IDLE',      '#888888'),
    'READY':     ('⬤ READY',     '#F0C040'),
    'RECORDING': ('⬤ RECORDING', '#E0302A'),
    'PAUSED':    ('⬤ PAUSED',    '#E08020'),
}


# ---------------------------------------------------------------------------
# ROS2 node (runs in background thread)
# ---------------------------------------------------------------------------

class TeleopGuiNode(Node):

    def __init__(self):
        super().__init__('teleop_gui')
        self._pedal_state = [0, 0, 0]
        self._pedal_cbs       = []
        self._node_status_cbs = []
        self._rec_state_cbs   = []
        self._rec_episode_cbs = []

        self.create_subscription(Joy,    '/teleop/pedal',       self._cb_pedal,       10)
        self.create_subscription(String, '/teleop/rec_state',   self._cb_rec_state,   10)
        self.create_subscription(Int32,  '/teleop/rec_episode', self._cb_rec_episode, 10)
        self.create_timer(1.0, self._poll_nodes)

        self._calib_client        = self.create_client(Trigger,  '/manus_inspire/calibrate')
        self._toggle_ep_client    = self.create_client(Trigger,  '/vive_rby1/toggle_episode')
        self._task_id_pub         = self.create_publisher(Int32,   '/teleop/task_id',      10)
        self._control_mode_pub    = self.create_publisher(String,  '/teleop/control_mode', 10)
        self._rby1_cmd_pub        = self.create_publisher(String,  '/teleop/rby1_command', 10)

    def _cb_pedal(self, msg):
        state = list(msg.buttons[:3]) + [0] * max(0, 3 - len(msg.buttons))
        self._pedal_state = state[:3]
        for cb in self._pedal_cbs:
            cb(self._pedal_state)

    def _cb_rec_state(self, msg: String):
        for cb in self._rec_state_cbs:
            cb(msg.data)

    def _cb_rec_episode(self, msg: Int32):
        for cb in self._rec_episode_cbs:
            cb(msg.data)

    def _poll_nodes(self):
        names  = {n for n, _ in self.get_node_names_and_namespaces()}
        status = {node: (node in names) for node, _ in NODES_TO_WATCH}
        for cb in self._node_status_cbs:
            cb(status)

    def publish_task_id(self, task_id: int):
        self._task_id_pub.publish(Int32(data=task_id))

    def publish_control_mode(self, mode: str):
        self._control_mode_pub.publish(String(data=mode))

    def publish_rby1_command(self, command: str):
        self._rby1_cmd_pub.publish(String(data=command))

    def call_calibrate(self, done_cb):
        if not self._calib_client.wait_for_service(timeout_sec=1.0):
            done_cb(False, 'Service not available')
            return
        future = self._calib_client.call_async(Trigger.Request())
        future.add_done_callback(
            lambda f: done_cb(f.result().success, f.result().message))

    def call_toggle_episode(self, done_cb):
        if not self._toggle_ep_client.wait_for_service(timeout_sec=1.0):
            done_cb(False, 'Service not available')
            return
        future = self._toggle_ep_client.call_async(Trigger.Request())
        future.add_done_callback(
            lambda f: done_cb(f.result().success, f.result().message))


# ---------------------------------------------------------------------------
# Qt signals bridge (thread-safe)
# ---------------------------------------------------------------------------

class Signals(QObject):
    pedal_updated       = Signal(list)
    node_status_updated = Signal(dict)
    calib_status        = Signal(str)
    calib_started       = Signal()
    calib_failed        = Signal(str)
    rec_state_changed   = Signal(str)
    rec_episode_changed = Signal(int)


# ---------------------------------------------------------------------------
# Main window
# ---------------------------------------------------------------------------

class TeleopGuiWindow(QWidget):

    def __init__(self, ros_node: TeleopGuiNode, signals: Signals):
        super().__init__()
        self._node = ros_node
        self._sig  = signals
        self._rec_state = 'IDLE'

        signals.pedal_updated.connect(self._on_pedal)
        signals.node_status_updated.connect(self._on_nodes)
        signals.calib_status.connect(self._on_calib_status)
        signals.calib_started.connect(self._start_calib_progress)
        signals.calib_failed.connect(self._on_calib_failed)
        signals.rec_state_changed.connect(self._on_rec_state)
        signals.rec_episode_changed.connect(self._on_rec_episode)

        self._build_ui()

    # ------------------------------------------------------------------
    # UI construction
    # ------------------------------------------------------------------

    def _build_ui(self):
        self.setWindowTitle('Teleop Control')
        self.setMinimumWidth(360)

        root = QVBoxLayout()
        root.setSpacing(8)
        root.addWidget(self._build_node_panel())
        root.addWidget(self._build_pedal_panel())
        root.addWidget(self._build_teleop_panel())
        root.addWidget(self._build_recording_panel())
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
            grid.addWidget(dot, i, 0, Qt.AlignCenter)
            grid.addWidget(QLabel(f'{pkg} / {node}'), i, 1)
            self._node_dots[node] = dot
        group.setLayout(grid)
        return group

    def _build_pedal_panel(self):
        group  = QGroupBox('Pedal')
        layout = QHBoxLayout()
        self._pedal_btns = []
        for label in ['Engage', '—', 'Episode']:
            btn = QPushButton(label)
            btn.setEnabled(False)
            btn.setFixedHeight(36)
            btn.setStyleSheet('background-color: #ccc; color: #444;')
            layout.addWidget(btn)
            self._pedal_btns.append(btn)
        group.setLayout(layout)
        return group

    def _build_teleop_panel(self):
        group  = QGroupBox('Teleop')
        layout = QHBoxLayout()
        layout.setSpacing(6)

        buttons = [
            ('▶  Teleop Start', 'teleop_start', '#4CAF50', 'white'),
            ('Zero Pose',       'zero_pose',    '#5C6BC0', 'white'),
            ('VLA Pose',        'vla_pose2',    '#5C6BC0', 'white'),
            ('■  Teleop Stop',  'teleop_stop',  '#E53935', 'white'),
        ]
        for label, cmd, bg, fg in buttons:
            btn = QPushButton(label)
            btn.setFixedHeight(36)
            btn.setStyleSheet(f'background-color: {bg}; color: {fg}; font-weight: bold;')
            btn.clicked.connect(lambda checked, c=cmd: self._node.publish_rby1_command(c))
            layout.addWidget(btn)

        group.setLayout(layout)
        return group

    def _build_recording_panel(self):
        group  = QGroupBox('Recording')
        layout = QVBoxLayout()
        layout.setSpacing(6)

        # State indicator
        self._rec_state_label = QLabel('⬤ IDLE')
        self._rec_state_label.setFont(QFont('Monospace', 12))
        self._rec_state_label.setStyleSheet('color: #888;')
        self._rec_countdown = 0
        self._rec_countdown_timer = QTimer()
        self._rec_countdown_timer.timeout.connect(self._tick_countdown)

        # task_id row
        task_row = QHBoxLayout()
        task_row.addWidget(QLabel('task_id'))
        self._task_spin = QSpinBox()
        self._task_spin.setMinimum(0)
        self._task_spin.setMaximum(9999)
        self._task_spin.setFixedWidth(70)
        self._task_spin.valueChanged.connect(self._on_task_id_changed)
        task_row.addWidget(self._task_spin)
        task_row.addStretch()

        # episode row
        ep_row = QHBoxLayout()
        ep_row.addWidget(QLabel('episode'))
        self._ep_label = QLabel('—')
        self._ep_label.setFont(QFont('Monospace', 11))
        ep_row.addWidget(self._ep_label)
        ep_row.addStretch()

        # Control mode: Position / Impedance
        mode_row = QHBoxLayout()
        mode_row.addWidget(QLabel('Mode'))
        self._radio_position  = QRadioButton('Position')
        self._radio_impedance = QRadioButton('Impedance')
        self._radio_position.setChecked(True)
        self._mode_group = QButtonGroup()
        self._mode_group.addButton(self._radio_position,  0)
        self._mode_group.addButton(self._radio_impedance, 1)
        self._mode_group.idClicked.connect(self._on_control_mode_changed)
        mode_row.addWidget(self._radio_position)
        mode_row.addWidget(self._radio_impedance)
        mode_row.addStretch()

        # Start / End button
        self._rec_btn = QPushButton('▶  Start Episode')
        self._rec_btn.setFixedHeight(36)
        self._rec_btn.setStyleSheet('background-color: #4CAF50; color: white; font-weight: bold;')
        self._rec_btn.clicked.connect(self._on_rec_btn)

        layout.addWidget(self._rec_state_label)
        layout.addLayout(task_row)
        layout.addLayout(ep_row)
        layout.addLayout(mode_row)
        layout.addWidget(self._rec_btn)
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
        for i, (btn, pressed) in enumerate(zip(self._pedal_btns, state)):
            color = '#A6D256' if pressed else '#ccc'
            btn.setStyleSheet(f'background-color: {color}; color: #333;')

    def _on_nodes(self, status):
        for node, alive in status.items():
            if node in self._node_dots:
                color = '#A6D256' if alive else '#ED325A'
                self._node_dots[node].setStyleSheet(f'color: {color};')

    def _on_rec_state(self, state: str):
        prev_state = self._rec_state
        self._rec_state = state
        text, color = REC_STATE_STYLE.get(state, ('⬤ ' + state, '#888'))
        self._rec_state_label.setText(text)
        self._rec_state_label.setStyleSheet(f'color: {color};')

        is_idle = (state == 'IDLE')
        self._task_spin.setEnabled(is_idle)

        if is_idle:
            self._rec_countdown_timer.stop()
            self._rec_btn.setText('▶  Start Episode')
            self._rec_btn.setStyleSheet(
                'background-color: #4CAF50; color: white; font-weight: bold;')
            self._rec_btn.setEnabled(True)
            self._ep_label.setText('—')
        elif prev_state == 'IDLE' and state == 'READY':
            # Just started — 3-second countdown before End button appears
            self._rec_btn.setText('Starting in 3...')
            self._rec_btn.setStyleSheet(
                'background-color: #888; color: white; font-weight: bold;')
            self._rec_btn.setEnabled(False)
            self._rec_countdown = 3
            self._rec_countdown_timer.start(1000)
        else:
            self._rec_btn.setText('■  End Episode')
            self._rec_btn.setStyleSheet(
                'background-color: #E53935; color: white; font-weight: bold;')
            self._rec_btn.setEnabled(state in ('READY', 'PAUSED'))

    def _tick_countdown(self):
        self._rec_countdown -= 1
        if self._rec_countdown > 0:
            self._rec_btn.setText(f'Starting in {self._rec_countdown}...')
        else:
            self._rec_countdown_timer.stop()
            self._rec_btn.setText('■  End Episode')
            self._rec_btn.setStyleSheet(
                'background-color: #E53935; color: white; font-weight: bold;')
            self._rec_btn.setEnabled(True)

    def _on_rec_episode(self, episode: int):
        self._ep_label.setText(str(episode) if episode >= 0 else '—')

    def _on_task_id_changed(self, index: int):
        self._node.publish_task_id(index)

    def _on_control_mode_changed(self, button_id: int):
        mode = 'impedance' if button_id == 1 else 'position'
        self._node.publish_control_mode(mode)

    def _on_rec_btn(self):
        self._rec_btn.setEnabled(False)
        def done(success, msg):
            self._rec_btn.setEnabled(True)
            if not success:
                self.get_logger() if False else None  # no-op; errors shown via rec_state topic
        threading.Thread(
            target=self._node.call_toggle_episode, args=(done,), daemon=True).start()

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
                self._sig.calib_failed.emit(f'FAILED: {msg}')
            else:
                self._sig.calib_started.emit()

        threading.Thread(
            target=self._node.call_calibrate, args=(done,), daemon=True).start()

    def _on_calib_failed(self, msg):
        self._sig.calib_status.emit(msg)
        self._calib_btn.setEnabled(True)

    def _start_calib_progress(self):
        self._calib_elapsed = 0.0
        self._calib_phase   = 1
        self._sig.calib_status.emit('Phase 1/2: Open hands fully...')
        self._calib_tick_timer = QTimer()
        self._calib_tick_timer.timeout.connect(self._tick_calib)
        self._calib_tick_timer.start(100)

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

    ros_node._pedal_cbs.append(       lambda s: signals.pedal_updated.emit(s))
    ros_node._node_status_cbs.append( lambda s: signals.node_status_updated.emit(s))
    ros_node._rec_state_cbs.append(   lambda s: signals.rec_state_changed.emit(s))
    ros_node._rec_episode_cbs.append( lambda e: signals.rec_episode_changed.emit(e))

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
