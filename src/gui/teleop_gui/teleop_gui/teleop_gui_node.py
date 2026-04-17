"""
teleop_gui_node.py  (v2 — wide layout)

Top:    RB-Y1 area  — connect, status, power/servo/pose controls
Bottom: Teleop area — node status, pedal, teleop, recording, calibration
"""

import json
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
    QRadioButton, QButtonGroup, QSpinBox, QLineEdit,
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
    ('rby1_rt_node',         'rby1_rt'),
]

REC_STATE_STYLE = {
    'IDLE':      ('⬤ IDLE',      '#888888'),
    'READY':     ('⬤ READY',     '#F0C040'),
    'RECORDING': ('⬤ RECORDING', '#E0302A'),
    'PAUSED':    ('⬤ PAUSED',    '#E08020'),
}

# status indicator colors
_C_ON    = '#A6D256'
_C_OFF   = '#ED325A'
_C_IDLE  = 'lightgray'
_C_FAULT = '#ED325A'


# ---------------------------------------------------------------------------
# ROS2 node (background thread)
# ---------------------------------------------------------------------------

class TeleopGuiNode(Node):

    def __init__(self):
        super().__init__('teleop_gui')
        self._pedal_state = [0, 0, 0]
        self._pedal_cbs         = []
        self._node_status_cbs   = []
        self._rec_state_cbs     = []
        self._rec_episode_cbs   = []
        self._tracker_status_cbs = []
        self._rby1_status_cbs   = []

        self.create_subscription(Joy,    '/teleop/pedal',          self._cb_pedal,          10)
        self.create_subscription(String, '/teleop/rec_state',      self._cb_rec_state,      10)
        self.create_subscription(Int32,  '/teleop/rec_episode',    self._cb_rec_episode,    10)
        self.create_subscription(String, '/teleop/tracker_status', self._cb_tracker_status, 10)
        self.create_subscription(String, '/rby1_status',           self._cb_rby1_status,    10)
        self.create_timer(1.0, self._poll_nodes)

        self._calib_client     = self.create_client(Trigger, '/manus_inspire/calibrate')
        self._toggle_ep_client = self.create_client(Trigger, '/vive_rby1/toggle_episode')

        self._task_id_pub      = self.create_publisher(Int32,  '/teleop/task_id',      10)
        self._control_mode_pub = self.create_publisher(String, '/teleop/control_mode', 10)
        self._rby1_cmd_pub     = self.create_publisher(String, '/teleop/rby1_command', 10)
        self._mirror_mode_pub  = self.create_publisher(String, '/teleop/mirror_mode',  10)

    # ── callbacks ──────────────────────────────────────────────────────────

    def _cb_pedal(self, msg):
        state = list(msg.buttons[:3]) + [0] * max(0, 3 - len(msg.buttons))
        self._pedal_state = state[:3]
        for cb in self._pedal_cbs:
            cb(self._pedal_state)

    def _cb_rec_state(self, msg):
        for cb in self._rec_state_cbs:
            cb(msg.data)

    def _cb_rec_episode(self, msg):
        for cb in self._rec_episode_cbs:
            cb(msg.data)

    def _cb_tracker_status(self, msg):
        parts = msg.data.split()
        sl = parts[0].split(':')[1]
        sr = parts[1].split(':')[1]
        for cb in self._tracker_status_cbs:
            cb(sl, sr)

    def _cb_rby1_status(self, msg):
        try:
            data = json.loads(msg.data)
            for cb in self._rby1_status_cbs:
                cb(data)
        except Exception:
            pass

    def _poll_nodes(self):
        names  = {n for n, _ in self.get_node_names_and_namespaces()}
        status = {node: (node in names) for node, _ in NODES_TO_WATCH}
        for cb in self._node_status_cbs:
            cb(status)

    # ── publishers / service calls ─────────────────────────────────────────

    def publish_task_id(self, task_id):
        self._task_id_pub.publish(Int32(data=task_id))

    def publish_control_mode(self, mode):
        self._control_mode_pub.publish(String(data=mode))

    def publish_rby1_command(self, command):
        self._rby1_cmd_pub.publish(String(data=command))

    def publish_mirror_mode(self, mirror):
        self._mirror_mode_pub.publish(String(data='mirror' if mirror else 'normal'))

    def call_calibrate(self, done_cb):
        if not self._calib_client.wait_for_service(timeout_sec=1.0):
            done_cb(False, 'Service not available')
            return
        future = self._calib_client.call_async(Trigger.Request())
        future.add_done_callback(lambda f: done_cb(f.result().success, f.result().message))

    def call_toggle_episode(self, done_cb):
        if not self._toggle_ep_client.wait_for_service(timeout_sec=1.0):
            done_cb(False, 'Service not available')
            return
        future = self._toggle_ep_client.call_async(Trigger.Request())
        future.add_done_callback(lambda f: done_cb(f.result().success, f.result().message))


# ---------------------------------------------------------------------------
# Qt signals bridge
# ---------------------------------------------------------------------------

class Signals(QObject):
    pedal_updated          = Signal(list)
    node_status_updated    = Signal(dict)
    calib_status           = Signal(str)
    calib_started          = Signal()
    calib_failed           = Signal(str)
    rec_state_changed      = Signal(str)
    rec_episode_changed    = Signal(int)
    tracker_status_changed = Signal(str, str)
    rby1_status_changed    = Signal(dict)


# ---------------------------------------------------------------------------
# Helper: colored status label
# ---------------------------------------------------------------------------

def _status_label(title: str) -> QLabel:
    lbl = QLabel(f'  {title}: —  ')
    lbl.setFont(QFont('Monospace', 10))
    lbl.setAlignment(Qt.AlignCenter)
    lbl.setFixedHeight(26)
    lbl.setStyleSheet(f'background-color: {_C_IDLE}; border-radius: 4px;')
    return lbl


def _btn(text, color, text_color='white', height=32) -> QPushButton:
    b = QPushButton(text)
    b.setFixedHeight(height)
    b.setStyleSheet(
        f'background-color: {color}; color: {text_color}; font-weight: bold;')
    return b


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
        signals.tracker_status_changed.connect(self._on_tracker_status)
        signals.rby1_status_changed.connect(self._on_rby1_status)

        self._build_ui()

    # ── UI construction ────────────────────────────────────────────────────

    def _build_ui(self):
        self.setWindowTitle('Teleop Control')
        self.setMinimumWidth(960)

        root = QVBoxLayout()
        root.setSpacing(6)
        root.addWidget(self._build_rby1_panel())
        root.addWidget(self._build_teleop_section())
        self.setLayout(root)

    # ── RB-Y1 panel ────────────────────────────────────────────────────────

    def _build_rby1_panel(self):
        group  = QGroupBox('RB-Y1')
        layout = QVBoxLayout()
        layout.setSpacing(5)

        layout.addLayout(self._build_rby1_status_row())
        layout.addLayout(self._build_rby1_connect_row())
        layout.addLayout(self._build_rby1_init_row())
        layout.addLayout(self._build_rby1_pose_row())

        group.setLayout(layout)
        return group

    def _build_rby1_status_row(self):
        row = QHBoxLayout()
        row.setSpacing(6)

        self._lbl_power   = _status_label('Power')
        self._lbl_servo   = _status_label('Servo')
        self._lbl_control = _status_label('Control')
        self._lbl_stream  = _status_label('Stream')

        for lbl in (self._lbl_power, self._lbl_servo,
                    self._lbl_control, self._lbl_stream):
            row.addWidget(lbl)
        return row

    def _build_rby1_connect_row(self):
        row = QHBoxLayout()
        row.setSpacing(6)

        self._rb_sim  = QRadioButton('Sim')
        self._rb_real = QRadioButton('Real')
        self._rb_sim.setChecked(True)
        conn_grp = QButtonGroup(self)
        conn_grp.addButton(self._rb_sim,  0)
        conn_grp.addButton(self._rb_real, 1)
        conn_grp.idClicked.connect(self._on_sim_real_changed)

        self._ip_edit = QLineEdit('localhost:50051')
        self._ip_edit.setFixedWidth(200)

        btn_connect = _btn('Connect', '#1565C0', height=30)
        btn_connect.setFixedWidth(90)
        btn_connect.clicked.connect(self._on_connect)

        row.addWidget(self._rb_sim)
        row.addWidget(self._rb_real)
        row.addWidget(self._ip_edit)
        row.addWidget(btn_connect)
        row.addStretch()
        return row

    def _build_rby1_init_row(self):
        row = QHBoxLayout()
        row.setSpacing(5)

        btns = [
            ('Power On',      'power_on',      '#388E3C'),
            ('Servo On',      'servo_on',      '#1976D2'),
            ('Err Reset',     'error_reset',   '#F57C00'),
            ('Ctrl Enable',   'control_enable','#7B1FA2'),
            ('Gripper Init',  'gripper_init',  '#00838F'),
        ]
        for label, cmd, color in btns:
            b = _btn(label, color, height=30)
            b.clicked.connect(lambda _, c=cmd: self._node.publish_rby1_command(c))
            row.addWidget(b)
        row.addStretch()
        return row

    def _build_rby1_pose_row(self):
        row = QHBoxLayout()
        row.setSpacing(5)

        btns = [
            ('Power Off',  'power_off',  '#C62828'),
            ('Zero Pose',  'zero_pose',  '#546E7A'),
            ('Ready Pose', 'ready_pose', '#546E7A'),
            ('VLA Pose',   'vla_pose',   '#546E7A'),
            ('VLA2 Pose',  'vla_pose2',  '#546E7A'),
        ]
        for label, cmd, color in btns:
            b = _btn(label, color, height=30)
            b.clicked.connect(lambda _, c=cmd: self._node.publish_rby1_command(c))
            row.addWidget(b)

        # Stop Move — 긴 pause 없이 즉시 stream 종료
        b_stop = _btn('⬛ Stop Move', '#B71C1C', height=30)
        b_stop.clicked.connect(lambda: self._node.publish_rby1_command('stop_move'))
        row.addWidget(b_stop)
        row.addStretch()
        return row

    # ── Teleop section (bottom) ────────────────────────────────────────────

    def _build_teleop_section(self):
        group  = QGroupBox('Teleop')
        layout = QHBoxLayout()
        layout.setSpacing(8)

        layout.addWidget(self._build_node_panel(), 2)
        layout.addWidget(self._build_teleop_panel(), 2)
        layout.addWidget(self._build_recording_panel(), 3)
        layout.addWidget(self._build_calib_panel(), 2)

        group.setLayout(layout)
        return group

    def _build_node_panel(self):
        group = QGroupBox('Nodes')
        vbox  = QVBoxLayout()
        vbox.setSpacing(3)

        grid = QGridLayout()
        grid.setSpacing(3)
        self._node_dots = {}
        for i, (node, pkg) in enumerate(NODES_TO_WATCH):
            dot = QLabel('●')
            dot.setFont(QFont('Monospace', 13))
            dot.setStyleSheet('color: #888;')
            grid.addWidget(dot, i, 0, Qt.AlignCenter)
            grid.addWidget(QLabel(pkg), i, 1)
            self._node_dots[node] = dot

        # Tracker status
        n = len(NODES_TO_WATCH)
        self._tracker_dot_l = QLabel('● L')
        self._tracker_dot_r = QLabel('● R')
        for dot in (self._tracker_dot_l, self._tracker_dot_r):
            dot.setFont(QFont('Monospace', 10))
            dot.setStyleSheet('color: #888;')
        tr_row = QHBoxLayout()
        tr_row.addWidget(self._tracker_dot_l)
        tr_row.addWidget(self._tracker_dot_r)
        tr_row.addStretch()
        tr_widget = QWidget()
        tr_widget.setLayout(tr_row)
        grid.addWidget(QLabel('Tracker'), n, 1)
        grid.addWidget(tr_widget, n, 2)

        vbox.addLayout(grid)
        vbox.addSpacing(4)
        vbox.addWidget(self._build_pedal_panel())
        vbox.addStretch()
        group.setLayout(vbox)
        return group

    def _build_pedal_panel(self):
        group  = QGroupBox('Pedal')
        layout = QHBoxLayout()
        self._pedal_btns = []
        for label in ['Resume/Pause', '—', '● Rec']:
            btn = QPushButton(label)
            btn.setEnabled(False)
            btn.setFixedHeight(32)
            btn.setStyleSheet('background-color: #ccc; color: #444;')
            layout.addWidget(btn)
            self._pedal_btns.append(btn)
        group.setLayout(layout)
        return group

    def _build_teleop_panel(self):
        group  = QGroupBox('Control')
        layout = QVBoxLayout()
        layout.setSpacing(5)

        # Teleop action buttons
        for label, cmd, bg in [
            ('▶  Teleop Start', 'teleop_start', '#4CAF50'),
            ('VLA2 Pose',       'vla_pose2',    '#5C6BC0'),
            ('■  Teleop Stop',  'teleop_stop',  '#E53935'),
        ]:
            b = _btn(label, bg, height=34)
            b.clicked.connect(lambda _, c=cmd: self._node.publish_rby1_command(c))
            layout.addWidget(b)

        layout.addSpacing(4)

        # Control mode
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
        layout.addLayout(mode_row)

        # Mirror mode
        mirror_row = QHBoxLayout()
        mirror_row.addWidget(QLabel('Tracking'))
        self._radio_normal = QRadioButton('Normal')
        self._radio_mirror = QRadioButton('Mirror')
        self._radio_normal.setChecked(True)
        self._mirror_group = QButtonGroup()
        self._mirror_group.addButton(self._radio_normal, 0)
        self._mirror_group.addButton(self._radio_mirror, 1)
        self._mirror_group.idClicked.connect(self._on_mirror_mode_changed)
        mirror_row.addWidget(self._radio_normal)
        mirror_row.addWidget(self._radio_mirror)
        mirror_row.addStretch()
        layout.addLayout(mirror_row)

        layout.addStretch()
        group.setLayout(layout)
        return group

    def _build_recording_panel(self):
        group  = QGroupBox('Recording')
        layout = QVBoxLayout()
        layout.setSpacing(5)

        self._rec_state_label = QLabel('⬤ IDLE')
        self._rec_state_label.setFont(QFont('Monospace', 12))
        self._rec_state_label.setStyleSheet('color: #888;')
        self._rec_countdown = 0
        self._rec_countdown_timer = QTimer()
        self._rec_countdown_timer.timeout.connect(self._tick_countdown)

        task_row = QHBoxLayout()
        task_row.addWidget(QLabel('task_id'))
        self._task_spin = QSpinBox()
        self._task_spin.setMinimum(0)
        self._task_spin.setMaximum(9999)
        self._task_spin.setFixedWidth(70)
        self._task_spin.valueChanged.connect(self._on_task_id_changed)
        task_row.addWidget(self._task_spin)
        task_row.addStretch()

        ep_row = QHBoxLayout()
        ep_row.addWidget(QLabel('episode'))
        self._ep_label = QLabel('—')
        self._ep_label.setFont(QFont('Monospace', 11))
        ep_row.addWidget(self._ep_label)
        ep_row.addStretch()

        self._rec_btn = QPushButton('▶  Start Episode')
        self._rec_btn.setFixedHeight(36)
        self._rec_btn.setStyleSheet(
            'background-color: #4CAF50; color: white; font-weight: bold;')
        self._rec_btn.clicked.connect(self._on_rec_btn)

        layout.addWidget(self._rec_state_label)
        layout.addLayout(task_row)
        layout.addLayout(ep_row)
        layout.addWidget(self._rec_btn)
        layout.addStretch()
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
        layout.addStretch()
        group.setLayout(layout)
        return group

    # ── Signal handlers ────────────────────────────────────────────────────

    def _on_sim_real_changed(self, btn_id):
        if btn_id == 0:
            self._ip_edit.setText('localhost:50051')
        else:
            self._ip_edit.setText('192.168.30.1:50051')

    def _on_connect(self):
        ip = self._ip_edit.text().strip()
        self._node.publish_rby1_command(f'connect\n{ip}')

    def _on_rby1_status(self, data: dict):
        power   = data.get('power_state',   'False') == 'True'
        servo   = data.get('servo_state',   'False') == 'True'
        stream  = data.get('stream_state',  'False') == 'True'
        ctrl    = data.get('control_state', '')

        def _apply(lbl, text, color):
            lbl.setText(f'  {text}  ')
            lbl.setStyleSheet(f'background-color: {color}; border-radius: 4px;')

        _apply(self._lbl_power,
               'Power On'  if power  else 'Power Off',
               _C_ON       if power  else _C_IDLE)

        _apply(self._lbl_servo,
               'Servo On'  if servo  else 'Servo Off',
               _C_ON       if servo  else _C_IDLE)

        if ctrl == 'State.Enabled':
            _apply(self._lbl_control, 'Enabled', _C_ON)
        elif 'Fault' in ctrl:
            _apply(self._lbl_control, 'FAULT', _C_FAULT)
        else:
            _apply(self._lbl_control, 'Idle', _C_IDLE)

        _apply(self._lbl_stream,
               'Stream On' if stream else 'Stream Off',
               _C_ON       if stream else _C_IDLE)

    def _on_pedal(self, state):
        for btn, pressed in zip(self._pedal_btns, state):
            color = '#A6D256' if pressed else '#ccc'
            btn.setStyleSheet(f'background-color: {color}; color: #333;')

    def _on_nodes(self, status):
        for node, alive in status.items():
            if node in self._node_dots:
                color = '#A6D256' if alive else '#ED325A'
                self._node_dots[node].setStyleSheet(f'color: {color};')

    def _on_tracker_status(self, sl, sr):
        _colors = {'OK': '#4CAF50', 'JITTER': '#F0C040', 'LOST': '#E0302A'}
        self._tracker_dot_l.setStyleSheet(f'color: {_colors.get(sl, "#888")};')
        self._tracker_dot_r.setStyleSheet(f'color: {_colors.get(sr, "#888")};')

    def _on_rec_state(self, state):
        prev_state = self._rec_state
        self._rec_state = state
        text, color = REC_STATE_STYLE.get(state, ('⬤ ' + state, '#888'))
        self._rec_state_label.setText(text)
        self._rec_state_label.setStyleSheet(f'color: {color};')

        is_idle = (state == 'IDLE')
        self._task_spin.setEnabled(is_idle)
        self._radio_position.setEnabled(is_idle)
        self._radio_impedance.setEnabled(is_idle)
        self._radio_normal.setEnabled(is_idle)
        self._radio_mirror.setEnabled(is_idle)

        if is_idle:
            self._rec_countdown_timer.stop()
            self._rec_btn.setText('▶  Start Episode')
            self._rec_btn.setStyleSheet(
                'background-color: #4CAF50; color: white; font-weight: bold;')
            self._rec_btn.setEnabled(True)
            self._ep_label.setText('—')
        elif prev_state == 'IDLE' and state == 'READY':
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

    def _on_rec_episode(self, episode):
        self._ep_label.setText(str(episode) if episode >= 0 else '—')

    def _on_task_id_changed(self, index):
        self._node.publish_task_id(index)

    def _on_control_mode_changed(self, button_id):
        self._node.publish_control_mode('impedance' if button_id == 1 else 'position')

    def _on_mirror_mode_changed(self, button_id):
        self._node.publish_mirror_mode(button_id == 1)

    def _on_rec_btn(self):
        self._rec_btn.setEnabled(False)
        def done(success, msg):
            self._rec_btn.setEnabled(True)
        threading.Thread(
            target=self._node.call_toggle_episode, args=(done,), daemon=True).start()

    # ── Calibration ────────────────────────────────────────────────────────

    def _on_calib_status(self, text):
        self._calib_label.setText(f'Status: {text}')

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

    _CALIB_PHASE_MSGS = {
        1: 'Phase 1/4: Open hands fully...',
        2: 'Phase 2/4: Thumbs up (fist, thumb pointing up)...',
        3: 'Phase 3/4: Press thumb to side of index finger...',
        4: 'Phase 4/4: Open fingers, bend thumb only...',
    }

    def _start_calib_progress(self):
        self._calib_elapsed = 0.0
        self._calib_phase   = 1
        self._sig.calib_status.emit(self._CALIB_PHASE_MSGS[1])
        self._calib_tick_timer = QTimer()
        self._calib_tick_timer.timeout.connect(self._tick_calib)
        self._calib_tick_timer.start(100)

    def _tick_calib(self):
        self._calib_elapsed += 0.1
        done_phases = (self._calib_phase - 1) * CALIB_DURATION
        pct = int((done_phases + self._calib_elapsed) / (CALIB_DURATION * 4) * 100)
        self._calib_bar.setValue(min(pct, 100))

        if self._calib_elapsed >= CALIB_DURATION:
            self._calib_elapsed = 0.0
            if self._calib_phase < 4:
                self._calib_phase += 1
                self._sig.calib_status.emit(self._CALIB_PHASE_MSGS[self._calib_phase])
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

    ros_node._pedal_cbs.append(          lambda s:    signals.pedal_updated.emit(s))
    ros_node._node_status_cbs.append(    lambda s:    signals.node_status_updated.emit(s))
    ros_node._rec_state_cbs.append(      lambda s:    signals.rec_state_changed.emit(s))
    ros_node._rec_episode_cbs.append(    lambda e:    signals.rec_episode_changed.emit(e))
    ros_node._tracker_status_cbs.append( lambda l, r: signals.tracker_status_changed.emit(l, r))
    ros_node._rby1_status_cbs.append(    lambda d:    signals.rby1_status_changed.emit(d))

    spin_thread = threading.Thread(target=rclpy.spin, args=(ros_node,), daemon=True)
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