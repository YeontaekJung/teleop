"""
teleop_gui_node.py  (v2 — wide layout)

Top:    RB-Y1 area  — connect, status, power/servo/pose controls
Bottom: Teleop area — node status, pedal, teleop, recording, calibration

Widget naming convention
  _lbl_*   QLabel
  _btn_*   QPushButton
  _rb_*    QRadioButton
  _bg_*    QButtonGroup
  _le_*    QLineEdit
  _chk_*   QCheckBox
  _spin_*  QSpinBox
  _pbar_*  QProgressBar
  _timer_* QTimer
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
    QRadioButton, QButtonGroup, QSpinBox, QLineEdit, QCheckBox,
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

_C_ON      = '#A6D256'
_C_OFF     = 'lightgray'
_C_OFF_RED = '#E53935'
_C_FAULT   = '#ED325A'


# ---------------------------------------------------------------------------
# ROS2 node (background thread)
# ---------------------------------------------------------------------------

class TeleopGuiNode(Node):

    def __init__(self):
        super().__init__('teleop_gui')
        self._pedal_state         = [0, 0, 0]
        self._pedal_cbs           = []
        self._node_status_cbs     = []
        self._rec_state_cbs       = []
        self._rec_episode_cbs     = []
        self._tracker_status_cbs  = []
        self._rby1_status_cbs     = []

        self.create_subscription(Joy,    '/teleop/pedal',          self._cb_pedal,          10)
        self.create_subscription(String, '/teleop/rec_state',      self._cb_rec_state,      10)
        self.create_subscription(Int32,  '/teleop/rec_episode',    self._cb_rec_episode,    10)
        self.create_subscription(String, '/teleop/tracker_status', self._cb_tracker_status, 10)
        self.create_subscription(String, '/rby1_status',           self._cb_rby1_status,    10)
        self.create_timer(1.0, self._poll_nodes)

        self._calib_client     = self.create_client(Trigger, '/manus_inspire/calibrate')
        self._toggle_ep_client = self.create_client(Trigger, '/vive_rby1/toggle_episode')

        self._pub_task_id      = self.create_publisher(Int32,  '/teleop/task_id',      10)
        self._pub_control_mode = self.create_publisher(String, '/teleop/control_mode', 10)
        self._pub_rby1_cmd     = self.create_publisher(String, '/teleop/rby1_command', 10)
        self._pub_mirror_mode  = self.create_publisher(String, '/teleop/mirror_mode',  10)

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

    def pub_task_id(self, task_id: int):
        self._pub_task_id.publish(Int32(data=task_id))

    def pub_control_mode(self, mode: str):
        self._pub_control_mode.publish(String(data=mode))

    def pub_rby1_cmd(self, command: str):
        self._pub_rby1_cmd.publish(String(data=command))

    def pub_mirror_mode(self, mirror: bool):
        self._pub_mirror_mode.publish(String(data='mirror' if mirror else 'normal'))

    def call_calibrate(self, done_cb):
        if not self._calib_client.wait_for_service(timeout_sec=1.0):
            done_cb(False, 'Service not available')
            return
        fut = self._calib_client.call_async(Trigger.Request())
        fut.add_done_callback(lambda f: done_cb(f.result().success, f.result().message))

    def call_toggle_episode(self, done_cb):
        if not self._toggle_ep_client.wait_for_service(timeout_sec=1.0):
            done_cb(False, 'Service not available')
            return
        fut = self._toggle_ep_client.call_async(Trigger.Request())
        fut.add_done_callback(lambda f: done_cb(f.result().success, f.result().message))


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
# Helpers
# ---------------------------------------------------------------------------

def _make_status_label(title: str) -> QLabel:
    lbl = QLabel(f'  {title}: —  ')
    lbl.setFont(QFont('Monospace', 10))
    lbl.setAlignment(Qt.AlignCenter)
    lbl.setFixedHeight(26)
    lbl.setStyleSheet(f'background-color: {_C_OFF}; border-radius: 4px;')
    return lbl


def _make_btn(text: str, color: str, text_color: str = 'white',
              height: int = 32) -> QPushButton:
    btn = QPushButton(text)
    btn.setFixedHeight(height)
    btn.setStyleSheet(
        f'background-color: {color}; color: {text_color}; font-weight: bold;')
    return btn


# ---------------------------------------------------------------------------
# Main window
# ---------------------------------------------------------------------------

class TeleopGuiWindow(QWidget):

    def __init__(self, ros_node: TeleopGuiNode, signals: Signals):
        super().__init__()
        self._node      = ros_node
        self._sig       = signals
        self._rec_state = 'IDLE'
        self._stream_on = False

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
        root.addWidget(self._build_rby1_group())
        root.addWidget(self._build_teleop_group())
        self.setLayout(root)

    # ── RB-Y1 group ────────────────────────────────────────────────────────

    def _build_rby1_group(self) -> QGroupBox:
        group = QGroupBox('RB-Y1')
        vbox  = QVBoxLayout()
        vbox.setSpacing(5)
        vbox.addLayout(self._build_status_row())
        vbox.addLayout(self._build_connect_row())

        # Init + Pose rows on the left, STOP MOVE spanning both on the right
        bottom = QHBoxLayout()
        bottom.setSpacing(8)
        left = QVBoxLayout()
        left.setSpacing(4)
        left.addLayout(self._build_init_row())
        left.addLayout(self._build_pose_row())
        bottom.addLayout(left, 1)

        btn_stop = QPushButton('⚠  STOP\nMOVE')
        btn_stop.setFixedSize(110, 70)
        btn_stop.setStyleSheet(
            'background-color: #FFD600; color: #000000;'
            'font-weight: bold; font-size: 13px;')
        btn_stop.clicked.connect(lambda: self._node.pub_rby1_cmd('stop_move'))
        bottom.addWidget(btn_stop)

        vbox.addLayout(bottom)
        group.setLayout(vbox)
        return group

    def _build_status_row(self) -> QHBoxLayout:
        row = QHBoxLayout()
        row.setSpacing(6)
        self._lbl_power   = _make_status_label('Power')
        self._lbl_servo   = _make_status_label('Servo')
        self._lbl_control = _make_status_label('Control')
        self._lbl_stream  = _make_status_label('Stream')
        self._lbl_gripper = _make_status_label('Gripper')
        for lbl in (self._lbl_power, self._lbl_servo, self._lbl_control,
                    self._lbl_stream, self._lbl_gripper):
            row.addWidget(lbl)
        return row

    def _build_connect_row(self) -> QHBoxLayout:
        row = QHBoxLayout()
        row.setSpacing(6)

        self._rb_sim  = QRadioButton('Sim')
        self._rb_real = QRadioButton('Real')
        self._rb_sim.setChecked(True)
        self._bg_conn = QButtonGroup(self)
        self._bg_conn.addButton(self._rb_sim,  0)
        self._bg_conn.addButton(self._rb_real, 1)
        self._bg_conn.idClicked.connect(self._on_sim_real_changed)

        self._le_ip = QLineEdit('localhost:50051')
        self._le_ip.setFixedWidth(200)

        self._chk_no_gripper = QCheckBox('No Gripper')
        self._chk_no_gripper.setChecked(True)

        self._btn_connect = _make_btn('Connect', '#1565C0', height=30)
        self._btn_connect.setFixedWidth(90)
        self._btn_connect.clicked.connect(self._on_connect)

        row.addWidget(self._rb_sim)
        row.addWidget(self._rb_real)
        row.addWidget(self._le_ip)
        row.addWidget(self._chk_no_gripper)
        row.addWidget(self._btn_connect)
        row.addStretch()
        return row

    def _build_init_row(self) -> QHBoxLayout:
        row = QHBoxLayout()
        row.setSpacing(5)
        for label, cmd, color in [
            ('Power On',     'power_on',      '#388E3C'),
            ('Servo On',     'servo_on',      '#1976D2'),
            ('Err Reset',    'error_reset',   '#F57C00'),
            ('Ctrl Enable',  'control_enable','#7B1FA2'),
            ('Gripper Init', 'gripper_init',  '#00838F'),
        ]:
            btn = _make_btn(label, color, height=30)
            if cmd == 'teleop_start':
                btn.clicked.connect(self._on_teleop_start)
            else:
                btn.clicked.connect(lambda _, c=cmd: self._node.pub_rby1_cmd(c))
            row.addWidget(btn)
        row.addStretch()
        return row

    def _build_pose_row(self) -> QHBoxLayout:
        row = QHBoxLayout()
        row.setSpacing(5)
        for label, cmd, color in [
            ('Power Off',  'power_off',  '#C62828'),
            ('Zero Pose',  'zero_pose',  '#546E7A'),
            ('Ready Pose', 'ready_pose', '#546E7A'),
            ('VLA Pose',   'vla_pose',   '#546E7A'),
            ('VLA2 Pose',  'vla_pose2',  '#546E7A'),
        ]:
            btn = _make_btn(label, color, height=30)
            btn.clicked.connect(lambda _, c=cmd: self._node.pub_rby1_cmd(c))
            row.addWidget(btn)
        row.addStretch()
        return row

    # ── Teleop group ───────────────────────────────────────────────────────

    def _build_teleop_group(self) -> QGroupBox:
        group = QGroupBox('Teleop')
        hbox  = QHBoxLayout()
        hbox.setSpacing(8)
        hbox.addWidget(self._build_nodes_panel(), 2)
        hbox.addWidget(self._build_control_panel(), 2)
        hbox.addWidget(self._build_recording_panel(), 3)
        hbox.addWidget(self._build_calib_panel(), 2)
        group.setLayout(hbox)
        return group

    def _build_nodes_panel(self) -> QGroupBox:
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

        # Tracker status row
        n = len(NODES_TO_WATCH)
        self._lbl_tracker_l = QLabel('● L')
        self._lbl_tracker_r = QLabel('● R')
        for lbl in (self._lbl_tracker_l, self._lbl_tracker_r):
            lbl.setFont(QFont('Monospace', 10))
            lbl.setStyleSheet('color: #888;')
        tr_row = QHBoxLayout()
        tr_row.addWidget(self._lbl_tracker_l)
        tr_row.addWidget(self._lbl_tracker_r)
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

    def _build_pedal_panel(self) -> QGroupBox:
        group = QGroupBox('Pedal')
        hbox  = QHBoxLayout()
        self._btn_pedals = []
        for label in ['Resume/Pause', '—', '● Rec']:
            btn = QPushButton(label)
            btn.setEnabled(False)
            btn.setFixedHeight(32)
            btn.setStyleSheet('background-color: #ccc; color: #444;')
            hbox.addWidget(btn)
            self._btn_pedals.append(btn)
        group.setLayout(hbox)
        return group

    def _build_control_panel(self) -> QGroupBox:
        group = QGroupBox('Control')
        vbox  = QVBoxLayout()
        vbox.setSpacing(5)

        teleop_row = QHBoxLayout()
        teleop_row.setSpacing(5)
        for label, cmd, color in [
            ('▶  Teleop Start', 'teleop_start', '#4CAF50'),
            ('■  Teleop Stop',  'teleop_stop',  '#E53935'),
        ]:
            btn = _make_btn(label, color, height=34)
            btn.clicked.connect(lambda _, c=cmd: self._node.pub_rby1_cmd(c))
            teleop_row.addWidget(btn)
        vbox.addLayout(teleop_row)

        vbox.addSpacing(4)

        ik_row = QHBoxLayout()
        ik_row.addWidget(QLabel('IK'))
        self._rb_ik_sdk  = QRadioButton('SDK')
        self._rb_ik_pink = QRadioButton('Pink')
        self._rb_ik_sdk.setChecked(True)
        self._bg_ik = QButtonGroup()
        self._bg_ik.addButton(self._rb_ik_sdk,  0)
        self._bg_ik.addButton(self._rb_ik_pink, 1)
        self._bg_ik.idClicked.connect(self._on_ik_mode_changed)
        ik_row.addWidget(self._rb_ik_sdk)
        ik_row.addWidget(self._rb_ik_pink)
        ik_row.addStretch()
        vbox.addLayout(ik_row)

        ctrl_row = QHBoxLayout()
        ctrl_row.addWidget(QLabel('Ctrl'))
        self._rb_ctrl_position  = QRadioButton('Position')
        self._rb_ctrl_impedance = QRadioButton('Impedance')
        self._rb_ctrl_position.setChecked(True)
        self._bg_ctrl = QButtonGroup()
        self._bg_ctrl.addButton(self._rb_ctrl_position,  0)
        self._bg_ctrl.addButton(self._rb_ctrl_impedance, 1)
        self._bg_ctrl.idClicked.connect(self._on_ctrl_mode_changed)
        ctrl_row.addWidget(self._rb_ctrl_position)
        ctrl_row.addWidget(self._rb_ctrl_impedance)
        ctrl_row.addStretch()
        vbox.addLayout(ctrl_row)

        mirror_row = QHBoxLayout()
        mirror_row.addWidget(QLabel('Tracking'))
        self._rb_track_normal = QRadioButton('Normal')
        self._rb_track_mirror = QRadioButton('Mirror')
        self._rb_track_normal.setChecked(True)
        self._bg_track = QButtonGroup()
        self._bg_track.addButton(self._rb_track_normal, 0)
        self._bg_track.addButton(self._rb_track_mirror, 1)
        self._bg_track.idClicked.connect(self._on_mirror_mode_changed)
        mirror_row.addWidget(self._rb_track_normal)
        mirror_row.addWidget(self._rb_track_mirror)
        mirror_row.addStretch()
        vbox.addLayout(mirror_row)

        vbox.addStretch()
        group.setLayout(vbox)
        return group

    def _build_recording_panel(self) -> QGroupBox:
        group = QGroupBox('Recording')
        vbox  = QVBoxLayout()
        vbox.setSpacing(5)

        self._lbl_rec_state = QLabel('⬤ IDLE')
        self._lbl_rec_state.setFont(QFont('Monospace', 12))
        self._lbl_rec_state.setStyleSheet('color: #888;')

        self._timer_rec_countdown = QTimer()
        self._timer_rec_countdown.timeout.connect(self._tick_countdown)
        self._rec_countdown = 0

        task_row = QHBoxLayout()
        task_row.addWidget(QLabel('task_id'))
        self._spin_task = QSpinBox()
        self._spin_task.setMinimum(0)
        self._spin_task.setMaximum(9999)
        self._spin_task.setFixedWidth(70)
        self._spin_task.valueChanged.connect(
            lambda v: self._node.pub_task_id(v))
        task_row.addWidget(self._spin_task)
        task_row.addStretch()

        ep_row = QHBoxLayout()
        ep_row.addWidget(QLabel('episode'))
        self._lbl_episode = QLabel('—')
        self._lbl_episode.setFont(QFont('Monospace', 11))
        ep_row.addWidget(self._lbl_episode)
        ep_row.addStretch()

        self._btn_rec = _make_btn('▶  Start Episode', '#4CAF50', height=36)
        self._btn_rec.clicked.connect(self._on_rec_btn)

        vbox.addWidget(self._lbl_rec_state)
        vbox.addLayout(task_row)
        vbox.addLayout(ep_row)
        vbox.addWidget(self._btn_rec)
        vbox.addStretch()
        group.setLayout(vbox)
        return group

    def _build_calib_panel(self) -> QGroupBox:
        group = QGroupBox('Manus Calibration')
        vbox  = QVBoxLayout()
        self._lbl_calib = QLabel('Status: READY')
        self._pbar_calib = QProgressBar()
        self._pbar_calib.setRange(0, 100)
        self._pbar_calib.setValue(0)
        self._pbar_calib.setVisible(False)
        self._btn_calib = QPushButton('Recalibrate')
        self._btn_calib.setFixedHeight(36)
        self._btn_calib.clicked.connect(self._on_recalibrate)
        vbox.addWidget(self._lbl_calib)
        vbox.addWidget(self._pbar_calib)
        vbox.addWidget(self._btn_calib)
        vbox.addStretch()
        group.setLayout(vbox)
        return group

    # ── Signal handlers ────────────────────────────────────────────────────

    def _on_sim_real_changed(self, btn_id: int):
        self._le_ip.setText(
            'localhost:50051' if btn_id == 0 else '192.168.30.1:50051')

    def _on_connect(self):
        cmd = f'connect\n{self._le_ip.text().strip()}'
        if self._chk_no_gripper.isChecked():
            cmd += '\nno_gripper'
        self._node.pub_rby1_cmd(cmd)

    def _on_rby1_status(self, data: dict):
        power   = data.get('power_state',   'False') == 'True'
        servo   = data.get('servo_state',   'False') == 'True'
        stream  = data.get('stream_state',  'False') == 'True'
        gripper = data.get('gripper_state', 'False') == 'True'
        ctrl    = data.get('control_state', '')

        def _set(lbl, text, color):
            lbl.setText(f'  {text}  ')
            lbl.setStyleSheet(f'background-color: {color}; border-radius: 4px;')

        _set(self._lbl_power,   'Power On'  if power   else 'Power Off',  _C_ON if power   else _C_OFF_RED)
        _set(self._lbl_servo,   'Servo On'  if servo   else 'Servo Off',  _C_ON if servo   else _C_OFF_RED)
        _set(self._lbl_stream,  'Stream On' if stream  else 'Stream Off', _C_ON if stream  else _C_OFF_RED)
        _set(self._lbl_gripper, 'Gripper ✓' if gripper else 'Gripper ✗',  _C_ON if gripper else _C_OFF)

        if stream != self._stream_on:
            self._stream_on = stream
            self._update_rec_btn_enable()

        if ctrl == 'State.Enabled':
            _set(self._lbl_control, 'Enabled', _C_ON)
        elif 'Fault' in ctrl:
            _set(self._lbl_control, 'FAULT', _C_FAULT)
        else:
            _set(self._lbl_control, 'Idle', _C_OFF)

    def _update_rec_btn_enable(self):
        """stream 상태가 바뀌면 End Episode 버튼 활성화 상태 갱신"""
        if self._btn_rec.text() == '■  End Episode':
            self._btn_rec.setEnabled(self._stream_on and self._rec_state in ('READY', 'PAUSED'))

    def _on_pedal(self, state: list):
        for i, (btn, pressed) in enumerate(zip(self._btn_pedals, state)):
            color = '#A6D256' if pressed else '#ccc'
            btn.setStyleSheet(f'background-color: {color}; color: #333;')
            if i == 0:
                btn.setProperty('pressed', pressed)

    def _on_nodes(self, status: dict):
        for node, alive in status.items():
            if node in self._node_dots:
                color = '#A6D256' if alive else '#ED325A'
                self._node_dots[node].setStyleSheet(f'color: {color};')

    def _on_tracker_status(self, sl: str, sr: str):
        _colors = {'OK': '#4CAF50', 'JITTER': '#F0C040', 'LOST': '#E0302A'}
        self._lbl_tracker_l.setStyleSheet(f'color: {_colors.get(sl, "#888")};')
        self._lbl_tracker_r.setStyleSheet(f'color: {_colors.get(sr, "#888")};')

    def _on_rec_state(self, state: str):
        prev            = self._rec_state
        self._rec_state = state
        text, color     = REC_STATE_STYLE.get(state, ('⬤ ' + state, '#888'))
        self._lbl_rec_state.setText(text)
        self._lbl_rec_state.setStyleSheet(f'color: {color};')

        is_idle = (state == 'IDLE')
        for w in (self._spin_task,
                  self._rb_ik_sdk, self._rb_ik_pink,
                  self._rb_ctrl_position, self._rb_ctrl_impedance,
                  self._rb_track_normal, self._rb_track_mirror):
            w.setEnabled(is_idle)

        if is_idle:
            self._timer_rec_countdown.stop()
            self._btn_rec.setText('▶  Start Episode')
            self._btn_rec.setStyleSheet(
                'background-color: #4CAF50; color: white; font-weight: bold;')
            self._btn_rec.setEnabled(True)
            self._lbl_episode.setText('—')
        elif prev == 'IDLE' and state == 'READY':
            self._btn_rec.setText('Starting in 3...')
            self._btn_rec.setStyleSheet(
                'background-color: #888; color: white; font-weight: bold;')
            self._btn_rec.setEnabled(False)
            self._rec_countdown = 3
            self._timer_rec_countdown.start(1000)
        else:
            self._btn_rec.setText('■  End Episode')
            self._btn_rec.setStyleSheet(
                'background-color: #E53935; color: white; font-weight: bold;')
            self._btn_rec.setEnabled(self._stream_on and state in ('READY', 'PAUSED'))

    def _tick_countdown(self):
        self._rec_countdown -= 1
        if self._rec_countdown > 0:
            self._btn_rec.setText(f'Starting in {self._rec_countdown}...')
        else:
            self._timer_rec_countdown.stop()
            self._btn_rec.setText('■  End Episode')
            self._btn_rec.setStyleSheet(
                'background-color: #E53935; color: white; font-weight: bold;')
            self._btn_rec.setEnabled(True)

    def _on_rec_episode(self, episode: int):
        self._lbl_episode.setText(str(episode) if episode >= 0 else '—')

    def _on_ik_mode_changed(self, _btn_id: int):
        self._pub_combined_mode()

    def _on_ctrl_mode_changed(self, _btn_id: int):
        self._pub_combined_mode()

    def _pub_combined_mode(self):
        ik   = 'sdk'  if self._bg_ik.checkedId()   == 0 else 'pink'
        ctrl = 'position' if self._bg_ctrl.checkedId() == 0 else 'impedance'
        self._node.pub_control_mode(f'{ik}_{ctrl}')

    def _sync_control_settings(self):
        self._pub_combined_mode()
        self._node.pub_mirror_mode(self._bg_track.checkedId() == 1)
        self._node.pub_task_id(self._spin_task.value())

    def _on_teleop_start(self):
        self._sync_control_settings()
        self._node.pub_rby1_cmd('teleop_start')

    def _on_mirror_mode_changed(self, btn_id: int):
        self._node.pub_mirror_mode(btn_id == 1)

    def _on_rec_btn(self):
        self._sync_control_settings()
        self._btn_rec.setEnabled(False)
        threading.Thread(
            target=self._node.call_toggle_episode,
            args=(lambda ok, _: self._btn_rec.setEnabled(True),),
            daemon=True,
        ).start()

    # ── Calibration ────────────────────────────────────────────────────────

    def _on_calib_status(self, text: str):
        self._lbl_calib.setText(f'Status: {text}')

    def _on_recalibrate(self):
        self._btn_calib.setEnabled(False)
        self._pbar_calib.setVisible(True)
        self._pbar_calib.setValue(0)
        self._sig.calib_status.emit('Calling service...')

        def done(ok, msg):
            if not ok:
                self._sig.calib_failed.emit(f'FAILED: {msg}')
            else:
                self._sig.calib_started.emit()

        threading.Thread(
            target=self._node.call_calibrate, args=(done,), daemon=True).start()

    def _on_calib_failed(self, msg: str):
        self._sig.calib_status.emit(msg)
        self._btn_calib.setEnabled(True)

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
        self._timer_calib = QTimer()
        self._timer_calib.timeout.connect(self._tick_calib)
        self._timer_calib.start(100)

    def _tick_calib(self):
        self._calib_elapsed += 0.1
        pct = int(((self._calib_phase - 1) * CALIB_DURATION + self._calib_elapsed)
                  / (CALIB_DURATION * 4) * 100)
        self._pbar_calib.setValue(min(pct, 100))

        if self._calib_elapsed >= CALIB_DURATION:
            self._calib_elapsed = 0.0
            if self._calib_phase < 4:
                self._calib_phase += 1
                self._sig.calib_status.emit(self._CALIB_PHASE_MSGS[self._calib_phase])
            else:
                self._timer_calib.stop()
                self._pbar_calib.setValue(100)
                self._sig.calib_status.emit('COMPLETE')
                self._btn_calib.setEnabled(True)


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
    window._pub_combined_mode()  # vive_rby1_node에 초기 IK 모드 전송

    ret = app.exec()
    ros_node.destroy_node()
    rclpy.shutdown()
    sys.exit(ret)


if __name__ == '__main__':
    main()
