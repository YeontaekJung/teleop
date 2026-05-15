"""
teleop_gui_node.py  (v3 — service-based interface)

Top:    RB-Y1 area  — connect, status, power/servo/pose controls
Middle: Joint Position — preset dropdown, joint inputs, execute, save
Bottom: Teleop area  — node status, pedal, teleop, recording, calibration
"""

import json
import os
import sys
import threading

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy, JointState
from std_msgs.msg import Int32, String
from std_srvs.srv import Trigger

from rby1_core_msgs.srv import (
    ConnectRobot, SetPower, SetServo, SetControlMode, MoveToJointPosition,
)
from scm_recording_msgs.srv import SetTeleOpPose

try:
    import yaml
except ImportError:
    yaml = None

from PySide6.QtWidgets import (
    QApplication, QWidget, QVBoxLayout, QHBoxLayout,
    QGroupBox, QLabel, QPushButton, QProgressBar, QGridLayout,
    QRadioButton, QButtonGroup, QSpinBox, QDoubleSpinBox, QLineEdit,
    QCheckBox, QComboBox, QInputDialog, QMessageBox, QScrollArea,
)
from PySide6.QtCore import Qt, QTimer, Signal, QObject
from PySide6.QtGui import QFont

CALIB_DURATION = 4.0  # seconds per phase — must match manus_inspire

BODY_JOINT_NAMES = [
    'torso_0', 'torso_1', 'torso_2', 'torso_3', 'torso_4', 'torso_5',
    'right_arm_0', 'right_arm_1', 'right_arm_2', 'right_arm_3',
    'right_arm_4', 'right_arm_5', 'right_arm_6',
    'left_arm_0', 'left_arm_1', 'left_arm_2', 'left_arm_3',
    'left_arm_4', 'left_arm_5', 'left_arm_6',
]

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
    'ARMING':    ('⬤ ARMING',    '#5555CC'),
    'READY':     ('⬤ READY',     '#F0C040'),
    'RECORDING': ('⬤ RECORDING', '#E0302A'),
    'PAUSED':    ('⬤ PAUSED',    '#E08020'),
}

_C_ON      = '#A6D256'
_C_OFF     = 'lightgray'
_C_OFF_RED = '#E53935'
_C_FAULT   = '#ED325A'

_USER_YAML = os.path.expanduser('~/.ros/teleop_gui_named_poses.yaml')


def _load_named_poses():
    """Load named_poses from user YAML, falling back to package default."""
    if yaml is None:
        return {}, 'ready'
    # Try user file first
    for path in [_USER_YAML]:
        if os.path.exists(path):
            try:
                with open(path) as f:
                    data = yaml.safe_load(f)
                return data.get('named_poses', {}), data.get('teleop_pose', 'ready')
            except Exception:
                pass
    # Fall back to package default
    try:
        pkg_yaml = os.path.join(
            os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
            'config', 'named_poses.yaml')
        if os.path.exists(pkg_yaml):
            with open(pkg_yaml) as f:
                data = yaml.safe_load(f)
            return data.get('named_poses', {}), data.get('teleop_pose', 'ready')
    except Exception:
        pass
    return {}, 'ready'


def _save_named_poses(named_poses: dict, teleop_pose: str):
    if yaml is None:
        return
    os.makedirs(os.path.dirname(_USER_YAML), exist_ok=True)
    with open(_USER_YAML, 'w') as f:
        yaml.safe_dump({'teleop_pose': teleop_pose, 'named_poses': named_poses}, f,
                       default_flow_style=False, allow_unicode=True)


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
        self._clutch_state_cbs    = []

        self.create_subscription(Joy,    '/teleop/pedal',          self._cb_pedal,          10)
        self.create_subscription(String, '/teleop/rec_state',      self._cb_rec_state,      10)
        self.create_subscription(Int32,  '/teleop/rec_episode',    self._cb_rec_episode,    10)
        self.create_subscription(String, '/teleop/tracker_status', self._cb_tracker_status, 10)
        self.create_subscription(String, '/rby1/state/status',     self._cb_rby1_status,    10)
        self.create_subscription(String, '/teleop/clutch_state',   self._cb_clutch_state,   10)
        self.create_timer(1.0, self._poll_nodes)

        # Service clients — rby1_rt
        self._cli_connect      = self.create_client(ConnectRobot,        '/rby1/connect')
        self._cli_power        = self.create_client(SetPower,            '/rby1/power')
        self._cli_servo        = self.create_client(SetServo,            '/rby1/servo')
        self._cli_ctrl_enable  = self.create_client(Trigger,             '/rby1/control_enable')
        self._cli_err_reset    = self.create_client(Trigger,             '/rby1/error_reset')
        self._cli_gripper_init = self.create_client(Trigger,             '/rby1/gripper_init')
        self._cli_stop_move    = self.create_client(Trigger,             '/rby1/stop_move')
        self._cli_ctrl_mode    = self.create_client(SetControlMode,      '/rby1/ctrl/mode')
        self._cli_move_joint   = self.create_client(MoveToJointPosition, '/rby1/move_to_joint_position')

        # Service clients — vive_rby1
        self._cli_teleop_start  = self.create_client(Trigger,       '/vive_rby1/teleop_start')
        self._cli_teleop_stop   = self.create_client(Trigger,       '/vive_rby1/teleop_stop')
        self._cli_toggle_clutch = self.create_client(Trigger,       '/vive_rby1/toggle_clutch')
        self._cli_set_pose      = self.create_client(SetTeleOpPose, '/vive_rby1/set_teleop_pose')
        self._cli_toggle_ep     = self.create_client(Trigger,       '/vive_rby1/toggle_episode')
        self._cli_calib         = self.create_client(Trigger,       '/manus_inspire/calibrate')

        self._pub_task_id   = self.create_publisher(Int32,  '/teleop/task_id',    10)
        self._pub_mirror    = self.create_publisher(String, '/teleop/mirror_mode', 10)

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

    def _cb_clutch_state(self, msg):
        for cb in self._clutch_state_cbs:
            cb(msg.data)

    def _poll_nodes(self):
        names  = {n for n, _ in self.get_node_names_and_namespaces()}
        status = {node: (node in names) for node, _ in NODES_TO_WATCH}
        for cb in self._node_status_cbs:
            cb(status)

    # ── publishers ─────────────────────────────────────────────────────────

    def pub_task_id(self, task_id: int):
        self._pub_task_id.publish(Int32(data=task_id))

    def pub_mirror_mode(self, mirror: bool):
        self._pub_mirror.publish(String(data='mirror' if mirror else 'normal'))

    # ── generic async service helper ────────────────────────────────────────

    def _call_async(self, client, request, done_cb=None):
        def _run():
            if not client.wait_for_service(timeout_sec=2.0):
                if done_cb:
                    done_cb(False, 'service not available')
                return
            fut = client.call_async(request)
            rclpy.spin_until_future_complete(self, fut, timeout_sec=30.0)
            if fut.done():
                res = fut.result()
                if done_cb:
                    done_cb(getattr(res, 'success', True), getattr(res, 'message', ''))
            else:
                if done_cb:
                    done_cb(False, 'timeout')
        threading.Thread(target=_run, daemon=True).start()

    # ── service call methods ────────────────────────────────────────────────

    def call_connect(self, host: str, no_gripper: bool, done_cb=None):
        req = ConnectRobot.Request()
        req.host = host
        req.no_gripper = no_gripper
        self._call_async(self._cli_connect, req, done_cb)

    def call_power(self, enable: bool, done_cb=None):
        req = SetPower.Request()
        req.enable = enable
        self._call_async(self._cli_power, req, done_cb)

    def call_servo(self, enable: bool, no_wheel: bool = False, done_cb=None):
        req = SetServo.Request()
        req.enable = enable
        req.no_wheel = no_wheel
        self._call_async(self._cli_servo, req, done_cb)

    def call_trigger(self, client, done_cb=None):
        self._call_async(client, Trigger.Request(), done_cb)

    def call_ctrl_mode(self, source: str, control: str, done_cb=None):
        req = SetControlMode.Request()
        req.source = source
        req.control = control
        self._call_async(self._cli_ctrl_mode, req, done_cb)

    def call_move_to_joint_position(self, positions: list, names: list,
                                     min_time: float = 5.0, done_cb=None):
        req = MoveToJointPosition.Request()
        req.target.name = names
        req.target.position = positions
        req.min_time = min_time
        self._call_async(self._cli_move_joint, req, done_cb)

    def call_set_teleop_pose(self, positions: list, names: list, done_cb=None):
        req = SetTeleOpPose.Request()
        req.pose.name = names
        req.pose.position = positions
        self._call_async(self._cli_set_pose, req, done_cb)

    def call_calibrate(self, done_cb):
        self.call_trigger(self._cli_calib, done_cb)

    def call_toggle_episode(self, done_cb):
        self.call_trigger(self._cli_toggle_ep, done_cb)


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
    clutch_state_changed   = Signal(str)
    service_result         = Signal(bool, str)


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

        # Load named poses
        self._named_poses, self._current_teleop_pose = _load_named_poses()

        signals.pedal_updated.connect(self._on_pedal)
        signals.node_status_updated.connect(self._on_nodes)
        signals.calib_status.connect(self._on_calib_status)
        signals.calib_started.connect(self._start_calib_progress)
        signals.calib_failed.connect(self._on_calib_failed)
        signals.rec_state_changed.connect(self._on_rec_state)
        signals.rec_episode_changed.connect(self._on_rec_episode)
        signals.tracker_status_changed.connect(self._on_tracker_status)
        signals.rby1_status_changed.connect(self._on_rby1_status)
        signals.clutch_state_changed.connect(self._on_clutch_state)
        signals.service_result.connect(self._on_service_result)

        self._build_ui()

    # ── UI construction ────────────────────────────────────────────────────

    def _build_ui(self):
        self.setWindowTitle('Teleop Control')
        self.setMinimumWidth(1100)

        root = QVBoxLayout()
        root.setSpacing(6)
        root.addWidget(self._build_rby1_group())
        root.addWidget(self._build_joint_position_group())
        root.addWidget(self._build_teleop_group())
        self.setLayout(root)

    # ── RB-Y1 group ────────────────────────────────────────────────────────

    def _build_rby1_group(self) -> QGroupBox:
        group = QGroupBox('RB-Y1')
        vbox  = QVBoxLayout()
        vbox.setSpacing(5)
        vbox.addLayout(self._build_status_row())
        vbox.addLayout(self._build_connect_row())

        bottom = QHBoxLayout()
        bottom.setSpacing(8)
        left = QVBoxLayout()
        left.setSpacing(4)
        left.addLayout(self._build_init_row())
        bottom.addLayout(left, 1)

        btn_stop = QPushButton('⚠  STOP\nMOVE')
        btn_stop.setFixedSize(110, 70)
        btn_stop.setStyleSheet(
            'background-color: #FFD600; color: #000000;'
            'font-weight: bold; font-size: 13px;')
        btn_stop.clicked.connect(
            lambda: self._node.call_trigger(self._node._cli_stop_move))
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
        self._lbl_ctr_type = _make_status_label('Mode')
        for lbl in (self._lbl_power, self._lbl_servo, self._lbl_control,
                    self._lbl_stream, self._lbl_gripper, self._lbl_ctr_type):
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

        def _btn(label, color, fn):
            b = _make_btn(label, color, height=30)
            b.clicked.connect(fn)
            row.addWidget(b)
            return b

        _btn('Power On',    '#388E3C', lambda: self._node.call_power(True))
        _btn('Power Off',   '#C62828', lambda: self._node.call_power(False))
        _btn('Servo On',    '#1976D2', lambda: self._node.call_servo(True))
        _btn('Err Reset',   '#F57C00', lambda: self._node.call_trigger(self._node._cli_err_reset))
        _btn('Ctrl Enable', '#7B1FA2', lambda: self._node.call_trigger(self._node._cli_ctrl_enable))
        _btn('Gripper Init','#00838F', lambda: self._node.call_trigger(self._node._cli_gripper_init))
        row.addStretch()
        return row

    # ── Joint Position group ───────────────────────────────────────────────

    def _build_joint_position_group(self) -> QGroupBox:
        group = QGroupBox('Joint Position')
        main_row = QHBoxLayout()
        main_row.setSpacing(10)

        # Left: preset dropdown + joint fields
        left_vbox = QVBoxLayout()
        left_vbox.setSpacing(4)

        preset_row = QHBoxLayout()
        self._cmb_preset = QComboBox()
        self._cmb_preset.setMinimumWidth(140)
        for name in self._named_poses:
            self._cmb_preset.addItem(name)
        self._cmb_preset.currentTextChanged.connect(self._on_preset_selected)
        self._btn_execute = _make_btn('Execute', '#2E7D32', height=30)
        self._btn_execute.clicked.connect(self._on_execute_joint)
        preset_row.addWidget(QLabel('Preset:'))
        preset_row.addWidget(self._cmb_preset)
        preset_row.addWidget(self._btn_execute)
        preset_row.addStretch()
        left_vbox.addLayout(preset_row)

        # Joint input grid (20 joints, 2 columns of 10)
        self._joint_spins = {}
        joint_grid = QGridLayout()
        joint_grid.setSpacing(3)
        for idx, name in enumerate(BODY_JOINT_NAMES):
            col_base = (idx // 10) * 2
            row_pos  = idx % 10
            lbl = QLabel(name)
            lbl.setFont(QFont('Monospace', 8))
            spin = QDoubleSpinBox()
            spin.setDecimals(4)
            spin.setRange(-6.28, 6.28)
            spin.setSingleStep(0.01)
            spin.setFixedWidth(90)
            joint_grid.addWidget(lbl,  row_pos, col_base)
            joint_grid.addWidget(spin, row_pos, col_base + 1)
            self._joint_spins[name] = spin
        left_vbox.addLayout(joint_grid)

        # Save preset row
        save_row = QHBoxLayout()
        save_row.addWidget(QLabel('Save as:'))
        self._le_preset_name = QLineEdit()
        self._le_preset_name.setPlaceholderText('preset name')
        self._le_preset_name.setFixedWidth(130)
        self._btn_save_preset = _make_btn('Save', '#546E7A', height=28)
        self._btn_save_preset.clicked.connect(self._on_save_preset)
        save_row.addWidget(self._le_preset_name)
        save_row.addWidget(self._btn_save_preset)
        save_row.addStretch()
        left_vbox.addLayout(save_row)
        main_row.addLayout(left_vbox, 3)

        # Right: Teleop Pose selector
        right_vbox = QVBoxLayout()
        right_vbox.setSpacing(6)
        right_vbox.addWidget(QLabel('Teleop Start/Stop Pose:'))
        self._cmb_teleop_pose = QComboBox()
        self._cmb_teleop_pose.setMinimumWidth(140)
        for name in self._named_poses:
            self._cmb_teleop_pose.addItem(name)
        if self._current_teleop_pose in self._named_poses:
            self._cmb_teleop_pose.setCurrentText(self._current_teleop_pose)
        self._cmb_teleop_pose.currentTextChanged.connect(self._on_teleop_pose_changed)
        right_vbox.addWidget(self._cmb_teleop_pose)
        right_vbox.addStretch()
        main_row.addLayout(right_vbox, 1)

        group.setLayout(main_row)

        # Seed fields from current teleop_pose if available
        if self._current_teleop_pose in self._named_poses:
            self._fill_joint_fields(self._current_teleop_pose)

        return group

    def _fill_joint_fields(self, preset_name: str):
        if preset_name not in self._named_poses:
            return
        pose = self._named_poses[preset_name]
        names = pose.get('joint_names', [])
        positions = pose.get('positions', [])
        for name, val in zip(names, positions):
            if name in self._joint_spins:
                self._joint_spins[name].setValue(val)

    def _on_preset_selected(self, name: str):
        self._fill_joint_fields(name)

    def _on_execute_joint(self):
        positions = [self._joint_spins[n].value() for n in BODY_JOINT_NAMES]
        self._btn_execute.setEnabled(False)
        self._node.call_move_to_joint_position(
            positions, BODY_JOINT_NAMES, min_time=5.0,
            done_cb=lambda ok, msg: self._sig.service_result.emit(ok, msg))
        QTimer.singleShot(1000, lambda: self._btn_execute.setEnabled(True))

    def _on_save_preset(self):
        name = self._le_preset_name.text().strip()
        if not name:
            return
        positions = [self._joint_spins[n].value() for n in BODY_JOINT_NAMES]
        self._named_poses[name] = {
            'joint_names': list(BODY_JOINT_NAMES),
            'positions': positions,
        }
        _save_named_poses(self._named_poses, self._cmb_teleop_pose.currentText())
        # Update dropdowns
        for cmb in (self._cmb_preset, self._cmb_teleop_pose):
            if cmb.findText(name) < 0:
                cmb.addItem(name)
        self._cmb_preset.setCurrentText(name)
        self._le_preset_name.clear()

    def _on_teleop_pose_changed(self, name: str):
        if name not in self._named_poses:
            return
        self._current_teleop_pose = name
        _save_named_poses(self._named_poses, name)
        pose = self._named_poses[name]
        self._node.call_set_teleop_pose(
            pose.get('positions', []), pose.get('joint_names', []))

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
        for label in ['Resume/Pause', 'Discard', '● Rec']:
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
        btn_ts = _make_btn('▶  Teleop Start', '#4CAF50', height=34)
        btn_ts.clicked.connect(
            lambda: self._node.call_trigger(self._node._cli_teleop_start))
        btn_tp = _make_btn('■  Teleop Stop',  '#E53935', height=34)
        btn_tp.clicked.connect(
            lambda: self._node.call_trigger(self._node._cli_teleop_stop))
        teleop_row.addWidget(btn_ts)
        teleop_row.addWidget(btn_tp)
        vbox.addLayout(teleop_row)

        clutch_row = QHBoxLayout()
        clutch_row.setSpacing(5)
        self._lbl_clutch = QLabel('Disengaged')
        self._lbl_clutch.setAlignment(Qt.AlignCenter)
        self._lbl_clutch.setFixedHeight(28)
        self._lbl_clutch.setStyleSheet(
            'background-color: #E53935; color: white; border-radius: 4px; padding: 0 6px;')
        self._btn_clutch_toggle = _make_btn('⚙  Clutch Toggle', '#F57C00', height=28)
        self._btn_clutch_toggle.clicked.connect(
            lambda: self._node.call_trigger(self._node._cli_toggle_clutch))
        clutch_row.addWidget(self._lbl_clutch, 1)
        clutch_row.addWidget(self._btn_clutch_toggle, 2)
        vbox.addLayout(clutch_row)

        vbox.addSpacing(4)

        ctrl_row = QHBoxLayout()
        ctrl_row.addWidget(QLabel('Ctrl:'))
        self._rb_ctrl_position  = QRadioButton('Position')
        self._rb_ctrl_impedance = QRadioButton('Impedance')
        self._rb_ctrl_impedance.setChecked(True)
        self._bg_ctrl = QButtonGroup()
        self._bg_ctrl.addButton(self._rb_ctrl_position,  0)
        self._bg_ctrl.addButton(self._rb_ctrl_impedance, 1)
        self._bg_ctrl.idClicked.connect(self._on_ctrl_mode_changed)
        ctrl_row.addWidget(self._rb_ctrl_position)
        ctrl_row.addWidget(self._rb_ctrl_impedance)
        ctrl_row.addStretch()
        vbox.addLayout(ctrl_row)

        src_row = QHBoxLayout()
        src_row.addWidget(QLabel('Src:'))
        self._rb_src_joint     = QRadioButton('Joint')
        self._rb_src_cartesian = QRadioButton('Cartesian')
        self._rb_src_cartesian.setChecked(True)
        self._bg_src = QButtonGroup()
        self._bg_src.addButton(self._rb_src_joint,     0)
        self._bg_src.addButton(self._rb_src_cartesian, 1)
        self._bg_src.idClicked.connect(self._on_ctrl_mode_changed)
        src_row.addWidget(self._rb_src_joint)
        src_row.addWidget(self._rb_src_cartesian)
        src_row.addStretch()
        vbox.addLayout(src_row)

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

        task_row = QHBoxLayout()
        task_row.addWidget(QLabel('task_id'))
        self._spin_task = QSpinBox()
        self._spin_task.setMinimum(0)
        self._spin_task.setMaximum(9999)
        self._spin_task.setFixedWidth(70)
        self._spin_task.valueChanged.connect(lambda v: self._node.pub_task_id(v))
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
        host = self._le_ip.text().strip()
        no_gripper = self._chk_no_gripper.isChecked()
        self._btn_connect.setEnabled(False)
        self._node.call_connect(
            host, no_gripper,
            done_cb=lambda ok, msg: (
                self._sig.service_result.emit(ok, f'connect: {msg}'),
                QTimer.singleShot(500, lambda: self._btn_connect.setEnabled(True)),
            ))

    def _on_service_result(self, ok: bool, msg: str):
        if not ok:
            self._node.get_logger().warn(f'[GUI] service result: {msg}')

    def _on_rby1_status(self, data: dict):
        power   = data.get('power_state',   'False') == 'True'
        servo   = data.get('servo_state',   'False') == 'True'
        stream  = data.get('stream_state',  'False') == 'True'
        gripper = data.get('gripper_state', 'False') == 'True'
        ctrl    = data.get('control_state', '')
        ctr_type = data.get('ctr_type', '')

        def _set(lbl, text, color):
            lbl.setText(f'  {text}  ')
            lbl.setStyleSheet(f'background-color: {color}; border-radius: 4px;')

        _set(self._lbl_power,    'Power On'  if power   else 'Power Off',  _C_ON if power   else _C_OFF_RED)
        _set(self._lbl_servo,    'Servo On'  if servo   else 'Servo Off',  _C_ON if servo   else _C_OFF_RED)
        _set(self._lbl_stream,   'Stream On' if stream  else 'Stream Off', _C_ON if stream  else _C_OFF_RED)
        _set(self._lbl_gripper,  'Gripper ✓' if gripper else 'Gripper ✗',  _C_ON if gripper else _C_OFF)
        _set(self._lbl_ctr_type, ctr_type or '—', _C_ON if stream else _C_OFF)

        if stream != self._stream_on:
            self._stream_on = stream

        if ctrl == 'State.Enabled':
            _set(self._lbl_control, 'Enabled', _C_ON)
        elif 'Fault' in ctrl:
            _set(self._lbl_control, 'FAULT', _C_FAULT)
        else:
            _set(self._lbl_control, 'Idle', _C_OFF)

    def _on_pedal(self, state: list):
        for i, (btn, pressed) in enumerate(zip(self._btn_pedals, state)):
            color = '#A6D256' if pressed else '#ccc'
            btn.setStyleSheet(f'background-color: {color}; color: #333;')

    def _on_nodes(self, status: dict):
        for node, alive in status.items():
            if node in self._node_dots:
                color = '#A6D256' if alive else '#ED325A'
                self._node_dots[node].setStyleSheet(f'color: {color};')

    def _on_tracker_status(self, sl: str, sr: str):
        _colors = {'OK': '#4CAF50', 'JITTER': '#F0C040', 'LOST': '#E0302A'}
        self._lbl_tracker_l.setStyleSheet(f'color: {_colors.get(sl, "#888")};')
        self._lbl_tracker_r.setStyleSheet(f'color: {_colors.get(sr, "#888")};')

    def _on_clutch_state(self, state: str):
        engaged = (state == 'ENGAGED')
        self._lbl_clutch.setText('Engaged' if engaged else 'Disengaged')
        color = '#4CAF50' if engaged else '#E53935'
        self._lbl_clutch.setStyleSheet(
            f'background-color: {color}; color: white; border-radius: 4px; padding: 0 6px;')

    def _on_rec_state(self, state: str):
        self._rec_state = state
        text, color = REC_STATE_STYLE.get(state, ('⬤ ' + state, '#888'))
        self._lbl_rec_state.setText(text)
        self._lbl_rec_state.setStyleSheet(f'color: {color};')

        is_idle = (state == 'IDLE')
        for w in (self._spin_task,
                  self._rb_ctrl_position, self._rb_ctrl_impedance,
                  self._rb_src_joint, self._rb_src_cartesian,
                  self._rb_track_normal, self._rb_track_mirror):
            w.setEnabled(is_idle)

        if is_idle:
            self._btn_rec.setText('▶  Start Episode')
            self._btn_rec.setStyleSheet(
                'background-color: #4CAF50; color: white; font-weight: bold;')
            self._btn_rec.setEnabled(True)
            self._lbl_episode.setText('—')
        elif state == 'ARMING':
            self._btn_rec.setText('Warming up...')
            self._btn_rec.setStyleSheet(
                'background-color: #888; color: white; font-weight: bold;')
            self._btn_rec.setEnabled(False)
        else:
            self._btn_rec.setText('■  End Episode')
            self._btn_rec.setStyleSheet(
                'background-color: #E53935; color: white; font-weight: bold;')
            self._btn_rec.setEnabled(state in ('READY', 'PAUSED'))

    def _on_rec_episode(self, episode: int):
        self._lbl_episode.setText(str(episode) if episode >= 0 else '—')

    def _on_ctrl_mode_changed(self, _btn_id: int):
        src  = 'joint'     if self._bg_src.checkedId()  == 0 else 'cartesian'
        ctrl = 'position'  if self._bg_ctrl.checkedId() == 0 else 'impedance'
        self._node.call_ctrl_mode(src, ctrl)

    def _on_mirror_mode_changed(self, btn_id: int):
        self._node.pub_mirror_mode(btn_id == 1)

    def _on_rec_btn(self):
        self._node.pub_task_id(self._spin_task.value())
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
    ros_node._clutch_state_cbs.append(   lambda s:    signals.clutch_state_changed.emit(s))

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
