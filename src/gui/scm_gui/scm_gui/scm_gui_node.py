"""
scm_gui_node.py  (v3 — service-based interface)

Top:    RB-Y1 area  — connect, status, power/servo/pose controls
Middle: Joint Position — preset dropdown, joint inputs, execute, save
Bottom: Teleop area  — node status, pedal, teleop, recording, calibration
"""

import json
import math
import os
import sys
import threading
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy, JointState
from std_msgs.msg import Int32, String
from std_srvs.srv import SetBool, Trigger

from rby1_core_msgs.srv import (
    ConnectRobot, SetPower, SetServo, SetControlMode, MoveToJointPosition,
    SetCartesianJointLimits, SetNullspaceWeight, SetNullspaceJointRef,
)
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter as RosParameter, ParameterValue, ParameterType
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

CORE_NODES = [
    ('rby1_core_node', 'rby1_core'),
]
TELEOP_NODES = [
    ('pedal_node',           'pedal_ros2'),
    ('vive_tracker_node',    'vive_ros2'),
    ('manus_data_publisher', 'manus_ros2'),
    ('vive_rby1_node',       'vive_rby1'),
    ('manus_inspire',        'manus_inspire'),
]
RECORDING_NODES = [
    ('scm_recording', 'recorder'),
]
NODES_TO_WATCH = CORE_NODES + TELEOP_NODES + RECORDING_NODES

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

def _get_config_yaml():
    try:
        from ament_index_python.packages import get_package_share_directory
        return os.path.join(
            get_package_share_directory('scm_gui'), 'config', 'named_poses.yaml')
    except Exception:
        pass
    return os.path.join(
        os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
        'config', 'named_poses.yaml')

_CONFIG_YAML = _get_config_yaml()


def _get_impedance_presets_yaml():
    try:
        from ament_index_python.packages import get_package_share_directory
        return os.path.join(
            get_package_share_directory('scm_gui'), 'config', 'impedance_presets.yaml')
    except Exception:
        pass
    return os.path.join(
        os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
        'config', 'impedance_presets.yaml')

_IMPEDANCE_PRESETS_YAML = _get_impedance_presets_yaml()


def _load_impedance_presets() -> dict:
    if yaml is None:
        return {}
    if os.path.exists(_IMPEDANCE_PRESETS_YAML):
        try:
            with open(_IMPEDANCE_PRESETS_YAML) as f:
                data = yaml.safe_load(f)
            return data.get('impedance_presets', {}) if data else {}
        except Exception:
            pass
    return {}


def _save_impedance_presets(presets: dict):
    if yaml is None:
        return
    with open(_IMPEDANCE_PRESETS_YAML, 'w') as f:
        yaml.safe_dump({'impedance_presets': presets},
                       f, default_flow_style=False, allow_unicode=True)


def _load_named_poses():
    if yaml is None:
        return {}, 'ready', 'a'
    if os.path.exists(_CONFIG_YAML):
        try:
            with open(_CONFIG_YAML) as f:
                data = yaml.safe_load(f)
            return (data.get('named_poses', {}),
                    data.get('teleop_pose', 'ready'),
                    data.get('robot_model', 'a'))
        except Exception:
            pass
    return {}, 'ready', 'a'


def _save_named_poses(named_poses: dict, teleop_pose: str, robot_model: str = 'a'):
    if yaml is None:
        return
    with open(_CONFIG_YAML, 'w') as f:
        yaml.safe_dump({
            'teleop_pose': teleop_pose,
            'named_poses': named_poses,
            'robot_model': robot_model,
        }, f, default_flow_style=False, allow_unicode=True)


# ---------------------------------------------------------------------------
# ROS2 node (background thread)
# ---------------------------------------------------------------------------

class ScmGuiNode(Node):

    def __init__(self):
        super().__init__('scm_gui')
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
        self.create_subscription(JointState, '/rby1/state/joint', self._cb_joint_state, 10)
        self._latest_joint_state = None
        self._next_joint_cb = None
        self.create_timer(1.0, self._poll_nodes)

        # Service clients — rby1_core
        self._cli_connect      = self.create_client(ConnectRobot,        '/rby1/connect')
        self._cli_power        = self.create_client(SetPower,            '/rby1/power')
        self._cli_servo        = self.create_client(SetServo,            '/rby1/servo')
        self._cli_ctrl_enable  = self.create_client(Trigger,             '/rby1/control_enable')
        self._cli_err_reset    = self.create_client(Trigger,             '/rby1/error_reset')
        self._cli_gripper_init = self.create_client(Trigger,             '/rby1/gripper_init')
        self._cli_stop_move    = self.create_client(Trigger,             '/rby1/stop_move')
        self._cli_ctrl_mode    = self.create_client(SetControlMode,      '/rby1/ctrl/mode')
        self._cli_move_joint   = self.create_client(MoveToJointPosition, '/rby1/move_to_joint_position')
        self._cli_set_param    = self.create_client(SetParameters,        '/rby1_core_node/set_parameters')
        self._cli_set_ci_limits = self.create_client(SetCartesianJointLimits, '/rby1/set_cartesian_joint_limits')
        self._cli_set_ns_weight = self.create_client(SetNullspaceWeight,       '/rby1/set_nullspace_weight')
        self._cli_set_ns_ref    = self.create_client(SetNullspaceJointRef,     '/rby1/set_nullspace_joint_ref')

        # Service clients — vive_rby1
        self._cli_set_use_torso = self.create_client(SetBool,       '/vive_rby1/set_use_torso')
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
        sl = parts[0].split(':')[1] if len(parts) > 0 else ''
        sr = parts[1].split(':')[1] if len(parts) > 1 else ''
        sb = parts[2].split(':')[1] if len(parts) > 2 and parts[2].startswith('sb:') else ''
        for cb in self._tracker_status_cbs:
            cb(sl, sr, sb)

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

    def _cb_joint_state(self, msg):
        self._latest_joint_state = msg
        if self._next_joint_cb is not None:
            cb = self._next_joint_cb
            self._next_joint_cb = None
            cb(list(msg.name), list(msg.position))

    def get_latest_joint_state(self):
        return self._latest_joint_state

    def request_next_joint_state(self, cb):
        self._next_joint_cb = cb

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

    def _call_async(self, client, request, done_cb=None, timeout_sec=30.0):
        def _run():
            if not client.wait_for_service(timeout_sec=2.0):
                if done_cb:
                    done_cb(False, 'service not available')
                return
            fut = client.call_async(request)
            deadline = time.monotonic() + timeout_sec
            while not fut.done():
                if time.monotonic() > deadline:
                    if done_cb:
                        done_cb(False, 'timeout')
                    return
                time.sleep(0.02)
            try:
                res = fut.result()
                if done_cb:
                    done_cb(getattr(res, 'success', True), getattr(res, 'message', ''))
            except Exception as e:
                if done_cb:
                    done_cb(False, str(e))
        threading.Thread(target=_run, daemon=True).start()

    # ── service call methods ────────────────────────────────────────────────

    def set_robot_model(self, model: str, done_cb=None):
        """Set robot_model parameter on rby1_core_node (must be called before connect)."""
        def _run():
            try:
                if not self._cli_set_param.wait_for_service(timeout_sec=2.0):
                    if done_cb:
                        done_cb(False, 'rby1_core_node param service not available')
                    return
                pv = ParameterValue()
                pv.type = ParameterType.PARAMETER_STRING
                pv.string_value = model
                p = RosParameter()
                p.name = 'robot_model'
                p.value = pv
                req = SetParameters.Request()
                req.parameters = [p]
                future = self._cli_set_param.call_async(req)
                deadline = time.monotonic() + 5.0
                while not future.done():
                    if time.monotonic() > deadline:
                        if done_cb:
                            done_cb(False, 'set_robot_model timeout')
                        return
                    time.sleep(0.02)
                result = future.result()
                ok = all(r.successful for r in result.results)
                reason = next((r.reason for r in result.results if not r.successful), '')
                if done_cb:
                    done_cb(ok, reason)
            except Exception as e:
                if done_cb:
                    done_cb(False, str(e))
        threading.Thread(target=_run, daemon=True).start()

    def call_connect(self, host: str, no_gripper: bool, done_cb=None):
        req = ConnectRobot.Request()
        req.host = host
        req.no_gripper = no_gripper
        def _run():
            if not self._cli_connect.wait_for_service(timeout_sec=2.0):
                if done_cb:
                    done_cb(False, 'service not available', [], [], [])
                return
            fut = self._cli_connect.call_async(req)
            deadline = time.monotonic() + 15.0
            while not fut.done():
                if time.monotonic() > deadline:
                    if done_cb:
                        done_cb(False, 'timeout', [], [], [])
                    return
                time.sleep(0.02)
            try:
                res = fut.result()
                names = list(res.joint_names) if res.success else []
                lower = list(res.q_lower)     if res.success else []
                upper = list(res.q_upper)     if res.success else []
                if done_cb:
                    done_cb(res.success, res.message, names, lower, upper)
            except Exception as e:
                if done_cb:
                    done_cb(False, str(e), [], [], [])
        threading.Thread(target=_run, daemon=True).start()

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

    def call_set_use_torso(self, enable: bool, done_cb=None):
        req = SetBool.Request()
        req.data = enable
        self._call_async(self._cli_set_use_torso, req, done_cb)

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

    def call_set_ci_limits(self, joint_names: list, min_rads: list,
                           max_rads: list, done_cb=None):
        req = SetCartesianJointLimits.Request()
        req.joint_names = list(joint_names)
        req.min_rad = [float(v) for v in min_rads]
        req.max_rad = [float(v) for v in max_rads]
        self._call_async(self._cli_set_ci_limits, req, done_cb)

    def call_set_ns_weight(self, joint_names: list, weights: list, done_cb=None):
        req = SetNullspaceWeight.Request()
        req.joint_names = list(joint_names)
        req.weights = [float(v) for v in weights]
        self._call_async(self._cli_set_ns_weight, req, done_cb)

    def call_set_nullspace_joint_ref(self, positions: list, names: list, done_cb=None):
        req = SetNullspaceJointRef.Request()
        req.target.name = list(names)
        req.target.position = [float(v) for v in positions]
        self._call_async(self._cli_set_ns_ref, req, done_cb)


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
    tracker_status_changed = Signal(str, str, str)   # sl, sr, sb(body)
    rby1_status_changed    = Signal(dict)
    clutch_state_changed   = Signal(str)
    service_result         = Signal(bool, str)
    connect_result         = Signal(bool, str)
    execute_done           = Signal(bool, str)
    joint_state_received   = Signal(list, list)
    joint_limits_received  = Signal(object)
    dispatch               = Signal(object)   # call a callable on the main thread


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

    def __init__(self, ros_node: ScmGuiNode, signals: Signals):
        super().__init__()
        self._node      = ros_node
        self._sig       = signals
        self._rec_state = 'IDLE'
        self._stream_on = False

        # Load named poses and persisted settings
        self._named_poses, self._current_teleop_pose, self._current_robot_model = _load_named_poses()
        self._imp_presets = _load_impedance_presets()

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
        signals.connect_result.connect(self._on_connect_result)
        signals.joint_state_received.connect(self._on_joint_state_received)
        signals.joint_limits_received.connect(self._on_joint_limits_received)

        self._joint_limits = {}

        self._build_ui()

    # ── UI construction ────────────────────────────────────────────────────

    def _build_ui(self):
        self.setWindowTitle('SCM Control')
        self.setMinimumWidth(1600)

        root = QHBoxLayout()
        root.setSpacing(8)
        root.setContentsMargins(6, 6, 6, 6)

        left_widget = QWidget()
        left_widget.setMinimumWidth(600)
        left_col = QVBoxLayout(left_widget)
        left_col.setSpacing(6)
        left_col.setContentsMargins(0, 0, 0, 0)
        left_col.addWidget(self._build_rby1_group())
        left_col.addWidget(self._build_node_indicators_group())
        left_col.addWidget(self._build_teleop_group())
        left_col.addStretch()

        right_widget = QWidget()
        right_widget.setMinimumWidth(800)
        right_col = QVBoxLayout(right_widget)
        right_col.setSpacing(6)
        right_col.setContentsMargins(0, 0, 0, 0)
        right_col.addWidget(self._build_joint_position_group())
        right_col.addWidget(self._build_cartesian_impedance_group())
        right_col.addStretch()

        root.addWidget(left_widget, stretch=1)
        root.addWidget(right_widget, stretch=1)

        self.setLayout(root)

    # ── RB-Y1 group ────────────────────────────────────────────────────────

    def _build_rby1_group(self) -> QGroupBox:
        group = QGroupBox('RB-Y1')
        vbox  = QVBoxLayout()
        vbox.setSpacing(5)
        vbox.addLayout(self._build_status_row())

        self._btn_stop = QPushButton('⚠  STOP\nMOVE')
        self._btn_stop.setFixedSize(110, 70)
        self._btn_stop.setStyleSheet(
            'background-color: #FFD600; color: #000000;'
            'font-weight: bold; font-size: 13px;')
        self._btn_stop.clicked.connect(self._on_stop_move)

        main_row = QHBoxLayout()
        main_row.setSpacing(8)
        main_row.addLayout(self._build_connect_settings_area())
        main_row.addLayout(self._build_buttons_area())
        main_row.addStretch()
        main_row.addWidget(self._btn_stop)

        vbox.addLayout(main_row)
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

    def _build_connect_settings_area(self) -> QVBoxLayout:
        outer = QVBoxLayout()
        outer.setSpacing(4)

        # Row 1: Sim/Real 선택 + IP 입력
        row1 = QHBoxLayout()
        row1.setSpacing(6)

        self._rb_sim  = QRadioButton('Sim')
        self._rb_real = QRadioButton('Real')
        self._rb_sim.setChecked(True)
        self._bg_conn = QButtonGroup(self)
        self._bg_conn.addButton(self._rb_sim,  0)
        self._bg_conn.addButton(self._rb_real, 1)
        self._bg_conn.idClicked.connect(self._on_sim_real_changed)

        self._le_ip = QLineEdit('localhost:50051')
        self._le_ip.setFixedWidth(140)

        row1.addWidget(self._rb_sim)
        row1.addWidget(self._rb_real)
        row1.addWidget(self._le_ip)

        # Row 2: Model 선택 + No Gripper
        row2 = QHBoxLayout()
        row2.setSpacing(6)

        self._rb_model_a = QRadioButton('Model A')
        self._rb_model_m = QRadioButton('Model M')
        self._rb_model_a.setChecked(self._current_robot_model != 'm')
        self._rb_model_m.setChecked(self._current_robot_model == 'm')
        self._bg_model = QButtonGroup(self)
        self._bg_model.addButton(self._rb_model_a, 0)
        self._bg_model.addButton(self._rb_model_m, 1)
        self._bg_model.idClicked.connect(self._on_model_changed)

        self._chk_no_gripper = QCheckBox('No Gripper')
        self._chk_no_gripper.setChecked(True)

        row2.addWidget(self._rb_model_a)
        row2.addWidget(self._rb_model_m)
        row2.addWidget(self._chk_no_gripper)

        outer.addLayout(row1)
        outer.addLayout(row2)
        return outer

    def _build_buttons_area(self) -> QHBoxLayout:
        row = QHBoxLayout()
        row.setSpacing(8)

        self._btn_connect = _make_btn('Connect', '#1565C0', height=30)
        self._btn_connect.setFixedWidth(90)
        self._btn_connect.clicked.connect(self._on_connect)

        btn_grid = QGridLayout()
        btn_grid.setSpacing(3)

        # Row 0: all main buttons at the same height
        btn_grid.addWidget(self._btn_connect, 0, 0)
        btn_grid.addWidget(self._make_btn_with_fb('Power On',  '#388E3C',
            lambda cb: self._node.call_power(True,  done_cb=cb),
            lambda ok, _: self._update_lbl(self._lbl_power,   'Power On',  _C_ON)      if ok else None), 0, 1)
        btn_grid.addWidget(self._make_btn_with_fb('Servo On',  '#1976D2',
            lambda cb: self._node.call_servo(True,  done_cb=cb),
            lambda ok, _: self._update_lbl(self._lbl_servo,   'Servo On',  _C_ON)      if ok else None), 0, 2)
        btn_grid.addWidget(self._make_btn_with_fb('Ctrl Enable', '#7B1FA2',
            lambda cb: self._node.call_trigger(self._node._cli_ctrl_enable, done_cb=cb),
            lambda ok, _: self._update_lbl(self._lbl_control, 'Enabled',   _C_ON)      if ok else None), 0, 3)
        btn_grid.addWidget(self._make_btn_with_fb('Gripper Init', '#00838F',
            lambda cb: self._node.call_trigger(self._node._cli_gripper_init, done_cb=cb)), 0, 4)

        # Row 1: off buttons directly below their on counterparts
        btn_grid.addWidget(self._make_btn_with_fb('Power Off', '#C62828',
            lambda cb: self._node.call_power(False, done_cb=cb),
            lambda ok, _: self._update_lbl(self._lbl_power,   'Power Off', _C_OFF_RED) if ok else None), 1, 1)
        btn_grid.addWidget(self._make_btn_with_fb('Servo Off', '#5C6BC0',
            lambda cb: self._node.call_servo(False, done_cb=cb),
            lambda ok, _: self._update_lbl(self._lbl_servo,   'Servo Off', _C_OFF_RED) if ok else None), 1, 2)
        btn_grid.addWidget(self._make_btn_with_fb('Err Reset',   '#F57C00',
            lambda cb: self._node.call_trigger(self._node._cli_err_reset,   done_cb=cb)), 1, 3)

        row.addLayout(btn_grid)
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
        self._sig.execute_done.connect(self._on_execute_done)
        self._btn_load_current = _make_btn('↓ Load Current Pos', '#795548', height=30)
        self._btn_load_current.clicked.connect(self._on_load_current_joints)
        self._le_preset_name = QLineEdit()
        self._le_preset_name.setPlaceholderText('preset name')
        self._le_preset_name.setFixedWidth(110)
        self._btn_save_preset = _make_btn('Save', '#546E7A', height=30)
        self._btn_save_preset.clicked.connect(self._on_save_preset)
        preset_row.addWidget(QLabel('Preset:'))
        preset_row.addWidget(self._cmb_preset)
        preset_row.addWidget(self._btn_execute)
        preset_row.addWidget(self._btn_load_current)
        preset_row.addSpacing(20)
        preset_row.addWidget(QLabel('Save as:'))
        preset_row.addWidget(self._le_preset_name)
        preset_row.addWidget(self._btn_save_preset)
        preset_row.addStretch()
        left_vbox.addLayout(preset_row)

        # Joint input grid — 3 columns: Torso | Right Arm | Left Arm
        self._joint_spins = {}
        self._joint_deg_labels = {}
        self._joint_limit_labels = {}
        self._filling_preset = False
        _groups = [
            ('Torso',     BODY_JOINT_NAMES[:6]),
            ('Right Arm', BODY_JOINT_NAMES[6:13]),
            ('Left Arm',  BODY_JOINT_NAMES[13:]),
        ]
        cols_layout = QHBoxLayout()
        cols_layout.setSpacing(12)

        def _make_deg_updater(label, sp):
            def _upd(v):
                label.setText(f'{math.degrees(v):+.1f}°')
            sp.valueChanged.connect(_upd)
            _upd(sp.value())

        for group_name, joints in _groups:
            col_vbox = QVBoxLayout()
            col_vbox.setSpacing(2)
            hdr = QLabel(group_name)
            hdr_font = QFont('Monospace', 9)
            hdr_font.setBold(True)
            hdr.setFont(hdr_font)
            hdr.setAlignment(Qt.AlignCenter)
            hdr.setStyleSheet(
                'background-color: #37474F; color: white; padding: 2px; border-radius: 3px;')
            col_vbox.addWidget(hdr)
            for name in joints:
                jrow = QHBoxLayout()
                jrow.setSpacing(3)
                lbl = QLabel(name.split('_')[-1])
                lbl.setFont(QFont('Monospace', 8))
                lim_lbl = QLabel('')
                lim_lbl.setFont(QFont('Monospace', 7))
                lim_lbl.setFixedWidth(84)
                lim_lbl.setAlignment(Qt.AlignCenter)
                lim_lbl.setStyleSheet('color: #555555;')
                spin = QDoubleSpinBox()
                spin.setDecimals(2)
                spin.setRange(-6.28, 6.28)
                spin.setSingleStep(0.01)
                spin.setFixedWidth(80)
                spin.valueChanged.connect(
                    lambda val, n=name: self._on_joint_spin_changed(n, val))
                deg_lbl = QLabel('+  0.0°')
                deg_lbl.setFont(QFont('Monospace', 8))
                deg_lbl.setFixedWidth(58)
                deg_lbl.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
                deg_lbl.setStyleSheet('color: #555555;')
                _make_deg_updater(deg_lbl, spin)
                jrow.addWidget(lbl)
                jrow.addWidget(lim_lbl)
                jrow.addWidget(spin)
                jrow.addWidget(deg_lbl)
                col_vbox.addLayout(jrow)
                self._joint_spins[name] = spin
                self._joint_deg_labels[name] = deg_lbl
                self._joint_limit_labels[name] = lim_lbl
            col_vbox.addStretch()
            cols_layout.addLayout(col_vbox)
        left_vbox.addLayout(cols_layout)

        main_row.addLayout(left_vbox)

        if 'zero' in self._named_poses:
            self._cmb_preset.setCurrentText('zero')
            self._fill_joint_fields('zero')

        group.setLayout(main_row)
        return group

    def _fill_joint_fields(self, preset_name: str):
        if preset_name not in self._named_poses:
            return
        pose = self._named_poses[preset_name]
        names = pose.get('joint_names', [])
        positions = pose.get('positions', [])
        self._filling_preset = True
        try:
            for name, val in zip(names, positions):
                if name in self._joint_spins:
                    self._joint_spins[name].setValue(val)
        finally:
            self._filling_preset = False

    def _on_joint_spin_changed(self, name: str, val: float):
        if not self._filling_preset:
            self._cmb_preset.setCurrentIndex(-1)
        self._check_joint_limit(name, val)

    def _on_preset_selected(self, name: str):
        self._fill_joint_fields(name)

    def _on_execute_joint(self):
        positions = [self._joint_spins[n].value() for n in BODY_JOINT_NAMES]
        self._btn_execute.setEnabled(False)
        self._btn_execute.setText('Executing...')
        self._btn_execute.setStyleSheet(
            'background-color: #F57C00; color: white; font-weight: bold;')
        self._node.call_move_to_joint_position(
            positions, BODY_JOINT_NAMES, min_time=0.0,
            done_cb=lambda ok, msg: self._sig.execute_done.emit(ok, msg))

    def _on_execute_done(self, ok: bool, msg: str):
        self._btn_execute.setEnabled(True)
        self._btn_execute.setText('Execute')
        self._btn_execute.setStyleSheet(
            'background-color: #2E7D32; color: white; font-weight: bold;')
        self._sig.service_result.emit(ok, msg)

    def _on_save_preset(self):
        name = self._le_preset_name.text().strip()
        if not name:
            return
        positions = [self._joint_spins[n].value() for n in BODY_JOINT_NAMES]
        self._named_poses[name] = {
            'joint_names': list(BODY_JOINT_NAMES),
            'positions': positions,
        }
        _save_named_poses(self._named_poses, self._current_teleop_pose, self._current_robot_model)
        if self._cmb_preset.findText(name) < 0:
            self._cmb_preset.addItem(name)
        if self._cmb_ns_ref.findText(name) < 0:
            self._cmb_ns_ref.addItem(name)
        self._cmb_preset.setCurrentText(name)
        self._le_preset_name.clear()

    def _on_load_current_joints(self):
        self._btn_load_current.setEnabled(False)
        self._btn_load_current.setText('Waiting...')
        self._node.request_next_joint_state(
            lambda names, pos: self._sig.joint_state_received.emit(names, pos))

    def _on_joint_state_received(self, names: list, positions: list):
        self._filling_preset = True
        try:
            for name, val in zip(names, positions):
                if name in self._joint_spins:
                    self._joint_spins[name].setValue(round(val, 2))
        finally:
            self._filling_preset = False
        self._cmb_preset.setCurrentIndex(-1)
        self._btn_load_current.setEnabled(True)
        self._btn_load_current.setText('↓ Load Current Pos')

    # ── Cartesian Impedance Params group ──────────────────────────────────

    def _build_cartesian_impedance_group(self) -> QGroupBox:
        group = QGroupBox('Cartesian Impedance Params')
        top_layout = QVBoxLayout()
        top_layout.setSpacing(4)
        self._filling_imp_preset = False

        # ── Preset row ────────────────────────────────────────────────────
        preset_row = QHBoxLayout()
        self._cmb_imp_preset = QComboBox()
        self._cmb_imp_preset.setMinimumWidth(130)
        for name in self._imp_presets:
            self._cmb_imp_preset.addItem(name)
        btn_load_imp = _make_btn('Load', '#455A64', height=28)
        btn_load_imp.clicked.connect(
            lambda: self._fill_imp_preset(self._cmb_imp_preset.currentText()))
        self._le_imp_preset_name = QLineEdit()
        self._le_imp_preset_name.setPlaceholderText('preset name')
        self._le_imp_preset_name.setFixedWidth(110)
        btn_save_imp = _make_btn('Save', '#546E7A', height=28)
        btn_save_imp.clicked.connect(self._on_save_imp_preset)
        preset_row.addWidget(QLabel('Preset:'))
        preset_row.addWidget(self._cmb_imp_preset)
        preset_row.addWidget(btn_load_imp)
        preset_row.addSpacing(20)
        preset_row.addWidget(QLabel('Save as:'))
        preset_row.addWidget(self._le_imp_preset_name)
        preset_row.addWidget(btn_save_imp)
        preset_row.addStretch()
        top_layout.addLayout(preset_row)

        # ── Main two-column area ──────────────────────────────────────────
        cols = QHBoxLayout()
        cols.setSpacing(16)

        # Left: Joint Limits (dynamic rows)
        jl_vbox = QVBoxLayout()
        jl_vbox.setSpacing(3)
        jl_hdr_row = QHBoxLayout()
        jl_hdr_lbl = QLabel('Joint Limits')
        jl_hdr_lbl.setFont(QFont('Monospace', 9))
        jl_hdr_lbl.setStyleSheet(
            'background-color: #37474F; color: white; padding: 2px; border-radius: 3px;')
        jl_hdr_lbl.setAlignment(Qt.AlignCenter)
        btn_add_jl = _make_btn('+ Add Joint', '#1565C0', height=26)
        btn_add_jl.clicked.connect(lambda: self._on_add_jl_row())
        jl_hdr_row.addWidget(jl_hdr_lbl)
        jl_hdr_row.addStretch()
        jl_hdr_row.addWidget(btn_add_jl)
        jl_vbox.addLayout(jl_hdr_row)

        # Column headers — QWidget wrapper mirrors row_widget structure exactly
        col_hdr_widget = QWidget()
        col_hdr = QHBoxLayout(col_hdr_widget)
        col_hdr.setContentsMargins(0, 1, 0, 1)
        col_hdr.setSpacing(4)
        for txt, w in [('Joint', 120), ('Min (rad)', 80), ('Max (rad)', 80), ('', 30)]:
            lbl = QLabel(txt)
            lbl.setFont(QFont('Monospace', 8))
            lbl.setFixedWidth(w)
            col_hdr.addWidget(lbl)
        jl_vbox.addWidget(col_hdr_widget)

        # Scroll area for dynamic rows
        self._jl_container = QWidget()
        self._jl_container_layout = QVBoxLayout(self._jl_container)
        self._jl_container_layout.setSpacing(2)
        self._jl_container_layout.setContentsMargins(0, 0, 0, 0)
        self._jl_container_layout.addStretch()
        self._jl_rows = []
        scroll = QScrollArea()
        scroll.setWidget(self._jl_container)
        scroll.setWidgetResizable(True)
        scroll.setMinimumHeight(140)
        scroll.setMaximumHeight(200)
        scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        scroll.setStyleSheet('QScrollArea { border: none; }')
        jl_vbox.addWidget(scroll)

        cols.addLayout(jl_vbox, stretch=3)

        # Right: Nullspace Weights (fixed 14 rows)
        ns_vbox = QVBoxLayout()
        ns_vbox.setSpacing(3)
        ns_hdr = QLabel('Nullspace Weights')
        ns_hdr.setFont(QFont('Monospace', 9))
        ns_hdr.setStyleSheet(
            'background-color: #37474F; color: white; padding: 2px; border-radius: 3px;')
        ns_hdr.setAlignment(Qt.AlignCenter)
        ns_vbox.addWidget(ns_hdr)

        self._ns_weight_spins = {}
        ns_grid = QGridLayout()
        ns_grid.setSpacing(3)
        ns_grid.addWidget(QLabel('Right Arm'), 0, 0, 1, 2)
        ns_grid.addWidget(QLabel('Left Arm'),  0, 3, 1, 2)
        for i in range(7):
            r_name = f'right_arm_{i}'
            l_name = f'left_arm_{i}'
            ns_grid.addWidget(QLabel(str(i)), i + 1, 0)
            r_spin = QDoubleSpinBox()
            r_spin.setRange(0.0, 5.0)
            r_spin.setDecimals(3)
            r_spin.setSingleStep(0.01)
            r_spin.setFixedWidth(72)
            r_spin.setValue(0.05)
            r_spin.valueChanged.connect(self._on_imp_param_changed)
            ns_grid.addWidget(r_spin, i + 1, 1)
            ns_grid.addWidget(QLabel(str(i)), i + 1, 3)
            l_spin = QDoubleSpinBox()
            l_spin.setRange(0.0, 5.0)
            l_spin.setDecimals(3)
            l_spin.setSingleStep(0.01)
            l_spin.setFixedWidth(72)
            l_spin.setValue(0.05)
            l_spin.valueChanged.connect(self._on_imp_param_changed)
            ns_grid.addWidget(l_spin, i + 1, 4)
            self._ns_weight_spins[r_name] = r_spin
            self._ns_weight_spins[l_name] = l_spin
        ns_vbox.addLayout(ns_grid)
        btn_apply_ns = self._make_btn_with_fb(
            'Apply Weights', '#1565C0',
            lambda cb: self._on_apply_ns_weights(cb), height=28)
        ns_vbox.addStretch()
        cols.addLayout(ns_vbox, stretch=2)

        # ── 3rd column: Nullspace Ref Pose ───────────────────────────────────
        ns_ref_vbox = QVBoxLayout()
        ns_ref_vbox.setSpacing(3)
        ns_ref_hdr = QLabel('Nullspace Ref Pose')
        ns_ref_hdr.setFont(QFont('Monospace', 9))
        ns_ref_hdr.setStyleSheet(
            'background-color: #37474F; color: white; padding: 2px; border-radius: 3px;')
        ns_ref_hdr.setAlignment(Qt.AlignCenter)
        ns_ref_vbox.addWidget(ns_ref_hdr)
        self._cmb_ns_ref = QComboBox()
        self._cmb_ns_ref.setMinimumWidth(130)
        for np_name in self._named_poses:
            self._cmb_ns_ref.addItem(np_name)
        if self._current_teleop_pose and self._cmb_ns_ref.findText(self._current_teleop_pose) >= 0:
            self._cmb_ns_ref.setCurrentText(self._current_teleop_pose)
        self._cmb_ns_ref.currentTextChanged.connect(self._on_imp_param_changed)
        ns_ref_vbox.addWidget(self._cmb_ns_ref)
        btn_apply_ns_ref = self._make_btn_with_fb(
            'Apply Nullspace Ref', '#1565C0',
            lambda cb: self._on_apply_ns_ref(cb), height=28)
        ns_ref_vbox.addStretch()
        cols.addLayout(ns_ref_vbox, stretch=1)

        top_layout.addLayout(cols)

        # ── Apply buttons — 동일 수평선 ──────────────────────────────────────
        apply_row = QHBoxLayout()
        apply_row.setSpacing(16)
        apply_left = QHBoxLayout()
        apply_left.addStretch()
        btn_apply_jl = self._make_btn_with_fb(
            'Apply Joint Limits', '#1565C0',
            lambda cb: self._on_apply_jl(cb), height=28)
        apply_left.addWidget(btn_apply_jl)
        apply_mid = QHBoxLayout()
        apply_mid.addStretch()
        btn_apply_ns = self._make_btn_with_fb(
            'Apply Weights', '#1565C0',
            lambda cb: self._on_apply_ns_weights(cb), height=28)
        apply_mid.addWidget(btn_apply_ns)
        apply_right = QHBoxLayout()
        apply_right.addStretch()
        btn_apply_ns_ref = self._make_btn_with_fb(
            'Apply Nullspace Ref', '#1565C0',
            lambda cb: self._on_apply_ns_ref(cb), height=28)
        apply_right.addWidget(btn_apply_ns_ref)
        apply_row.addLayout(apply_left,  stretch=3)
        apply_row.addLayout(apply_mid,   stretch=2)
        apply_row.addLayout(apply_right, stretch=1)
        top_layout.addLayout(apply_row)

        group.setLayout(top_layout)

        # Load default preset if available
        if 'default' in self._imp_presets:
            self._fill_imp_preset('default')
            self._cmb_imp_preset.setCurrentText('default')

        return group

    def _on_add_jl_row(self, joint_name=None, min_val=0.0, max_val=1.0):
        row_widget = QWidget()
        row_layout = QHBoxLayout(row_widget)
        row_layout.setContentsMargins(0, 1, 0, 1)
        row_layout.setSpacing(4)

        combo = QComboBox()
        combo.addItems(BODY_JOINT_NAMES)
        combo.setFixedWidth(120)
        if joint_name and joint_name in BODY_JOINT_NAMES:
            combo.setCurrentText(joint_name)

        spin_min = QDoubleSpinBox()
        spin_min.setRange(-6.28, 6.28)
        spin_min.setDecimals(3)
        spin_min.setSingleStep(0.01)
        spin_min.setFixedWidth(80)
        spin_min.setValue(min_val)

        spin_max = QDoubleSpinBox()
        spin_max.setRange(-6.28, 6.28)
        spin_max.setDecimals(3)
        spin_max.setSingleStep(0.01)
        spin_max.setFixedWidth(80)
        spin_max.setValue(max_val)

        btn_rm = _make_btn('X', '#B71C1C', height=24)
        btn_rm.setFixedWidth(30)

        combo.currentTextChanged.connect(self._on_imp_param_changed)
        spin_min.valueChanged.connect(self._on_imp_param_changed)
        spin_max.valueChanged.connect(self._on_imp_param_changed)

        row_layout.addWidget(combo)
        row_layout.addWidget(spin_min)
        row_layout.addWidget(spin_max)
        row_layout.addWidget(btn_rm)

        row_data = {'combo': combo, 'min': spin_min, 'max': spin_max, 'widget': row_widget}
        self._jl_rows.append(row_data)
        btn_rm.clicked.connect(lambda: self._on_remove_jl_row(row_data))

        # Insert before the stretch item
        idx = self._jl_container_layout.count() - 1
        self._jl_container_layout.insertWidget(idx, row_widget)
        self._on_imp_param_changed()

    def _on_remove_jl_row(self, row_data: dict):
        row_data['widget'].setParent(None)
        row_data['widget'].deleteLater()
        if row_data in self._jl_rows:
            self._jl_rows.remove(row_data)
        self._on_imp_param_changed()

    def _on_apply_jl(self, done_cb=None):
        names = [r['combo'].currentText() for r in self._jl_rows]
        mins  = [r['min'].value()         for r in self._jl_rows]
        maxs  = [r['max'].value()         for r in self._jl_rows]
        self._node.call_set_ci_limits(names, mins, maxs, done_cb)

    def _on_apply_ns_weights(self, done_cb=None):
        names   = list(self._ns_weight_spins.keys())
        weights = [self._ns_weight_spins[n].value() for n in names]
        self._node.call_set_ns_weight(names, weights, done_cb)

    def _fill_imp_preset(self, name: str):
        preset = self._imp_presets.get(name)
        if not preset:
            return
        self._filling_imp_preset = True
        try:
            # Clear all existing joint limit rows
            for row in list(self._jl_rows):
                self._on_remove_jl_row(row)
            # Re-populate from preset
            for jl in preset.get('joint_limits', []):
                self._on_add_jl_row(jl.get('name'), jl.get('min', 0.0), jl.get('max', 1.0))
            # Fill nullspace weights
            weights = preset.get('nullspace_weights', {})
            r_weights = weights.get('right_arm', [0.05] * 7)
            l_weights = weights.get('left_arm',  [0.05] * 7)
            for i in range(7):
                r_val = r_weights[i] if i < len(r_weights) else 0.05
                l_val = l_weights[i] if i < len(l_weights) else 0.05
                self._ns_weight_spins[f'right_arm_{i}'].setValue(r_val)
                self._ns_weight_spins[f'left_arm_{i}'].setValue(l_val)
            # Set nullspace ref dropdown
            ns_ref = preset.get('nullspace_ref', '')
            if ns_ref and self._cmb_ns_ref.findText(ns_ref) >= 0:
                self._cmb_ns_ref.setCurrentText(ns_ref)
        finally:
            self._filling_imp_preset = False

    def _on_save_imp_preset(self):
        name = self._le_imp_preset_name.text().strip()
        if not name:
            return
        r_weights = [self._ns_weight_spins[f'right_arm_{i}'].value() for i in range(7)]
        l_weights = [self._ns_weight_spins[f'left_arm_{i}'].value()  for i in range(7)]
        self._imp_presets[name] = {
            'joint_limits': [
                {'name': r['combo'].currentText(),
                 'min':  round(r['min'].value(), 4),
                 'max':  round(r['max'].value(), 4)}
                for r in self._jl_rows
            ],
            'nullspace_weights': {
                'right_arm': [round(v, 4) for v in r_weights],
                'left_arm':  [round(v, 4) for v in l_weights],
            },
            'nullspace_ref': self._cmb_ns_ref.currentText(),
        }
        _save_impedance_presets(self._imp_presets)
        if self._cmb_imp_preset.findText(name) < 0:
            self._cmb_imp_preset.addItem(name)
        self._cmb_imp_preset.setCurrentText(name)
        self._le_imp_preset_name.clear()

    def _on_imp_param_changed(self, *_):
        if not self._filling_imp_preset:
            self._cmb_imp_preset.setCurrentIndex(-1)

    def _on_apply_ns_ref(self, done_cb=None):
        name = self._cmb_ns_ref.currentText()
        if name not in self._named_poses:
            if done_cb:
                done_cb(False, 'no pose selected')
            return
        pose = self._named_poses[name]
        positions = pose.get('positions', [])
        jnt_names = pose.get('joint_names', [])
        self._node.call_set_teleop_pose(positions, jnt_names)
        self._node.call_set_nullspace_joint_ref(positions, jnt_names, done_cb)

    # ── Node Indicators group ─────────────────────────────────────────────

    def _build_node_indicators_group(self) -> QGroupBox:
        group = QGroupBox('Node Indicators')
        hbox = QHBoxLayout()
        hbox.setSpacing(4)
        self._node_dots = {}

        sections = [
            ('core',           CORE_NODES,   1),
            ('vision',         [],           1),
            ('statemachine',   [],           1),
            ('motion planning', [],          1),
            ('driving',        [],           1),
            ('VLA',            [],           1),
            ('teleop',         TELEOP_NODES, 2),
            ('recording',      RECORDING_NODES, 1),
        ]

        for title, nodes, stretch in sections:
            sub = QGroupBox(title)
            sub_vbox = QVBoxLayout()
            sub_vbox.setSpacing(2)
            sub_vbox.setContentsMargins(4, 4, 4, 4)
            for node, pkg in nodes:
                row = QHBoxLayout()
                row.setSpacing(3)
                dot = QLabel('●')
                dot.setFont(QFont('Monospace', 11))
                dot.setStyleSheet('color: #888;')
                row.addWidget(dot)
                row.addWidget(QLabel(pkg))
                row.addStretch()
                sub_vbox.addLayout(row)
                self._node_dots[node] = dot
            sub_vbox.addStretch()
            sub.setLayout(sub_vbox)
            hbox.addWidget(sub, stretch)

        group.setLayout(hbox)
        return group

    # ── Teleop group ───────────────────────────────────────────────────────

    def _build_teleop_group(self) -> QGroupBox:
        group = QGroupBox('Teleop')
        hbox  = QHBoxLayout()
        hbox.setSpacing(8)

        left_col = QVBoxLayout()
        left_col.setSpacing(4)
        left_col.addWidget(self._build_nodes_panel())
        left_col.addWidget(self._build_calib_panel())
        left_col.addStretch()
        left_widget = QWidget()
        left_widget.setLayout(left_col)

        hbox.addWidget(left_widget, 2)
        hbox.addWidget(self._build_control_panel(), 2)
        hbox.addWidget(self._build_recording_panel(), 3)
        group.setLayout(hbox)
        return group

    def _build_nodes_panel(self) -> QGroupBox:
        group = QGroupBox('Tracker')
        vbox  = QVBoxLayout()
        vbox.setSpacing(3)

        self._lbl_tracker_l = QLabel('● L')
        self._lbl_tracker_r = QLabel('● R')
        self._lbl_tracker_b = QLabel('● B')
        for lbl in (self._lbl_tracker_l, self._lbl_tracker_r, self._lbl_tracker_b):
            lbl.setFont(QFont('Monospace', 10))
            lbl.setStyleSheet('color: #888;')
        tr_row = QHBoxLayout()
        tr_row.addStretch()
        tr_row.addWidget(self._lbl_tracker_l)
        tr_row.addStretch()
        tr_row.addWidget(self._lbl_tracker_b)
        tr_row.addStretch()
        tr_row.addWidget(self._lbl_tracker_r)
        tr_row.addStretch()
        vbox.addLayout(tr_row)
        self._chk_use_torso = QCheckBox('Use Torso')
        self._chk_use_torso.setChecked(False)
        self._chk_use_torso.stateChanged.connect(self._on_use_torso_changed)
        vbox.addWidget(self._chk_use_torso)
        vbox.addStretch()
        group.setLayout(vbox)
        return group

    def _build_pedal_panel(self) -> QGroupBox:
        group = QGroupBox('Pedal')
        hbox  = QHBoxLayout()
        self._btn_pedals = []
        for label in ['Resume/\nPause', 'Discard', '● Rec']:
            btn = QPushButton(label)
            btn.setEnabled(False)
            btn.setFixedHeight(44)
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
        btn_ts = self._make_btn_with_fb('▶  Teleop Start', '#4CAF50',
            lambda cb: self._node.call_trigger(self._node._cli_teleop_start, done_cb=cb),
            height=34)
        btn_tp = self._make_btn_with_fb('■  Teleop Stop',  '#E53935',
            lambda cb: self._node.call_trigger(self._node._cli_teleop_stop,  done_cb=cb),
            height=34)
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

        task_ep_row = QHBoxLayout()
        task_ep_row.addWidget(QLabel('task_id'))
        self._spin_task = QSpinBox()
        self._spin_task.setMinimum(0)
        self._spin_task.setMaximum(9999)
        self._spin_task.setFixedWidth(70)
        self._spin_task.valueChanged.connect(lambda v: self._node.pub_task_id(v))
        task_ep_row.addWidget(self._spin_task)
        task_ep_row.addSpacing(20)
        task_ep_row.addWidget(QLabel('episode'))
        self._lbl_episode = QLabel('—')
        self._lbl_episode.setFont(QFont('Monospace', 11))
        task_ep_row.addWidget(self._lbl_episode)
        task_ep_row.addStretch()

        self._btn_rec = _make_btn('▶  Start Episode', '#4CAF50', height=36)
        self._btn_rec.clicked.connect(self._on_rec_btn)

        vbox.addWidget(self._lbl_rec_state)
        vbox.addLayout(task_ep_row)
        vbox.addWidget(self._btn_rec)
        vbox.addSpacing(4)
        vbox.addWidget(self._build_pedal_panel())
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

    def _on_model_changed(self, btn_id: int):
        self._current_robot_model = 'a' if btn_id == 0 else 'm'
        _save_named_poses(self._named_poses, self._current_teleop_pose, self._current_robot_model)
        self._node.set_robot_model(self._current_robot_model)

    # ── helpers ────────────────────────────────────────────────────────────

    def _update_lbl(self, lbl, text: str, color: str):
        lbl.setText(f'  {text}  ')
        lbl.setStyleSheet(f'background-color: {color}; border-radius: 4px;')

    def _make_btn_with_fb(self, label, color, fn, on_done=None, height=30):
        """Button that disables itself while the service is in-flight and calls on_done(ok, msg)."""
        b = _make_btn(label, color, height=height)
        def _click():
            b.setEnabled(False)
            def _done(ok, msg):
                def _update():
                    if on_done:
                        on_done(ok, msg)
                    b.setEnabled(True)
                self._sig.dispatch.emit(_update)
            fn(_done)
        b.clicked.connect(_click)
        return b

    def _on_stop_move(self):
        self._btn_stop.setEnabled(False)
        def _done(ok, msg):
            self._sig.dispatch.emit(lambda: self._btn_stop.setEnabled(True))
        self._node.call_trigger(self._node._cli_stop_move, done_cb=_done)

    # ── service callbacks ──────────────────────────────────────────────────

    def _on_connect(self):
        host = self._le_ip.text().strip()
        no_gripper = self._chk_no_gripper.isChecked()
        model = 'a' if self._bg_model.checkedId() == 0 else 'm'
        self._btn_connect.setEnabled(False)
        self._btn_connect.setText('Connecting...')
        self._btn_connect.setStyleSheet(
            'background-color: #888; color: white; font-weight: bold; border-radius: 4px;')

        def _after_param(ok, param_msg):
            if not ok:
                self._node.get_logger().warning(f'[GUI] set robot_model failed: {param_msg}')

            def _done(ok, msg, names, lower, upper):
                self._sig.service_result.emit(ok, f'connect: {msg}')
                self._sig.connect_result.emit(ok, msg)
                if ok and names:
                    limits = {n: (lo, hi) for n, lo, hi in zip(names, lower, upper)}
                    self._sig.joint_limits_received.emit(limits)
                self._sig.dispatch.emit(lambda: self._btn_connect.setEnabled(True))

            self._node.call_connect(host, no_gripper, done_cb=_done)

        self._node.set_robot_model(model, done_cb=_after_param)

    def _on_service_result(self, ok: bool, msg: str):
        if not ok:
            self._node.get_logger().warn(f'[GUI] service result: {msg}')

    def _on_connect_result(self, ok: bool, msg: str):
        if ok:
            self._btn_connect.setText('Connected')
            self._btn_connect.setStyleSheet(
                'background-color: #2E7D32; color: white; font-weight: bold;')
            self._rb_model_a.setEnabled(False)
            self._rb_model_m.setEnabled(False)
        else:
            self._btn_connect.setText('Failed')
            self._btn_connect.setStyleSheet(
                'background-color: #C62828; color: white; font-weight: bold;')
            self._rb_model_a.setEnabled(True)
            self._rb_model_m.setEnabled(True)

    def _on_joint_limits_received(self, limits: dict):
        self._joint_limits = limits
        for name, (lo, hi) in limits.items():
            spin = self._joint_spins.get(name)
            if spin is None:
                continue
            lo_d, hi_d = math.degrees(lo), math.degrees(hi)
            spin.setToolTip(f'Limit: {lo_d:.1f}° ~ {hi_d:.1f}°')
            lim_lbl = self._joint_limit_labels.get(name)
            if lim_lbl is not None:
                lim_lbl.setText(f'({lo:.2f},  {hi:.2f})')
            self._check_joint_limit(name, spin.value())

    def _check_joint_limit(self, name: str, val: float):
        spin = self._joint_spins.get(name)
        if spin is None:
            return
        lo, hi = self._joint_limits.get(name, (None, None))
        if lo is not None and (val < lo or val > hi):
            spin.setStyleSheet('background-color: #C62828; color: white;')
        else:
            spin.setStyleSheet('')

    def _on_rby1_status(self, data: dict):
        power   = bool(data.get('power_state',  False))
        servo   = bool(data.get('servo_state',  False))
        stream  = bool(data.get('stream_state', False))
        gripper = bool(data.get('has_gripper',  False))
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

    def _on_tracker_status(self, sl: str, sr: str, sb: str):
        _colors = {'OK': '#4CAF50', 'JITTER': '#F0C040', 'LOST': '#E0302A'}
        self._lbl_tracker_l.setStyleSheet(f'color: {_colors.get(sl, "#888")};')
        self._lbl_tracker_r.setStyleSheet(f'color: {_colors.get(sr, "#888")};')
        self._lbl_tracker_b.setStyleSheet(f'color: {_colors.get(sb, "#888")};')

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

    def _on_use_torso_changed(self, state):
        self._node.call_set_use_torso(state == Qt.Checked.value)

    def _on_rec_btn(self):
        self._node.pub_task_id(self._spin_task.value())
        self._btn_rec.setEnabled(False)
        self._node.call_toggle_episode(
            done_cb=lambda ok, _: self._sig.dispatch.emit(
                lambda: self._btn_rec.setEnabled(True)))

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
    ros_node = ScmGuiNode()
    signals  = Signals()

    ros_node._pedal_cbs.append(          lambda s:    signals.pedal_updated.emit(s))
    ros_node._node_status_cbs.append(    lambda s:    signals.node_status_updated.emit(s))
    ros_node._rec_state_cbs.append(      lambda s:    signals.rec_state_changed.emit(s))
    ros_node._rec_episode_cbs.append(    lambda e:    signals.rec_episode_changed.emit(e))
    ros_node._tracker_status_cbs.append( lambda l, r, b: signals.tracker_status_changed.emit(l, r, b))
    ros_node._rby1_status_cbs.append(    lambda d:    signals.rby1_status_changed.emit(d))
    ros_node._clutch_state_cbs.append(   lambda s:    signals.clutch_state_changed.emit(s))

    signals.dispatch.connect(lambda fn: fn())

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
