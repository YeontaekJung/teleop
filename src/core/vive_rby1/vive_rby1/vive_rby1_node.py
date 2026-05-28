"""
vive_rby1_node.py
Vive Tracker teleoperation bridge for RB-Y1.

This is the DEBUG node (vive_rby1_debug_node), the Python twin of the C++
production node. It uses pink-based IK (rby1_ik) instead of the C++
DifferentialIkSolver. It talks to hw-core over the same /rby1/* interface.

Subscriptions:
  /teleop/tracker/left    geometry_msgs/PoseStamped  (from vive_ros2)
  /teleop/tracker/right   geometry_msgs/PoseStamped  (from vive_ros2)
  /rby1/state/joint       sensor_msgs/JointState     (from hw-core rby1_core_node)
  /teleop/task_id         std_msgs/Int32             (from scm_gui dropdown)

Publications (to hw-core):
  /rby1/cmd/joint         sensor_msgs/JointState     (pink_position / pink_impedance)
  /rby1/cmd/pose          tf2_msgs/TFMessage         (sdk_position / sdk_impedance;
                                                     used as data, NOT a TF broadcast.
                                                     child_frame_id: ee_right / ee_left /
                                                     link_torso_5)
  /teleop/rec_state       std_msgs/String            (IDLE / READY / RECORDING / PAUSED)
  /teleop/rec_episode     std_msgs/Int32             (current episode_id, -1 when IDLE)

hw-core lifecycle (client):
  /rby1/ctrl/mode             SetControlMode  (mode select)
  /rby1/stream                SetStream       (teleop start/stop)
  /rby1/move_to_joint_position MoveToJointPosition (move to ready pose)

Services (server):
  /vive_rby1/toggle_episode  std_srvs/Trigger        (GUI Start/End Episode button)

Recording services (client → scm_recording core):
  /scm_recording/start       StartRecording
  /scm_recording/end         EndRecording
  /scm_recording/toggle_pause  TogglePause
  # /scm_recording/status    GetStatus               (not yet used)

Pedal mapping (3-pedal USB, sensor_msgs/Joy):
  buttons[0] — TOGGLE arm engage/disengage
               + auto TogglePause when session active (READY/PAUSED→RECORDING, RECORDING→PAUSED)
  buttons[1] — (spare)
  buttons[2] — TOGGLE StartRecording / EndRecording

Recording state machine:
  IDLE      → pedal2 / GUI → StartRecording(task_id) → READY
  READY     → arm engage   → auto TogglePause        → RECORDING
  RECORDING → arm disengage→ auto TogglePause        → PAUSED
  PAUSED    → arm engage   → auto TogglePause        → RECORDING
  any       → pedal2 / GUI → EndRecording            → IDLE

Delta computation (robot frame):
  Δpos_robot  = v2r_R @ (tracker_now - tracker_ref)
  target_pos  = ee_pos_at_engage + pos_scale * Δpos_robot
  target_rot  = (v2r_R @ dR @ v2r_R.T) @ ee_rot_at_engage
"""

import time
from collections import deque
import numpy as np
import pinocchio as pin
from scipy.spatial.transform import Rotation as R, Slerp

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Pose, Transform, TransformStamped
from sensor_msgs.msg import Joy, JointState
from std_msgs.msg import Int32, String
from std_srvs.srv import Trigger
from tf2_msgs.msg import TFMessage

from rby1_core_msgs.srv import SetControlMode, SetStream, MoveToJointPosition
from scm_recording_msgs.srv import StartRecording, EndRecording, TogglePause
from rby1_ik.rby1_ik import Rby1Ik, get_rby1_body_joint_name_list


REC_IDLE      = 'IDLE'
REC_READY     = 'READY'
REC_RECORDING = 'RECORDING'
REC_PAUSED    = 'PAUSED'

# Ready pose (degrees), matches hw-core build_ready_q(). Sent via
# /rby1/move_to_joint_position because the old "ready_pose" verb was removed.
READY_Q_DEG = {
    'torso':     [0.0, 30.0, -60.0, 30.0, 0.0, 0.0],
    'right_arm': [-8.68, -9.86,  1.89, -103.95,  0.37, 22.07, -10.35],
    'left_arm':  [-8.68,  9.86, -1.89, -103.95, -0.37, 22.07,  10.35],
}

# Maps GUI ik_mode → hw-core SetControlMode (source, control).
_MODE_MAP = {
    'pink_position':  ('joint',     'position'),
    'pink_impedance': ('joint',     'impedance'),
    'sdk_position':   ('cartesian', 'position'),
    'sdk_impedance':  ('cartesian', 'impedance'),
}


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def pose_stamped_to_SE3(msg: PoseStamped) -> pin.SE3:
    p = msg.pose.position
    q = msg.pose.orientation
    rot = R.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
    return pin.SE3(rot, np.array([p.x, p.y, p.z]))


def se3_to_pose(se3: pin.SE3) -> Pose:
    pose = Pose()
    pose.position.x = float(se3.translation[0])
    pose.position.y = float(se3.translation[1])
    pose.position.z = float(se3.translation[2])
    q = R.from_matrix(se3.rotation).as_quat()
    pose.orientation.x = float(q[0])
    pose.orientation.y = float(q[1])
    pose.orientation.z = float(q[2])
    pose.orientation.w = float(q[3])
    return pose


def se3_to_transform(se3: pin.SE3) -> Transform:
    t = Transform()
    t.translation.x = float(se3.translation[0])
    t.translation.y = float(se3.translation[1])
    t.translation.z = float(se3.translation[2])
    q = R.from_matrix(se3.rotation).as_quat()
    t.rotation.x = float(q[0])
    t.rotation.y = float(q[1])
    t.rotation.z = float(q[2])
    t.rotation.w = float(q[3])
    return t


def make_transform_stamped(child_frame_id: str, se3: pin.SE3, stamp) -> TransformStamped:
    ts = TransformStamped()
    ts.header.stamp = stamp
    ts.header.frame_id = 'base'
    ts.child_frame_id = child_frame_id
    ts.transform = se3_to_transform(se3)
    return ts


def se3_to_pose_stamped(se3: pin.SE3, frame_id='world') -> PoseStamped:
    msg = PoseStamped()
    msg.header.frame_id = frame_id
    msg.pose.position.x = float(se3.translation[0])
    msg.pose.position.y = float(se3.translation[1])
    msg.pose.position.z = float(se3.translation[2])
    q = R.from_matrix(se3.rotation).as_quat()
    msg.pose.orientation.x = float(q[0])
    msg.pose.orientation.y = float(q[1])
    msg.pose.orientation.z = float(q[2])
    msg.pose.orientation.w = float(q[3])
    return msg


def tracker_target_to_ee_target(se3: pin.SE3) -> pin.SE3:
    """Convert a tracker frame target into the corresponding ee frame target.

    In the RB-Y1 URDF, `tracker_left/right` are fixed children of `ee_left/right`
    with xyz=(0.05, 0.0, -0.1) and no rotation. The SDK teleop path in hw-core
    commands `ee_left/right`, so SDK targets must be expressed in the ee frames.
    """
    T_ee_tracker = pin.SE3(np.eye(3), np.array([0.05, 0.0, -0.1]))
    return se3 * T_ee_tracker.inverse()


# ---------------------------------------------------------------------------
# Node
# ---------------------------------------------------------------------------

class ViveRby1Node(Node):

    def __init__(self):
        super().__init__('vive_rby1_node')

        # Parameters
        self.declare_parameter('urdf_path', '/home/hss/jyi/2026/robot_description/rby1/rby1.urdf')
        self.declare_parameter('srdf_path', '/home/hss/jyi/2026/robot_description/rby1/rby1.srdf')
        self.declare_parameter('topic_tracker_left',   '/teleop/tracker/left')
        self.declare_parameter('topic_tracker_right',  '/teleop/tracker/right')
        self.declare_parameter('topic_tracker_body',   '/teleop/tracker/body')
        self.declare_parameter('topic_pedal',          '/teleop/pedal')
        self.declare_parameter('topic_joint_state',    '/rby1/state/joint')
        self.declare_parameter('pos_scale',            2.0)
        self.declare_parameter('ik_dt',                0.05)
        self.declare_parameter('publish_rate',         20.0)
        self.declare_parameter('sdk_max_delta_pos',    0.03)
        self.declare_parameter('sdk_max_delta_rot_deg', 20.0)
        self.declare_parameter('pedal_engage_index',   0)
        self.declare_parameter('pedal_episode_index',  2)

        urdf_path = self.get_parameter('urdf_path').value
        srdf_path = self.get_parameter('srdf_path').value
        topic_l   = self.get_parameter('topic_tracker_left').value
        topic_r   = self.get_parameter('topic_tracker_right').value
        topic_b   = self.get_parameter('topic_tracker_body').value
        topic_p   = self.get_parameter('topic_pedal').value
        topic_js  = self.get_parameter('topic_joint_state').value

        self._pos_scale         = self.get_parameter('pos_scale').value
        self._ik_dt             = self.get_parameter('ik_dt').value
        self._sdk_max_delta_pos = float(self.get_parameter('sdk_max_delta_pos').value)
        self._sdk_max_delta_rot = np.deg2rad(
            float(self.get_parameter('sdk_max_delta_rot_deg').value))
        self._pedal_engage_idx  = self.get_parameter('pedal_engage_index').value
        self._pedal_episode_idx = self.get_parameter('pedal_episode_index').value
        rate_hz                 = self.get_parameter('publish_rate').value
        self._publish_rate      = rate_hz

        # Coordinate transform: tracker world frame → robot base frame
        # world +Y (forward) → robot +X,  world +X (right) → robot -Y,  world +Z (up) → robot +Z
        self._v2r_R = np.array([[0.,  1.,  0.],
                                [-1.,  0.,  0.],
                                [ 0.,  0.,  1.]])

        # IK solver
        if not urdf_path or not srdf_path:
            self.get_logger().error('urdf_path / srdf_path not set!')
            raise RuntimeError('Missing URDF/SRDF paths')
        self._ik = Rby1Ik(urdf_path, srdf_path)
        self.get_logger().info('[vive_rby1] IK solver ready')

        # Tracker / joint state
        self._tracker_l: PoseStamped | None = None
        self._tracker_r: PoseStamped | None = None
        self._tracker_l_se3: pin.SE3 | None = None  # smoothed SE3 for IK
        self._tracker_r_se3: pin.SE3 | None = None
        self._tracker_b_se3: pin.SE3 | None = None  # body tracker (optional)
        self._tracker_smooth_alpha = 0.9  # SLERP alpha for rotation (0=no update, 1=no smoothing)
        self._joint_state: JointState | None = None

        # Tracker status monitoring
        self._tracker_buf_l:   deque = deque(maxlen=20)
        self._tracker_buf_r:   deque = deque(maxlen=20)
        self._tracker_stamp_l: float = 0.0
        self._tracker_stamp_r: float = 0.0

        # Pedal edge-detect state
        self._pedal_engage_prev  = False
        self._pedal_episode_prev = False

        # Clutch state
        self._engaged  = False
        self._ref_l:   pin.SE3 | None = None
        self._ref_r:   pin.SE3 | None = None
        self._ee_l_0:  pin.SE3 | None = None
        self._ee_r_0:  pin.SE3 | None = None
        self._sdk_prev_l: pin.SE3 | None = None
        self._sdk_prev_r: pin.SE3 | None = None
        # Body tracker clutch state (sdk_impedance only)
        self._ref_b:        pin.SE3 | None = None
        self._torso_ref_se3: pin.SE3 | None = None
        self._sdk_prev_torso: pin.SE3 | None = None

        # Recording state
        self._rec_state   = REC_IDLE
        self._rec_episode = -1
        self._rec_task_id = 0

        # IK mode: 'pink_position' | 'pink_impedance' | 'sdk_position' | 'sdk_impedance'
        self._ik_mode     = 'pink_position'
        self._mirror_mode = False  # True = facing operator (L/R swap + Y flip)
        self._warmup_ticks  = 0   # countdown for pre-engage hold publish
        self._teleop_active = False  # True once teleop_start / impedance_teleop_start is sent

        # Subscribers
        self.create_subscription(PoseStamped, topic_l,          self._cb_tracker_l,   10)
        self.create_subscription(PoseStamped, topic_r,          self._cb_tracker_r,   10)
        self.create_subscription(PoseStamped, topic_b,          self._cb_tracker_b,   10)
        self.create_subscription(Joy,         topic_p,          self._cb_pedal,       10)
        self.create_subscription(JointState,  topic_js,         self._cb_joint_state, 10)
        self.create_subscription(Int32,  '/teleop/task_id',      self._cb_task_id,       10)
        self.create_subscription(String, '/teleop/control_mode',   self._cb_control_mode,   10)
        self.create_subscription(String, '/teleop/rby1_command',   self._cb_rby1_command,   10)
        self.create_subscription(String, '/teleop/mirror_mode',    self._cb_mirror_mode,    10)

        # Publishers (to hw-core)
        self._pub_joint_cmd     = self.create_publisher(JointState,      '/rby1/cmd/joint', 10)  # pink_position/impedance
        # /rby1/cmd/pose carries tf2_msgs/TFMessage as plain data (NOT a TF broadcast).
        # Each TransformStamped names a target link via child_frame_id:
        # 'ee_right', 'ee_left', and optionally 'link_torso_5'.
        self._pub_pose_cmd      = self.create_publisher(TFMessage,       '/rby1/cmd/pose',  10)  # sdk_position/impedance
        self._pub_rec_state     = self.create_publisher(String,            '/teleop/rec_state',                10)
        self._pub_rec_ep        = self.create_publisher(Int32,             '/teleop/rec_episode',              10)
        self._pub_tracker_status = self.create_publisher(String,           '/teleop/tracker_status',           10)

        # Recording service clients
        self._cli_start_rec    = self.create_client(StartRecording, '/scm_recording/start')
        self._cli_end_rec      = self.create_client(EndRecording,   '/scm_recording/end')
        self._cli_toggle_pause = self.create_client(TogglePause,    '/scm_recording/toggle_pause')

        # hw-core lifecycle service clients
        self._cli_set_mode   = self.create_client(SetControlMode,      '/rby1/ctrl/mode')
        self._cli_stream     = self.create_client(SetStream,           '/rby1/stream')
        self._cli_move_joint = self.create_client(MoveToJointPosition, '/rby1/move_to_joint_position')

        # Service server: GUI Start/End Episode button
        self.create_service(Trigger, '/vive_rby1/toggle_episode', self._srv_toggle_episode)

        # Timer
        self._timer = self.create_timer(1.0 / rate_hz, self._timer_cb)

        self.get_logger().info('[vive_rby1] Ready — press pedal 0 to engage')

    # ------------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------------

    def _smooth_tracker_se3(self, prev: pin.SE3 | None, msg: PoseStamped) -> pin.SE3:
        p = msg.pose.position
        q = msg.pose.orientation
        pos_new = np.array([p.x, p.y, p.z])
        rot_new = R.from_quat([q.x, q.y, q.z, q.w])
        if prev is None:
            return pin.SE3(rot_new.as_matrix(), pos_new)
        # Clamp large position jumps (tracker dropout / jitter)
        raw_delta = pos_new - prev.translation
        delta_norm = np.linalg.norm(raw_delta)
        MAX_DELTA = 0.05  # m/frame
        if delta_norm > MAX_DELTA:
            pos_new = prev.translation + raw_delta / delta_norm * MAX_DELTA
        # Light SLERP on rotation only — controller LPF handles the rest
        alpha = self._tracker_smooth_alpha
        rot_smooth = Slerp([0, 1], R.from_quat(
            [R.from_matrix(prev.rotation).as_quat(), rot_new.as_quat()]))(alpha)
        return pin.SE3(rot_smooth.as_matrix(), pos_new)

    def _cb_tracker_l(self, msg: PoseStamped):
        self._tracker_l = msg
        self._tracker_stamp_l = time.monotonic()
        p = msg.pose.position
        self._tracker_buf_l.append([p.x, p.y, p.z])
        self._tracker_l_se3 = self._smooth_tracker_se3(self._tracker_l_se3, msg)

    def _cb_tracker_r(self, msg: PoseStamped):
        self._tracker_r = msg
        self._tracker_stamp_r = time.monotonic()
        p = msg.pose.position
        self._tracker_buf_r.append([p.x, p.y, p.z])
        self._tracker_r_se3 = self._smooth_tracker_se3(self._tracker_r_se3, msg)

    def _cb_tracker_b(self, msg: PoseStamped):
        self._tracker_b_se3 = self._smooth_tracker_se3(self._tracker_b_se3, msg)

    def _cb_joint_state(self, msg: JointState):
        self._joint_state = msg
        self._ik.update_from_joint_state(
            list(msg.name), list(msg.position))

    def _cb_task_id(self, msg: Int32):
        self._rec_task_id = msg.data

    def _cb_control_mode(self, msg: String):
        self._ik_mode = msg.data
        self.get_logger().info(f'[vive_rby1] IK mode → {msg.data}')

    def _cb_mirror_mode(self, msg: String):
        self._mirror_mode = (msg.data == 'mirror')
        self.get_logger().info(f'[vive_rby1] mirror mode → {self._mirror_mode}')
        # 토글 시 reference 리셋 — 갑작스러운 target jump 방지
        if self._engaged:
            self._ref_l = self._tracker_l_se3
            self._ref_r = self._tracker_r_se3
            q_pin = self._ik.configuration.q
            fid_l = self._ik.robot.model.getFrameId('tracker_left')
            fid_r = self._ik.robot.model.getFrameId('tracker_right')
            self._ee_l_0 = self._ik.robot.framePlacement(q_pin, fid_l)
            self._ee_r_0 = self._ik.robot.framePlacement(q_pin, fid_r)

    def _cb_rby1_command(self, msg: String):
        cmd = msg.data
        if cmd == 'teleop_start':
            self._do_teleop_start()
        elif cmd == 'teleop_stop':
            self._do_teleop_stop()
        elif cmd == 'ready_pose':
            self._call_move_to_ready()
        else:
            self.get_logger().warn(f'[vive_rby1] unknown rby1_command: {cmd}')

    def _cb_pedal(self, _msg: Joy):
        # ---- Pedal 0: arm engage toggle (only when teleop is active) ----
        if self._pedal_engage_idx < len(_msg.buttons):
            pressed = bool(_msg.buttons[self._pedal_engage_idx])
            if pressed and not self._pedal_engage_prev:
                if not self._teleop_active:
                    self.get_logger().warn('Cannot engage — teleop not active')
                elif self._engaged:
                    self._on_disengage()
                elif self._tracker_l is not None and self._tracker_r is not None:
                    self._on_engage()
                else:
                    self.get_logger().warn('Cannot engage — Vive trackers not ready')
            self._pedal_engage_prev = pressed

        # ---- Pedal 1: spare ----

        # ---- Pedal 2: episode start/end toggle ----
        if self._pedal_episode_idx < len(_msg.buttons):
            pressed = bool(_msg.buttons[self._pedal_episode_idx])
            if pressed and not self._pedal_episode_prev:
                self._toggle_episode()
            self._pedal_episode_prev = pressed

    # ------------------------------------------------------------------
    # Engage / disengage
    # ------------------------------------------------------------------

    def _on_engage(self):
        if self._tracker_l is None or self._tracker_r is None:
            self.get_logger().warn('Trackers not ready — ignoring engage')
            return

        self._ref_l = self._tracker_l_se3
        self._ref_r = self._tracker_r_se3

        q_pin = self._ik.configuration.q
        fid_l = self._ik.robot.model.getFrameId('tracker_left')
        fid_r = self._ik.robot.model.getFrameId('tracker_right')
        self._ee_l_0 = self._ik.robot.framePlacement(q_pin, fid_l)
        self._ee_r_0 = self._ik.robot.framePlacement(q_pin, fid_r)
        self._sdk_prev_l = None
        self._sdk_prev_r = None

        # Body tracker reference for torso (sdk_impedance only, optional)
        if self._ik_mode == 'sdk_impedance' and self._tracker_b_se3 is not None:
            self._ref_b = self._tracker_b_se3
            fid_torso = self._ik.robot.model.getFrameId('link_torso_5')
            self._torso_ref_se3 = self._ik.robot.framePlacement(q_pin, fid_torso)
            self._sdk_prev_torso = None
            self.get_logger().info('[vive_rby1] torso ref captured at engage')
        else:
            self.get_logger().warn(
                f'[vive_rby1] torso ref NOT captured: mode={self._ik_mode} tracker_b={self._tracker_b_se3 is not None}')

        self._engaged = True
        self.get_logger().info('Clutch ENGAGED')

        # Auto-resume recording if session is active
        if self._rec_state in (REC_READY, REC_PAUSED):
            self._call_toggle_pause()

    def _on_disengage(self):
        self._engaged = False
        self._sdk_prev_l = None
        self._sdk_prev_r = None
        self._ref_b = None
        self._torso_ref_se3 = None
        self._sdk_prev_torso = None
        self.get_logger().info('Clutch DISENGAGED')

        # Auto-pause recording if currently recording
        if self._rec_state == REC_RECORDING:
            self._call_toggle_pause()

    # ------------------------------------------------------------------
    # Episode start / end
    # ------------------------------------------------------------------

    def _toggle_episode(self):
        if self._rec_state == REC_IDLE:
            if not self._cli_start_rec.service_is_ready():
                self.get_logger().warn('StartRecording service not available')
                return
            req = StartRecording.Request()
            req.task_id = self._rec_task_id
            self._cli_start_rec.call_async(req).add_done_callback(self._on_start_done)
        elif self._rec_state == REC_RECORDING:
            self.get_logger().warn('EndRecording blocked — disengage arm first (must be PAUSED)')
        else:  # READY or PAUSED
            if not self._cli_end_rec.service_is_ready():
                self.get_logger().warn('EndRecording service not available')
                return
            self._cli_end_rec.call_async(EndRecording.Request()).add_done_callback(self._on_end_done)

    def _srv_toggle_episode(self, _req, resp):
        """Service handler — GUI Start/End Episode button."""
        self._toggle_episode()
        resp.success = True
        resp.message = 'OK'
        return resp

    def _call_toggle_pause(self):
        # Don't check service_is_ready() — server may only advertise after session starts
        self._cli_toggle_pause.call_async(TogglePause.Request()).add_done_callback(
            self._on_toggle_pause_done)

    # ------------------------------------------------------------------
    # Service response callbacks
    # ------------------------------------------------------------------

    def _on_start_done(self, future):
        result = future.result()
        if result.result:
            self._rec_state   = REC_READY
            self._rec_episode = result.episode_id
            self.get_logger().info(
                f'[vive_rby1] READY — task {result.task_id} ep {result.episode_id}')
            self._warmup_ticks = int(self._publish_rate)
            self._do_teleop_start()
        else:
            self.get_logger().error(f'StartRecording failed: {result.message}')
        self._publish_rec_state()

    def _on_end_done(self, future):
        result = future.result()
        if result.result:
            self._rec_state   = REC_IDLE
            self._rec_episode = -1
            self._engaged     = False
            self.get_logger().info('[vive_rby1] Recording ENDED — stream off → ready_pose')
            self._do_teleop_stop()
            self._call_move_to_ready()
        else:
            self.get_logger().error(f'EndRecording failed: {result.message}')
        self._publish_rec_state()

    # ------------------------------------------------------------------
    # hw-core lifecycle (service-based, async + done-callback chaining)
    # ------------------------------------------------------------------

    def _make_ready_target(self) -> JointState:
        js = JointState()
        js.name = get_rby1_body_joint_name_list()
        deg = (READY_Q_DEG['torso'] + READY_Q_DEG['right_arm'] + READY_Q_DEG['left_arm'])
        js.position = [float(np.deg2rad(v)) for v in deg]
        return js

    def _do_teleop_start(self):
        """SetControlMode → MoveToJointPosition(ready) → SetStream(true), chained."""
        if not self._cli_set_mode.service_is_ready():
            self.get_logger().warn('[vive_rby1] /rby1/ctrl/mode not ready — skip teleop_start')
            return
        source, control = _MODE_MAP.get(self._ik_mode, ('cartesian', 'impedance'))
        req = SetControlMode.Request()
        req.source = source
        req.control = control
        self.get_logger().info(f'[vive_rby1] teleop_start: set mode {source}/{control}')
        self._cli_set_mode.call_async(req).add_done_callback(self._after_set_mode)

    def _after_set_mode(self, future):
        try:
            if not future.result().success:
                self.get_logger().error('[vive_rby1] SetControlMode failed — abort teleop_start')
                self._abort_arming()
                return
        except Exception as e:
            self.get_logger().error(f'[vive_rby1] SetControlMode exception: {e}')
            self._abort_arming()
            return
        req = MoveToJointPosition.Request()
        req.target = self._make_ready_target()
        req.min_time = 5.0
        self.get_logger().info('[vive_rby1] teleop_start: moving to ready pose')
        self._cli_move_joint.call_async(req).add_done_callback(self._after_move_ready_start)

    def _after_move_ready_start(self, future):
        try:
            if not future.result().success:
                self.get_logger().error('[vive_rby1] MoveToJointPosition failed — abort teleop_start')
                self._abort_arming()
                return
        except Exception as e:
            self.get_logger().error(f'[vive_rby1] MoveToJointPosition exception: {e}')
            self._abort_arming()
            return
        req = SetStream.Request()
        req.enable = True
        self.get_logger().info('[vive_rby1] teleop_start: starting stream')
        self._cli_stream.call_async(req).add_done_callback(self._after_stream_on)

    def _after_stream_on(self, future):
        try:
            ok = future.result().success
        except Exception as e:
            self.get_logger().error(f'[vive_rby1] SetStream exception: {e}')
            ok = False
        if ok:
            self._teleop_active = True
            self._warmup_ticks = int(self._publish_rate)
            self.get_logger().info('[vive_rby1] teleop_start: complete')
        else:
            self.get_logger().error('[vive_rby1] SetStream failed — teleop not active')
            self._abort_arming()

    def _abort_arming(self):
        # teleop_start failed mid-sequence; if we had just armed a recording
        # session (READY, not yet engaged), revert it to IDLE.
        self._teleop_active = False
        self._warmup_ticks = 0
        if self._rec_state == REC_READY:
            self._rec_state = REC_IDLE
            self._rec_episode = -1
            self._publish_rec_state()

    def _do_teleop_stop(self):
        self._teleop_active = False
        self._engaged = False
        if not self._cli_stream.service_is_ready():
            self.get_logger().warn('[vive_rby1] /rby1/stream not ready — skip teleop_stop')
            return
        req = SetStream.Request()
        req.enable = False
        self._cli_stream.call_async(req)

    def _call_move_to_ready(self):
        if not self._cli_move_joint.service_is_ready():
            self.get_logger().warn('[vive_rby1] /rby1/move_to_joint_position not ready — skip ready_pose')
            return
        req = MoveToJointPosition.Request()
        req.target = self._make_ready_target()
        req.min_time = 5.0
        self._cli_move_joint.call_async(req)

    def _on_toggle_pause_done(self, future):
        try:
            result = future.result()
        except Exception as e:
            self.get_logger().error(f'TogglePause exception: {e}')
            return
        if result.result:
            self._rec_state = REC_PAUSED if result.paused else REC_RECORDING
            self.get_logger().info(f'[vive_rby1] {self._rec_state}')
        else:
            self.get_logger().error(
                f'TogglePause failed — result={result.result} paused={result.paused} msg={result.message}')
        self._publish_rec_state()

    def _publish_rec_state(self):
        self._pub_rec_state.publish(String(data=self._rec_state))
        self._pub_rec_ep.publish(Int32(data=self._rec_episode))

    def _limit_sdk_target(self, prev: pin.SE3 | None, target: pin.SE3,
                          arm_name: str) -> pin.SE3 | None:
        if (not np.isfinite(target.translation).all() or
                not np.isfinite(target.rotation).all()):
            self.get_logger().warn(
                f'[vive_rby1] dropping non-finite SDK target for {arm_name}')
            return prev

        if prev is None:
            return target

        pos = target.translation.copy()
        delta = pos - prev.translation
        delta_norm = np.linalg.norm(delta)
        if delta_norm > self._sdk_max_delta_pos > 0.0:
            pos = prev.translation + delta / delta_norm * self._sdk_max_delta_pos

        rot_prev = R.from_matrix(prev.rotation)
        rot_target = R.from_matrix(target.rotation)
        angle = (rot_prev.inv() * rot_target).magnitude()
        if not np.isfinite(angle):
            self.get_logger().warn(
                f'[vive_rby1] dropping invalid SDK rotation for {arm_name}')
            return prev

        if angle > self._sdk_max_delta_rot > 0.0:
            ratio = self._sdk_max_delta_rot / angle
            slerp = Slerp(
                [0.0, 1.0],
                R.from_quat(np.vstack([rot_prev.as_quat(), rot_target.as_quat()])),
            )
            rot = slerp(ratio).as_matrix()
        else:
            rot = target.rotation

        return pin.SE3(rot, pos)

    def _tracker_status(self, buf: deque, stamp: float) -> str:
        if time.monotonic() - stamp > 0.5:
            return 'LOST'
        if len(buf) >= 10:
            arr = np.array(buf)
            vel = np.diff(arr, axis=0)
            if np.std(vel, axis=0).max() > 0.003:
                return 'JITTER'
        return 'OK'

    # ------------------------------------------------------------------
    # Main timer: IK and publish
    # ------------------------------------------------------------------

    def _timer_cb(self):
        sl = self._tracker_status(self._tracker_buf_l, self._tracker_stamp_l)
        sr = self._tracker_status(self._tracker_buf_r, self._tracker_stamp_r)
        self._pub_tracker_status.publish(String(data=f'L:{sl} R:{sr}'))

        # Warm-up: pink 모드만 joint 명령 pre-publish (SDK 모드는 /rby1/cmd/pose 사용)
        if self._warmup_ticks > 0:
            self._warmup_ticks -= 1
            if not self._ik_mode.startswith('sdk_'):
                self._publish_q20(self._ik.current_q20)
            return

        if self._tracker_l is None or self._tracker_r is None:
            return
        if not self._engaged or self._ref_l is None or self._ee_l_0 is None:
            return

        tracker_l_now = self._tracker_l_se3
        tracker_r_now = self._tracker_r_se3
        if tracker_l_now is None or tracker_r_now is None:
            return

        delta_l = tracker_l_now.translation - self._ref_l.translation
        delta_r = tracker_r_now.translation - self._ref_r.translation

        v2r = self._v2r_R
        if self._mirror_mode:
            # 마주보기: tracker 교차 (오른손→왼팔, 왼손→오른팔) + 좌우축 반전
            # mirror_flip: robot Y축(좌우)만 반전 — 마주보면 좌우가 대칭
            mirror_flip = np.diag([1., -1., 1.])
            #
            # --- 옵션 A (기본): 마주보고 뻗으면 다가옴 ---
            target_pos_l = self._ee_l_0.translation + self._pos_scale * (mirror_flip @ v2r @ delta_r)
            target_pos_r = self._ee_r_0.translation + self._pos_scale * (mirror_flip @ v2r @ delta_l)
            #
            # --- 옵션 B: forward도 반전 (위 두 줄 주석 후 아래 해제) ---
            # flip_all = np.diag([-1., -1., 1.])
            # target_pos_l = self._ee_l_0.translation + self._pos_scale * (flip_all @ v2r @ delta_r)
            # target_pos_r = self._ee_r_0.translation + self._pos_scale * (flip_all @ v2r @ delta_l)

            dR_l = tracker_r_now.rotation @ self._ref_r.rotation.T
            dR_r = tracker_l_now.rotation @ self._ref_l.rotation.T
        else:
            target_pos_l = self._ee_l_0.translation + self._pos_scale * (v2r @ delta_l)
            target_pos_r = self._ee_r_0.translation + self._pos_scale * (v2r @ delta_r)
            dR_l = tracker_l_now.rotation @ self._ref_l.rotation.T
            dR_r = tracker_r_now.rotation @ self._ref_r.rotation.T

        dR_l_robot = v2r @ dR_l @ v2r.T
        dR_r_robot = v2r @ dR_r @ v2r.T

        if self._mirror_mode:
            mirror_flip_rot = np.diag([1., -1., 1.])
            dR_l_robot = mirror_flip_rot @ dR_l_robot @ mirror_flip_rot
            dR_r_robot = mirror_flip_rot @ dR_r_robot @ mirror_flip_rot

        target_rot_l = dR_l_robot @ self._ee_l_0.rotation
        target_rot_r = dR_r_robot @ self._ee_r_0.rotation

        l_SE3 = pin.SE3(target_rot_l, target_pos_l)
        r_SE3 = pin.SE3(target_rot_r, target_pos_r)

        if self._ik_mode.startswith('sdk_'):
            ee_l = tracker_target_to_ee_target(l_SE3)
            ee_r = tracker_target_to_ee_target(r_SE3)
            sdk_l = self._limit_sdk_target(self._sdk_prev_l, ee_l, 'left')
            sdk_r = self._limit_sdk_target(self._sdk_prev_r, ee_r, 'right')
            if sdk_l is None or sdk_r is None:
                return
            self._sdk_prev_l = sdk_l
            self._sdk_prev_r = sdk_r
            msg = TFMessage()
            stamp = self.get_clock().now().to_msg()
            msg.transforms = [
                make_transform_stamped('ee_right', sdk_r, stamp),
                make_transform_stamped('ee_left',  sdk_l, stamp),
            ]
            # Torso target from body tracker (sdk_impedance only)
            if (self._ik_mode == 'sdk_impedance'
                    and self._ref_b is not None
                    and self._torso_ref_se3 is not None
                    and self._tracker_b_se3 is not None):
                v2r = self._v2r_R
                b_now = self._tracker_b_se3
                delta_pos = v2r @ (b_now.translation - self._ref_b.translation)
                delta_rot = v2r @ (b_now.rotation @ self._ref_b.rotation.T) @ v2r.T
                torso_tgt = pin.SE3(
                    delta_rot @ self._torso_ref_se3.rotation,
                    self._torso_ref_se3.translation + delta_pos,
                )
                sdk_torso = self._limit_sdk_target(self._sdk_prev_torso, torso_tgt, 'torso')
                if sdk_torso is not None:
                    self._sdk_prev_torso = sdk_torso
                    msg.transforms.append(
                        make_transform_stamped('link_torso_5', sdk_torso, stamp))
            elif self._ik_mode == 'sdk_impedance':
                self.get_logger().info(
                    f'[torso] skip: ref_b={self._ref_b is not None} '
                    f'torso_ref={self._torso_ref_se3 is not None} '
                    f'tracker_b={self._tracker_b_se3 is not None}',
                    throttle_duration_sec=2.0)
            self._pub_pose_cmd.publish(msg)
        else:
            q20 = self._ik.solve_ik_to_q20(l_SE3, r_SE3, self._ik_dt)
            self._publish_q20(q20)

    def _publish_q20(self, q20: np.ndarray):
        # /rby1/cmd/joint is name-keyed (hw-core matches by joint name); the mode
        # (JointPosition vs JointImpedance) is selected via SetControlMode, so
        # both pink modes publish to the same topic. No gripper slots.
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = get_rby1_body_joint_name_list()
        msg.position = [float(v) for v in q20]
        self._pub_joint_cmd.publish(msg)


# ---------------------------------------------------------------------------

def main(args=None):
    rclpy.init(args=args)
    node = ViveRby1Node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
