"""
vive_rby1_node.py
Vive Tracker teleoperation bridge for RB-Y1.

Subscriptions:
  /teleop/tracker/left    geometry_msgs/PoseStamped  (from vive_ros2)
  /teleop/tracker/right   geometry_msgs/PoseStamped  (from vive_ros2)
  /teleop/pedal           sensor_msgs/Joy            (from pedal driver)
  /rby1_status_joint      sensor_msgs/JointState     (from rby1 SDK)
  /teleop/task_id         std_msgs/Int32             (from teleop_gui dropdown)

Publications:
  /rby1_teleop_command    interbotix_xs_msgs/JointGroupCommand
  /teleop/rec_state       std_msgs/String            (IDLE / READY / RECORDING / PAUSED)
  /teleop/rec_episode     std_msgs/Int32             (current episode_id, -1 when IDLE)

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

import numpy as np
import pinocchio as pin
from scipy.spatial.transform import Rotation as R

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Joy, JointState
from std_msgs.msg import Int32, String
from std_srvs.srv import Trigger
from interbotix_xs_msgs.msg import JointGroupCommand
from rby1_core_msgs.action import Rby1Command

from scm_recording_msgs.srv import StartRecording, EndRecording, TogglePause
from rby1_ik.rby1_ik import Rby1Ik


REC_IDLE      = 'IDLE'
REC_READY     = 'READY'
REC_RECORDING = 'RECORDING'
REC_PAUSED    = 'PAUSED'


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def pose_stamped_to_SE3(msg: PoseStamped) -> pin.SE3:
    p = msg.pose.position
    q = msg.pose.orientation
    rot = R.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
    return pin.SE3(rot, np.array([p.x, p.y, p.z]))


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
        self.declare_parameter('topic_pedal',          '/teleop/pedal')
        self.declare_parameter('topic_joint_state',    '/rby1_status_joint')
        self.declare_parameter('topic_teleop_command', '/rby1_teleop_command')
        self.declare_parameter('pos_scale',            2.0)
        self.declare_parameter('ik_dt',                0.05)
        self.declare_parameter('publish_rate',         20.0)
        self.declare_parameter('pedal_engage_index',   0)
        self.declare_parameter('pedal_episode_index',  2)

        urdf_path = self.get_parameter('urdf_path').value
        srdf_path = self.get_parameter('srdf_path').value
        topic_l   = self.get_parameter('topic_tracker_left').value
        topic_r   = self.get_parameter('topic_tracker_right').value
        topic_p   = self.get_parameter('topic_pedal').value
        topic_js  = self.get_parameter('topic_joint_state').value
        topic_cmd = self.get_parameter('topic_teleop_command').value

        self._pos_scale         = self.get_parameter('pos_scale').value
        self._ik_dt             = self.get_parameter('ik_dt').value
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
        self._joint_state: JointState | None = None

        # Pedal edge-detect state
        self._pedal_engage_prev  = False
        self._pedal_episode_prev = False

        # Clutch state
        self._engaged  = False
        self._ref_l:   pin.SE3 | None = None
        self._ref_r:   pin.SE3 | None = None
        self._ee_l_0:  pin.SE3 | None = None
        self._ee_r_0:  pin.SE3 | None = None

        # Recording state
        self._rec_state   = REC_IDLE
        self._rec_episode = -1
        self._rec_task_id = 0

        # Control mode: False = position, True = impedance
        self._use_impedance = False
        self._warmup_ticks  = 0   # countdown for pre-engage hold publish

        # Subscribers
        self.create_subscription(PoseStamped, topic_l,          self._cb_tracker_l,   10)
        self.create_subscription(PoseStamped, topic_r,          self._cb_tracker_r,   10)
        self.create_subscription(Joy,         topic_p,          self._cb_pedal,       10)
        self.create_subscription(JointState,  topic_js,         self._cb_joint_state, 10)
        self.create_subscription(Int32,  '/teleop/task_id',      self._cb_task_id,       10)
        self.create_subscription(String, '/teleop/control_mode', self._cb_control_mode,  10)

        # Publishers
        self._pub_cmd           = self.create_publisher(JointGroupCommand, topic_cmd,                          10)
        self._pub_impedance_cmd = self.create_publisher(JointGroupCommand, '/rby1_impedance_teleop_command',   10)
        self._pub_rec_state     = self.create_publisher(String,            '/teleop/rec_state',                10)
        self._pub_rec_ep        = self.create_publisher(Int32,             '/teleop/rec_episode',              10)

        # Recording service clients
        self._cli_start_rec    = self.create_client(StartRecording, '/scm_recording/start')
        self._cli_end_rec      = self.create_client(EndRecording,   '/scm_recording/end')
        self._cli_toggle_pause = self.create_client(TogglePause,    '/scm_recording/toggle_pause')

        # RB-Y1 command action client
        self._rby1_client = ActionClient(self, Rby1Command, '/rby1_command')

        # Service server: GUI Start/End Episode button
        self.create_service(Trigger, '/vive_rby1/toggle_episode', self._srv_toggle_episode)

        # Timer
        self._timer = self.create_timer(1.0 / rate_hz, self._timer_cb)

        self.get_logger().info('[vive_rby1] Ready — press pedal 0 to engage')

    # ------------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------------

    def _cb_tracker_l(self, msg: PoseStamped):
        self._tracker_l = msg

    def _cb_tracker_r(self, msg: PoseStamped):
        self._tracker_r = msg

    def _cb_joint_state(self, msg: JointState):
        self._joint_state = msg
        self._ik.update_from_joint_state(
            list(msg.name), list(msg.position))

    def _cb_task_id(self, msg: Int32):
        self._rec_task_id = msg.data

    def _cb_control_mode(self, msg: String):
        self._use_impedance = (msg.data == 'impedance')
        self.get_logger().info(f'[vive_rby1] control mode → {msg.data}')

    def _cb_pedal(self, _msg: Joy):
        # ---- Pedal 0: arm engage toggle ----
        if self._pedal_engage_idx < len(_msg.buttons):
            pressed = bool(_msg.buttons[self._pedal_engage_idx])
            if pressed and not self._pedal_engage_prev:
                if self._engaged:
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

        self._ref_l = pose_stamped_to_SE3(self._tracker_l)
        self._ref_r = pose_stamped_to_SE3(self._tracker_r)

        q_pin = self._ik.configuration.q
        fid_l = self._ik.robot.model.getFrameId('tracker_left')
        fid_r = self._ik.robot.model.getFrameId('tracker_right')
        self._ee_l_0 = self._ik.robot.framePlacement(q_pin, fid_l)
        self._ee_r_0 = self._ik.robot.framePlacement(q_pin, fid_r)

        self._engaged = True
        self.get_logger().info('Clutch ENGAGED')

        # Auto-resume recording if session is active
        if self._rec_state in (REC_READY, REC_PAUSED):
            self._call_toggle_pause()

    def _on_disengage(self):
        self._engaged = False
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

    def _srv_toggle_episode(self, req, resp):
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
            # Publish current joint state for 1 second so SDK auto-detects the topic
            self._warmup_ticks = int(self._publish_rate)
            # Already engaged while waiting for response → auto-resume immediately
            if self._engaged:
                self._call_toggle_pause()
        else:
            self.get_logger().error(f'StartRecording failed: {result.message}')
        self._publish_rec_state()

    def _on_end_done(self, future):
        result = future.result()
        if result.result:
            self._rec_state   = REC_IDLE
            self._rec_episode = -1
            self._engaged     = False
            self.get_logger().info('[vive_rby1] Recording ENDED — moving to vla_pose2')
            self._send_rby1_command('vla_pose2')
        else:
            self.get_logger().error(f'EndRecording failed: {result.message}')
        self._publish_rec_state()

    def _send_rby1_command(self, command: str):
        if not self._rby1_client.server_is_ready():
            self.get_logger().warn(f'rby1_command server not ready — skipping "{command}"')
            return
        goal_msg = Rby1Command.Goal()
        goal_msg.command = command
        self._rby1_client.send_goal_async(goal_msg)

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

    # ------------------------------------------------------------------
    # Main timer: IK and publish
    # ------------------------------------------------------------------

    def _timer_cb(self):
        # Warm-up: publish current joint state so SDK auto-detects the topic
        if self._warmup_ticks > 0:
            self._warmup_ticks -= 1
            self._publish_q20(self._ik.current_q20)
            return

        if self._tracker_l is None or self._tracker_r is None:
            return
        if not self._engaged or self._ref_l is None or self._ee_l_0 is None:
            return

        tracker_l_now = pose_stamped_to_SE3(self._tracker_l)
        tracker_r_now = pose_stamped_to_SE3(self._tracker_r)

        delta_l = tracker_l_now.translation - self._ref_l.translation
        delta_r = tracker_r_now.translation - self._ref_r.translation

        target_pos_l = self._ee_l_0.translation + self._pos_scale * (self._v2r_R @ delta_l)
        target_pos_r = self._ee_r_0.translation + self._pos_scale * (self._v2r_R @ delta_r)

        dR_l = tracker_l_now.rotation @ self._ref_l.rotation.T
        dR_r = tracker_r_now.rotation @ self._ref_r.rotation.T

        dR_l_robot = self._v2r_R @ dR_l @ self._v2r_R.T
        dR_r_robot = self._v2r_R @ dR_r @ self._v2r_R.T

        target_rot_l = dR_l_robot @ self._ee_l_0.rotation
        target_rot_r = dR_r_robot @ self._ee_r_0.rotation

        q20 = self._ik.solve_ik_to_q20(
            pin.SE3(target_rot_l, target_pos_l),
            pin.SE3(target_rot_r, target_pos_r),
            self._ik_dt)

        self._publish_q20(q20)

    def _publish_q20(self, q20: np.ndarray):
        cmd = JointGroupCommand()
        cmd.name = 'All'
        cmd.cmd  = np.concatenate((q20, np.array([0., 0.]))).tolist()
        if self._use_impedance:
            self._pub_impedance_cmd.publish(cmd)
        else:
            self._pub_cmd.publish(cmd)


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
