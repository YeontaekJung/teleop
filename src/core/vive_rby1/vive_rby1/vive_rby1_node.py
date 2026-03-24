"""
vive_rby1_node.py
Vive Tracker teleoperation bridge for RB-Y1.

Subscriptions:
  /teleop/tracker/left    geometry_msgs/PoseStamped  (from vive_ros2)
  /teleop/tracker/right   geometry_msgs/PoseStamped  (from vive_ros2)
  /teleop/pedal           sensor_msgs/Joy            (from pedal driver)
  /rby1_status_joint      sensor_msgs/JointState     (from rby1 SDK)

Publications:
  /rby1_teleop_command    interbotix_xs_msgs/JointGroupCommand
  /teleop/recording       std_msgs/Bool              (recording active state)

Pedal mapping (3-pedal USB, sensor_msgs/Joy):
  buttons[0] — HOLD to engage teleoperation (dead-man switch)
  buttons[1] — (spare)
  buttons[2] — TOGGLE recording start/stop

Clutch logic (pedal 0):
  Pedal HELD    → tracking active, /rby1_teleop_command published
  Pedal RELEASE → publishing stops (robot holds last position)
  On re-engage  → reference pose re-captured (no jump)

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
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Joy, JointState
from std_msgs.msg import Bool
from interbotix_xs_msgs.msg import JointGroupCommand
from scm_recording_msgs.srv import StartRecording, EndRecording
from rby1_ik.rby1_ik import Rby1Ik


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
        self.declare_parameter('pos_scale',      2.0)
        self.declare_parameter('ik_dt',          0.05)
        self.declare_parameter('publish_rate',   20.0)
        self.declare_parameter('pedal_button_index', 0)
        self.declare_parameter('pedal_record_index', 2)
        self.declare_parameter('recording_task_id',  0)

        urdf_path = self.get_parameter('urdf_path').value
        srdf_path = self.get_parameter('srdf_path').value
        topic_l   = self.get_parameter('topic_tracker_left').value
        topic_r   = self.get_parameter('topic_tracker_right').value
        topic_p   = self.get_parameter('topic_pedal').value
        topic_js  = self.get_parameter('topic_joint_state').value
        topic_cmd = self.get_parameter('topic_teleop_command').value

        self._pos_scale       = self.get_parameter('pos_scale').value
        self._ik_dt           = self.get_parameter('ik_dt').value
        self._pedal_idx       = self.get_parameter('pedal_button_index').value
        self._pedal_rec_idx   = self.get_parameter('pedal_record_index').value
        self._recording_task  = self.get_parameter('recording_task_id').value
        rate_hz               = self.get_parameter('publish_rate').value

        # Coordinate transform: tracker world frame (ROS, Z-up) → robot base frame
        # world +Y (forward) → robot +X,  world +X (right) → robot -Y,  world +Z (up) → robot +Z
        self._v2r_R = np.array([[0.,  1.,  0.],
                                [-1.,  0.,  0.],
                                [ 0.,  0.,  1.]])

        # IK solver
        if not urdf_path or not srdf_path:
            self.get_logger().error('urdf_path / srdf_path not set! Check config yaml.')
            raise RuntimeError('Missing URDF/SRDF paths')
        self._ik = Rby1Ik(urdf_path, srdf_path)
        self.get_logger().info('[vive_rby1] IK solver ready')

        # State
        self._tracker_l: PoseStamped | None = None
        self._tracker_r: PoseStamped | None = None
        self._joint_state: JointState | None = None

        # Pedal state
        self._pedal_engage_active = False   # buttons[0]: current hold state
        self._pedal_record_prev   = False   # buttons[2]: previous state for edge detect

        # Recording state
        self._recording_active = False

        # Clutch state
        self._engaged = False
        self._ref_l: pin.SE3 | None = None
        self._ref_r: pin.SE3 | None = None
        self._ee_l_0: pin.SE3 | None = None
        self._ee_r_0: pin.SE3 | None = None

        # Subscribers
        self.create_subscription(PoseStamped, topic_l,  self._cb_tracker_l,    10)
        self.create_subscription(PoseStamped, topic_r,  self._cb_tracker_r,    10)
        self.create_subscription(Joy,         topic_p,  self._cb_pedal,        10)
        self.create_subscription(JointState,  topic_js, self._cb_joint_state,  10)

        # Publishers
        self._pub_cmd = self.create_publisher(JointGroupCommand, topic_cmd, 10)
        self._pub_rec = self.create_publisher(Bool, '/teleop/recording', 10)

        # Recording service clients (scm_recording_msgs — source external ws before launch)
        self._cli_start_rec = self.create_client(StartRecording, '/recording/start')
        self._cli_end_rec   = self.create_client(EndRecording,   '/recording/end')

        # Timer
        self._timer = self.create_timer(1.0 / rate_hz, self._timer_cb)

        self.get_logger().info('[vive_rby1] Ready — hold pedal 0 to engage')

    # ------------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------------

    def _cb_tracker_l(self, msg: PoseStamped):
        self._tracker_l = msg

    def _cb_tracker_r(self, msg: PoseStamped):
        self._tracker_r = msg

    def _cb_joint_state(self, msg: JointState):
        self._joint_state = msg
        if self._joint_state is not None:
            self._ik.update_from_joint_state(
                list(self._joint_state.name),
                list(self._joint_state.position),
            )

    def _cb_pedal(self, _msg: Joy):
        # ---- Pedal 0: dead-man switch (hold to engage) ----
        if self._pedal_idx < len(_msg.buttons):
            active = bool(_msg.buttons[self._pedal_idx])
            if active and not self._pedal_engage_active:
                self._on_engage()
            elif not active and self._pedal_engage_active:
                self._on_disengage()
            self._pedal_engage_active = active

        # ---- Pedal 1: spare ----

        # ---- Pedal 2: recording toggle (rising edge) ----
        if self._pedal_rec_idx < len(_msg.buttons):
            pressed = bool(_msg.buttons[self._pedal_rec_idx])
            if pressed and not self._pedal_record_prev:
                self._toggle_recording()
            self._pedal_record_prev = pressed

    # ------------------------------------------------------------------
    # Recording toggle (pedal 2)
    # ------------------------------------------------------------------

    def _toggle_recording(self):
        if not self._recording_active:
            if not self._cli_start_rec.service_is_ready():
                self.get_logger().warn('[vive_rby1] StartRecording service not available')
                return
            req = StartRecording.Request()
            req.task_id = self._recording_task
            future = self._cli_start_rec.call_async(req)
            future.add_done_callback(self._on_start_recording_done)
        else:
            if not self._cli_end_rec.service_is_ready():
                self.get_logger().warn('[vive_rby1] EndRecording service not available')
                return
            future = self._cli_end_rec.call_async(EndRecording.Request())
            future.add_done_callback(self._on_end_recording_done)

    def _on_start_recording_done(self, future):
        result = future.result()
        if result.result:
            self._recording_active = True
            self.get_logger().info(
                f'[vive_rby1] Recording STARTED — episode {result.episode_id}')
        else:
            self.get_logger().error(f'[vive_rby1] StartRecording failed: {result.message}')
        self._pub_rec.publish(Bool(data=self._recording_active))

    def _on_end_recording_done(self, future):
        result = future.result()
        if result.result:
            self._recording_active = False
            self.get_logger().info('[vive_rby1] Recording ENDED')
        else:
            self.get_logger().error(f'[vive_rby1] EndRecording failed: {result.message}')
        self._pub_rec.publish(Bool(data=self._recording_active))

    # ------------------------------------------------------------------
    # Clutch engage / disengage
    # ------------------------------------------------------------------

    def _on_engage(self):
        """Capture reference poses when pedal 0 is pressed."""
        if self._tracker_l is None or self._tracker_r is None:
            self.get_logger().warn('Pedal pressed but trackers not ready — ignoring')
            return

        self._ref_l = pose_stamped_to_SE3(self._tracker_l)
        self._ref_r = pose_stamped_to_SE3(self._tracker_r)

        q_pin = self._ik.configuration.q
        fid_l = self._ik.robot.model.getFrameId('tracker_left')
        fid_r = self._ik.robot.model.getFrameId('tracker_right')
        self._ee_l_0 = self._ik.robot.framePlacement(q_pin, fid_l)
        self._ee_r_0 = self._ik.robot.framePlacement(q_pin, fid_r)

        self._engaged = True
        self.get_logger().info('Clutch ENGAGED — teleoperation active')

    def _on_disengage(self):
        self._engaged = False
        self.get_logger().info('Clutch DISENGAGED — robot holding position')

    # ------------------------------------------------------------------
    # Main timer: compute IK and publish command
    # ------------------------------------------------------------------

    def _timer_cb(self):
        if self._tracker_l is None or self._tracker_r is None:
            return
        # Publish only while pedal is held — no auto-engage
        if not self._engaged or self._ref_l is None or self._ee_l_0 is None:
            return

        tracker_l_now = pose_stamped_to_SE3(self._tracker_l)
        tracker_r_now = pose_stamped_to_SE3(self._tracker_r)

        # Delta position in world frame → robot frame
        delta_l = tracker_l_now.translation - self._ref_l.translation
        delta_r = tracker_r_now.translation - self._ref_r.translation

        target_pos_l = self._ee_l_0.translation + self._pos_scale * (self._v2r_R @ delta_l)
        target_pos_r = self._ee_r_0.translation + self._pos_scale * (self._v2r_R @ delta_r)

        # Delta rotation in world frame → robot frame via similarity transform
        dR_l = tracker_l_now.rotation @ self._ref_l.rotation.T
        dR_r = tracker_r_now.rotation @ self._ref_r.rotation.T

        dR_l_robot = self._v2r_R @ dR_l @ self._v2r_R.T
        dR_r_robot = self._v2r_R @ dR_r @ self._v2r_R.T

        target_rot_l = dR_l_robot @ self._ee_l_0.rotation
        target_rot_r = dR_r_robot @ self._ee_r_0.rotation

        l_SE3 = pin.SE3(target_rot_l, target_pos_l)
        r_SE3 = pin.SE3(target_rot_r, target_pos_r)

        q20 = self._ik.solve_ik_to_q20(l_SE3, r_SE3, self._ik_dt)

        self._publish_command(q20)

    def _publish_command(self, q20: np.ndarray):
        cmd = JointGroupCommand()
        cmd.name = 'All'
        cmd.cmd = np.concatenate((q20, np.array([0., 0.]))).tolist()
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
