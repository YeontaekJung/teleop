"""
vive_rby1_node.py
Vive Tracker teleoperation bridge for RB-Y1.

Subscriptions:
  /teleop/tracker/left    geometry_msgs/PoseStamped  (from vive_ros2)
  /teleop/tracker/right   geometry_msgs/PoseStamped  (from vive_ros2)
  /teleop/pedal           sensor_msgs/Joy            (from pedal driver)
  /rby1_status_joint      sensor_msgs/JointState     (from rby1 SDK)

Publication:
  /rby1_teleop_command    rby1_sdk_msgs/JointGroupCommand

Clutch logic:
  Pedal HELD   → tracking active (delta accumulates)
  Pedal RELEASE → command stops (robot holds position)
  On re-engage  → reference pose re-captured (no jump)

Delta computation (ROS frame):
  Δpos = tracker_now - tracker_ref   (in world frame, rotated to robot frame)
  target_EE = ee_pose_at_engage + Δpos
  target_orientation = tracker_now.orientation  (direct mapping)
"""

import numpy as np
import pinocchio as pin
from scipy.spatial.transform import Rotation as R

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Joy, JointState
from interbotix_xs_msgs.msg import JointGroupCommand
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
        self.declare_parameter('urdf_path', '')
        self.declare_parameter('srdf_path', '')
        self.declare_parameter('topic_tracker_left',   '/teleop/tracker/left')
        self.declare_parameter('topic_tracker_right',  '/teleop/tracker/right')
        self.declare_parameter('topic_pedal',          '/teleop/pedal')
        self.declare_parameter('topic_joint_state',    '/joint_states')
        self.declare_parameter('topic_teleop_command', '/rby1_teleop_command')
        self.declare_parameter('pos_scale',      1.0)
        self.declare_parameter('ik_dt',          0.05)
        self.declare_parameter('publish_rate',   20.0)
        self.declare_parameter('pedal_button_index', 0)

        urdf_path = self.get_parameter('urdf_path').value
        srdf_path = self.get_parameter('srdf_path').value
        topic_l   = self.get_parameter('topic_tracker_left').value
        topic_r   = self.get_parameter('topic_tracker_right').value
        topic_p   = self.get_parameter('topic_pedal').value
        topic_js  = self.get_parameter('topic_joint_state').value
        topic_cmd = self.get_parameter('topic_teleop_command').value

        self._pos_scale   = self.get_parameter('pos_scale').value
        self._ik_dt       = self.get_parameter('ik_dt').value
        self._pedal_idx   = self.get_parameter('pedal_button_index').value
        rate_hz           = self.get_parameter('publish_rate').value

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
        self._pedal_active = False

        # Clutch state
        self._engaged = False
        self._ref_l: pin.SE3 | None = None   # tracker pose at engage
        self._ref_r: pin.SE3 | None = None
        self._ee_l_0: pin.SE3 | None = None  # EE pose at engage (from FK / joint state)
        self._ee_r_0: pin.SE3 | None = None

        # Subscribers
        self.create_subscription(PoseStamped, topic_l,  self._cb_tracker_l, 10)
        self.create_subscription(PoseStamped, topic_r,  self._cb_tracker_r, 10)
        self.create_subscription(Joy,         topic_p,  self._cb_pedal,     10)
        self.create_subscription(JointState,  topic_js, self._cb_joint_state, 10)

        # Publisher
        self._pub_cmd = self.create_publisher(JointGroupCommand, topic_cmd, 10)

        # Timer
        self._timer = self.create_timer(1.0 / rate_hz, self._timer_cb)

    # ------------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------------

    def _cb_tracker_l(self, msg: PoseStamped):
        self._tracker_l = msg

    def _cb_tracker_r(self, msg: PoseStamped):
        self._tracker_r = msg

    def _cb_joint_state(self, msg: JointState):
        self._joint_state = msg
        # Keep IK config in sync with real robot
        if self._joint_state is not None:
            self._ik.update_from_joint_state(
                list(self._joint_state.name),
                list(self._joint_state.position),
            )

    def _cb_pedal(self, msg: Joy):
        if self._pedal_idx >= len(msg.buttons):
            return
        active = bool(msg.buttons[self._pedal_idx])

        if active and not self._pedal_active:
            self._on_engage()
        elif not active and self._pedal_active:
            self._on_disengage()

        self._pedal_active = active

    # ------------------------------------------------------------------
    # Clutch engage / disengage
    # ------------------------------------------------------------------

    def _on_engage(self):
        """Capture reference poses when pedal is pressed."""
        if self._tracker_l is None or self._tracker_r is None:
            self.get_logger().warn('Pedal pressed but trackers not ready — ignoring')
            return

        self._ref_l = pose_stamped_to_SE3(self._tracker_l)
        self._ref_r = pose_stamped_to_SE3(self._tracker_r)

        # Capture current EE pose via RobotWrapper FK
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
        if not self._engaged or self._ref_l is None or self._ee_l_0 is None:
            self._on_engage()
            return

        tracker_l_now = pose_stamped_to_SE3(self._tracker_l)
        tracker_r_now = pose_stamped_to_SE3(self._tracker_r)

        # Delta in world (ROS) frame
        delta_l = tracker_l_now.translation - self._ref_l.translation
        delta_r = tracker_r_now.translation - self._ref_r.translation

        # Target EE positions = pose at engage + scaled delta
        target_pos_l = self._ee_l_0.translation + self._pos_scale * delta_l
        target_pos_r = self._ee_r_0.translation + self._pos_scale * delta_r

        # Target EE orientations: rotate reference EE by tracker rotation delta
        dR_l = tracker_l_now.rotation @ self._ref_l.rotation.T
        dR_r = tracker_r_now.rotation @ self._ref_r.rotation.T

        target_rot_l = dR_l @ self._ee_l_0.rotation
        target_rot_r = dR_r @ self._ee_r_0.rotation

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
