"""
vive_tracker_node.py
OpenVR Vive Tracker 3.0 → ROS2 geometry_msgs/PoseStamped

Topics published:
  /teleop/tracker/left   (geometry_msgs/PoseStamped)
  /teleop/tracker/right  (geometry_msgs/PoseStamped)

Coordinate frame conversion: OpenVR → ROS
  OpenVR: right-handed, Y-up
  ROS:    right-handed, Z-up
  Mapping: ros.x = -vr.z,  ros.y = -vr.x,  ros.z = vr.y

Station alignment (optional):
  If both base stations are detected, computes a Z-rotation to align
  the tracking universe with the robot frame. If stations are not
  configured or not found, alignment is skipped (identity rotation).
"""

import numpy as np
from scipy.spatial.transform import Rotation as R

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

import openvr


# ---------------------------------------------------------------------------
# Coordinate helpers (same logic as vive_manager.py reference)
# ---------------------------------------------------------------------------

def openvr_to_ros_pos(p: np.ndarray) -> np.ndarray:
    """OpenVR position → ROS position."""
    x, y, z = p
    return np.array([-z, -x, y])


def openvr_col_to_ros(col: np.ndarray) -> np.ndarray:
    """Convert one column of the OpenVR rotation matrix to ROS frame."""
    x, y, z = col
    return np.array([-z, -x, y])


def openvr_mat34_to_ros(T: np.ndarray) -> tuple:
    """Convert OpenVR 4x4 matrix to (ros_pos, ros_rot_matrix)."""
    pos_ros = openvr_to_ros_pos(T[:3, 3])
    rot_ros = np.stack([
        openvr_col_to_ros(T[:3, 0]),
        openvr_col_to_ros(T[:3, 1]),
        openvr_col_to_ros(T[:3, 2]),
    ], axis=1)
    return pos_ros, rot_ros


def rot_matrix_to_quat_xyzw(rot: np.ndarray) -> np.ndarray:
    return R.from_matrix(rot).as_quat()  # [x, y, z, w]


def get_serial(vr_system, idx: int) -> str:
    return vr_system.getStringTrackedDeviceProperty(
        idx, openvr.Prop_SerialNumber_String
    )


def get_rotation_to_align_stations(p1_ros: np.ndarray, p2_ros: np.ndarray) -> np.ndarray:
    """
    Return 3x3 Z-rotation matrix that aligns the vector p1→p2 with the Y-axis.
    Used to zero out the yaw offset of the SteamVR tracking universe.
    """
    vec = p2_ros - p1_ros
    angle = np.arctan2(vec[1], vec[0])
    return R.from_euler('z', -angle).as_matrix()


# ---------------------------------------------------------------------------
# ROS2 Node
# ---------------------------------------------------------------------------

class ViveTrackerNode(Node):

    def __init__(self):
        super().__init__('vive_tracker_node')

        # Declare & read parameters
        self.declare_parameter('serial_station_left',  '')
        self.declare_parameter('serial_station_right', '')
        self.declare_parameter('serial_tracker_left',  'LHR-83AA739B')
        self.declare_parameter('serial_tracker_right', 'LHR-22E4DDD6')
        self.declare_parameter('publish_rate',         100.0)
        self.declare_parameter('topic_tracker_left',   '/teleop/tracker/left')
        self.declare_parameter('topic_tracker_right',  '/teleop/tracker/right')

        serials = {
            'tracker_left':  self.get_parameter('serial_tracker_left').value,
            'tracker_right': self.get_parameter('serial_tracker_right').value,
        }
        sl = self.get_parameter('serial_station_left').value
        sr = self.get_parameter('serial_station_right').value
        if sl:
            serials['station_left'] = sl
        if sr:
            serials['station_right'] = sr

        # reverse map: serial → device name
        self._serial_to_name = {v: k for k, v in serials.items() if v}

        rate_hz = self.get_parameter('publish_rate').value
        topic_l = self.get_parameter('topic_tracker_left').value
        topic_r = self.get_parameter('topic_tracker_right').value

        # Publishers
        self._pub_left  = self.create_publisher(PoseStamped, topic_l, 10)
        self._pub_right = self.create_publisher(PoseStamped, topic_r, 10)

        # Device poses: name → 4x4 numpy array
        self._poses: dict = {}
        self._prev_alive: set = set()

        # Station alignment rotation (3x3)
        self._station_rot: np.ndarray = np.eye(3)
        self._initialized = False

        # OpenVR init
        self._vr = openvr.init(openvr.VRApplication_Other)
        self.get_logger().info('OpenVR initialized')

        # Timer
        self._timer = self.create_timer(1.0 / rate_hz, self._timer_cb)

    # ------------------------------------------------------------------

    def _poll_devices(self) -> bool:
        """Read all device poses from SteamVR. Returns True if both trackers alive."""
        self._poses.clear()

        raw_poses = self._vr.getDeviceToAbsoluteTrackingPose(
            openvr.TrackingUniverseStanding, 0, openvr.k_unMaxTrackedDeviceCount
        )

        for i, pose in enumerate(raw_poses):
            if not pose.bPoseIsValid:
                continue
            serial = get_serial(self._vr, i)
            if serial not in self._serial_to_name:
                continue
            name = self._serial_to_name[serial]
            m = pose.mDeviceToAbsoluteTracking
            self._poses[name] = np.array([
                [m[0][0], m[0][1], m[0][2], m[0][3]],
                [m[1][0], m[1][1], m[1][2], m[1][3]],
                [m[2][0], m[2][1], m[2][2], m[2][3]],
                [0, 0, 0, 1],
            ])

        # Log connect/disconnect events
        alive_now = set(self._poses.keys())
        for name in alive_now - self._prev_alive:
            self.get_logger().info(f'Device connected:    {name}')
        for name in self._prev_alive - alive_now:
            self.get_logger().warn(f'Device disconnected: {name}')
        self._prev_alive = alive_now

        return 'tracker_left' in self._poses and 'tracker_right' in self._poses

    def _make_pose_stamped(self, device_name: str) -> PoseStamped:
        T = self._poses[device_name]
        pos_ros, rot_ros = openvr_mat34_to_ros(T)

        # Apply station alignment rotation
        pos_ros = self._station_rot @ pos_ros
        rot_ros = self._station_rot @ rot_ros

        quat = rot_matrix_to_quat_xyzw(rot_ros)

        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'world'
        msg.pose.position.x = float(pos_ros[0])
        msg.pose.position.y = float(pos_ros[1])
        msg.pose.position.z = float(pos_ros[2])
        msg.pose.orientation.x = float(quat[0])
        msg.pose.orientation.y = float(quat[1])
        msg.pose.orientation.z = float(quat[2])
        msg.pose.orientation.w = float(quat[3])
        return msg

    def _timer_cb(self):
        if not self._poll_devices():
            self._initialized = False
            return

        # One-time initialization
        if not self._initialized:
            if 'station_left' in self._poses and 'station_right' in self._poses:
                p1 = openvr_to_ros_pos(self._poses['station_left'][:3, 3])
                p2 = openvr_to_ros_pos(self._poses['station_right'][:3, 3])
                self._station_rot = get_rotation_to_align_stations(p1, p2)
                self.get_logger().info('Station alignment computed.')
            else:
                self._station_rot = np.eye(3)
                self.get_logger().info('No stations found — skipping alignment.')
            self._initialized = True

        self._pub_left.publish(self._make_pose_stamped('tracker_left'))
        self._pub_right.publish(self._make_pose_stamped('tracker_right'))

    def destroy_node(self):
        openvr.shutdown()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ViveTrackerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
