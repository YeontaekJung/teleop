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


def openvr_mat34_to_ros(mat34) -> tuple[np.ndarray, np.ndarray]:
    """
    Convert OpenVR 3x4 matrix to (ros_pos, ros_rot_matrix).
    """
    T = np.array([
        [mat34[0][0], mat34[0][1], mat34[0][2], mat34[0][3]],
        [mat34[1][0], mat34[1][1], mat34[1][2], mat34[1][3]],
        [mat34[2][0], mat34[2][1], mat34[2][2], mat34[2][3]],
        [0, 0, 0, 1],
    ])
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

    DEVICE_NAMES = ['station_left', 'station_right', 'tracker_left', 'tracker_right', 'tracker_body']

    def __init__(self):
        super().__init__('vive_tracker_node')

        # Declare & read parameters
        self.declare_parameter('serial_station_left',  'LHB-E369DC69')
        self.declare_parameter('serial_station_right', 'LHB-ACA3FF29')
        self.declare_parameter('serial_tracker_left',  'LHR-83AA739B')
        self.declare_parameter('serial_tracker_right', 'LHR-22E4DDD6')
        self.declare_parameter('serial_tracker_body',  '')
        self.declare_parameter('publish_rate',         100.0)
        self.declare_parameter('topic_tracker_left',   '/teleop/tracker/left')
        self.declare_parameter('topic_tracker_right',  '/teleop/tracker/right')
        self.declare_parameter('topic_tracker_body',   '/teleop/tracker/body')

        serials = {
            'station_left':  self.get_parameter('serial_station_left').value,
            'station_right': self.get_parameter('serial_station_right').value,
            'tracker_left':  self.get_parameter('serial_tracker_left').value,
            'tracker_right': self.get_parameter('serial_tracker_right').value,
        }
        serial_body = self.get_parameter('serial_tracker_body').value
        if serial_body:
            serials['tracker_body'] = serial_body

        # reverse map: serial → name
        self._serial_to_name = {v: k for k, v in serials.items()}
        self._name_to_idx = {n: i for i, n in enumerate(self.DEVICE_NAMES)}

        rate_hz = self.get_parameter('publish_rate').value
        topic_l = self.get_parameter('topic_tracker_left').value
        topic_r = self.get_parameter('topic_tracker_right').value
        topic_b = self.get_parameter('topic_tracker_body').value

        # Publishers
        self._pub_left  = self.create_publisher(PoseStamped, topic_l, 10)
        self._pub_right = self.create_publisher(PoseStamped, topic_r, 10)
        self._pub_body  = self.create_publisher(PoseStamped, topic_b, 10)

        # Device state: pose (4x4) for each of 5 devices
        self._T = [None] * 5
        self._alive = [False] * 5
        self._alive_prev = [False] * 5

        # Station alignment rotation (3x3, applied to tracker positions/orientations)
        self._station_rot: np.ndarray | None = None
        self._initialized = False

        # OpenVR init
        self._vr = openvr.init(openvr.VRApplication_Other)
        self.get_logger().info('OpenVR initialized')

        # Timer
        period = 1.0 / rate_hz
        self._timer = self.create_timer(period, self._timer_cb)

    # ------------------------------------------------------------------

    def _poll_devices(self) -> bool:
        """Read all device poses from SteamVR. Returns True if both trackers alive."""
        poses = self._vr.getDeviceToAbsoluteTrackingPose(
            openvr.TrackingUniverseStanding, 0, openvr.k_unMaxTrackedDeviceCount
        )

        for i in range(5):
            self._alive[i] = False

        for i, pose in enumerate(poses):
            if not pose.bPoseIsValid:
                continue
            serial = get_serial(self._vr, i)
            if serial not in self._serial_to_name:
                self.get_logger().warn(
                    f'Unknown device serial: {serial} (idx {i}) — add to trackers.yaml if needed',
                    throttle_duration_sec=10.0)
                continue
            name = self._serial_to_name[serial]
            idx = self._name_to_idx[name]
            m = pose.mDeviceToAbsoluteTracking
            self._T[idx] = np.array([
                [m[0][0], m[0][1], m[0][2], m[0][3]],
                [m[1][0], m[1][1], m[1][2], m[1][3]],
                [m[2][0], m[2][1], m[2][2], m[2][3]],
                [0, 0, 0, 1],
            ])
            self._alive[idx] = True

        # Log connect/disconnect events
        for i, name in enumerate(self.DEVICE_NAMES):
            if self._alive[i] != self._alive_prev[i]:
                if self._alive[i]:
                    self.get_logger().info(f'Device connected:    {name}')
                else:
                    self.get_logger().warn(f'Device disconnected: {name}')
            self._alive_prev[i] = self._alive[i]

        # Both stations (idx 0,1) and both hand trackers (idx 2,3) must be alive; body (idx 4) is optional
        return all(self._alive[:4])

    def _make_pose_stamped(self, device_idx: int) -> PoseStamped:
        T = self._T[device_idx]
        pos_ros, rot_ros = openvr_mat34_to_ros(T)

        # Apply station alignment rotation
        if self._station_rot is not None:
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

        # One-time station alignment
        if not self._initialized:
            p1 = openvr_to_ros_pos(self._T[0][:3, 3])  # station_left
            p2 = openvr_to_ros_pos(self._T[1][:3, 3])  # station_right
            self._station_rot = get_rotation_to_align_stations(p1, p2)
            self.get_logger().info('Station alignment computed.')
            self._initialized = True

        # Publish tracker poses (idx 2=left, 3=right, 4=body optional)
        self._pub_left.publish(self._make_pose_stamped(2))
        self._pub_right.publish(self._make_pose_stamped(3))
        if self._alive[4]:
            self._pub_body.publish(self._make_pose_stamped(4))

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
