"""
rby1_ik.py
Differential IK for RB-Y1 using pink + pinocchio.

Cleaned-up version of rby1_ik_pink.py (reference at repo root).
Key changes vs reference:
  - solver: 'scs' → 'proxqp'  (much faster)
  - torso joints are NOT zeroed mid-solve; they are excluded via velocity mask
  - solve_ik_to_q20() signature unchanged for compatibility
"""

import numpy as np
import pinocchio as pin
import pink
from pink import solve_ik
from pink.barriers import SelfCollisionBarrier
from pink.tasks import FrameTask, PostureTask
from pink.utils import process_collision_pairs


# ---------------------------------------------------------------------------
# Joint name helpers (matches RB-Y1 URDF)
# ---------------------------------------------------------------------------

def get_rby1_joint_name_list():
    return [
        "right_wheel", "left_wheel",
        "torso_0", "torso_1", "torso_2", "torso_3", "torso_4", "torso_5",
        "right_arm_0", "right_arm_1", "right_arm_2", "right_arm_3",
        "right_arm_4", "right_arm_5", "right_arm_6",
        "left_arm_0",  "left_arm_1",  "left_arm_2",  "left_arm_3",
        "left_arm_4",  "left_arm_5",  "left_arm_6",
        "head_0", "head_1",
    ]

def get_rby1_body_joint_name_list():   # 20 DOF (torso + both arms)
    return get_rby1_joint_name_list()[2:22]

def get_rby1_torso_joint_name_list():  # 6 DOF
    return get_rby1_joint_name_list()[2:8]


# ---------------------------------------------------------------------------
# IK class
# ---------------------------------------------------------------------------

class Rby1Ik:
    """
    Differential IK solver for RB-Y1.

    Usage:
        ik = Rby1Ik('/path/to/rby1.urdf', '/path/to/rby1.srdf')
        ik.update_configuration(q_current_pin)   # call each cycle with current robot state
        q20 = ik.solve_ik_to_q20(l_SE3, r_SE3, dt)
    """

    def __init__(self, urdf_path: str, srdf_path: str):
        self._body_joints  = get_rby1_body_joint_name_list()
        self._torso_joints = get_rby1_torso_joint_name_list()
        self._init_pink(urdf_path, srdf_path)

        q_ref = pin.neutral(self.robot.model)
        self.update_configuration(q_ref)

    # ------------------------------------------------------------------
    # Internal setup
    # ------------------------------------------------------------------

    def _init_pink(self, urdf_path: str, srdf_path: str):
        self._solver = 'proxqp'   # faster than 'scs'

        self.robot = pin.RobotWrapper.BuildFromURDF(
            filename=urdf_path,
            package_dirs=['.'],
            root_joint=None,
        )
        # Save original limits for reset
        self._llim = self.robot.model.lowerPositionLimit.copy()
        self._ulim = self.robot.model.upperPositionLimit.copy()
        print(f'[rby1_ik] URDF loaded: {self.robot}')

        self.robot.collision_data = process_collision_pairs(
            self.robot.model, self.robot.collision_model, srdf_path
        )

        # Tasks
        self._task_left  = FrameTask('tracker_left',  position_cost=50.0, orientation_cost=0.5)
        self._task_right = FrameTask('tracker_right', position_cost=50.0, orientation_cost=0.5)
        self._task_body  = FrameTask('link_torso_5',  position_cost=500.0, orientation_cost=20.0)
        self._task_posture = PostureTask(cost=1e-3)

        self.tasks = [
            self._task_left,
            self._task_right,
            self._task_body,
            self._task_posture,
        ]

        # Barriers
        self._collision_barrier = SelfCollisionBarrier(
            n_collision_pairs=len(self.robot.collision_model.collisionPairs),
            gain=20.0,
            safe_displacement_gain=1.0,
            d_min=0.02,
        )
        self.barriers = [self._collision_barrier]

        # Build velocity mask: zero out torso joints during solve
        # (torso stays fixed during teleoperation)
        self._torso_v_indices = []
        for name in self._torso_joints:
            jid = self.robot.model.getJointId(name)
            self._torso_v_indices.append(self.robot.model.joints[jid].idx_v)

    # ------------------------------------------------------------------
    # Configuration update  (call with current robot joint state each cycle)
    # ------------------------------------------------------------------

    def update_configuration(self, q_pin: np.ndarray):
        self.configuration = pink.Configuration(
            self.robot.model,
            self.robot.data,
            q_pin,
            collision_model=self.robot.collision_model,
            collision_data=self.robot.collision_data,
        )
        for task in self.tasks:
            task.set_target_from_configuration(self.configuration)

    def update_from_joint_state(self, joint_names: list, joint_positions: list):
        """Convenience: update from ROS JointState lists."""
        q_pin = pin.neutral(self.robot.model)
        for i, name in enumerate(self._body_joints):
            try:
                msg_idx = joint_names.index(name)
                jid = self.robot.model.getJointId(name)
                q_pin[self.robot.model.joints[jid].idx_q] = joint_positions[msg_idx]
            except ValueError:
                pass
        self.update_configuration(q_pin)

    # ------------------------------------------------------------------
    # Solve
    # ------------------------------------------------------------------

    def solve_ik_to_q20(self, l_SE3: pin.SE3, r_SE3: pin.SE3, dt: float) -> np.ndarray:
        """
        Solve one IK step.

        Args:
            l_SE3: target SE3 for left  end-effector (tracker_left frame)
            r_SE3: target SE3 for right end-effector (tracker_right frame)
            dt:    time step [s]

        Returns:
            q20: 20-DOF joint angles [torso(6) + right_arm(7) + left_arm(7)]
        """
        self._task_left.set_target(l_SE3)
        self._task_right.set_target(r_SE3)

        try:
            velocity = solve_ik(
                self.configuration,
                self.tasks,
                dt,
                solver=self._solver,
                barriers=self.barriers,
                safety_break=False,
            )

            # Zero torso velocities (torso stays fixed during teleoperation)
            for idx_v in self._torso_v_indices:
                velocity[idx_v] = 0.0

            # Clamp max velocity
            max_teleop_dq = 1.5
            max_abs = np.max(np.abs(velocity))
            if max_abs > max_teleop_dq:
                velocity = velocity / max_abs * max_teleop_dq

        except Exception as e:
            print(f'[rby1_ik] solve_ik failed: {e} → using zero velocity')
            velocity = np.zeros(self.robot.model.nv)

        self.configuration.integrate_inplace(velocity, dt)
        return self._q_pin_to_q20(self.configuration.q)

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    def _q_pin_to_q20(self, q_pin: np.ndarray) -> np.ndarray:
        q20 = np.zeros(20)
        for i, name in enumerate(self._body_joints):
            jid = self.robot.model.getJointId(name)
            q20[i] = q_pin[self.robot.model.joints[jid].idx_q]
        return q20

    def q_pin_from_q20(self, q20: np.ndarray) -> np.ndarray:
        q_pin = pin.neutral(self.robot.model)
        for i, name in enumerate(self._body_joints):
            jid = self.robot.model.getJointId(name)
            q_pin[self.robot.model.joints[jid].idx_q] = q20[i]
        return q_pin

    @property
    def current_q20(self) -> np.ndarray:
        return self._q_pin_to_q20(self.configuration.q)
