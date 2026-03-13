import numpy as np
import pinocchio as pin
import pink
from pink import solve_ik
from pink.barriers import SelfCollisionBarrier
from pink.tasks import FrameTask, PostureTask
from pink.utils import process_collision_pairs
import qpsolvers

def get_rby1_joint_name_list():
    rby1_joint_name_list = ["right_wheel", "left_wheel",
    "torso_0", "torso_1", "torso_2", "torso_3", "torso_4", "torso_5",
    "right_arm_0", "right_arm_1", "right_arm_2", "right_arm_3", "right_arm_4", "right_arm_5", "right_arm_6",
    "left_arm_0", "left_arm_1", "left_arm_2", "left_arm_3", "left_arm_4", "left_arm_5", "left_arm_6",
    "head_0", "head_1"]
    
    return rby1_joint_name_list

def get_rby1_body_joint_name_list():# 20dof
    return get_rby1_joint_name_list()[2:22]

def get_rby1_torso_joint_name_list(): # 6dof
    return get_rby1_joint_name_list()[2:8]


class Rby1IkPink():
    def __init__(self, urdf_path, srdf_path):
        self.name_list = get_rby1_body_joint_name_list()
        self.torso_name_list = get_rby1_torso_joint_name_list()
        self.init_pink(urdf_path, srdf_path)
        q_ref = pin.neutral((self.robot.model))
        self.update_configuration(q_ref)
        #self.update_lim_from_current_configuration()


    def set_limit_no_pass_zero(self, joint_name, lim):        
        # do_not_pass_zero right_arm_5
        joint_id = self.robot.model.getJointId(joint_name)
        idx = self.robot.model.joints[joint_id].idx_q
        q_now = self.configuration.q[idx]
        if q_now > 0:
            self.robot.model.lowerPositionLimit[idx] = lim
        else:
            self.robot.model.upperPositionLimit[idx] = -lim

    
    def update_lim_from_current_configuration(self):
        #reset limits
        self.robot.model.lowerPositionLimit = self.llim.copy()
        self.robot.model.upperPositionLimit = self.ulim.copy()

        one_deg = 0.017
        # do_not_pass_zero right_arm_5        
        self.set_limit_no_pass_zero("right_arm_5", one_deg)
        # do not pass zero left_arm_5
        self.set_limit_no_pass_zero("left_arm_5", one_deg)
        # do_not_pass_zero right_arm_3
        self.set_limit_no_pass_zero("right_arm_3", one_deg*5)
        # do_not_pass_zero left_arm_3
        self.set_limit_no_pass_zero("left_arm_3", one_deg*5)
        
        pass

    def init_pink(self, urdf_path, srdf_path):
        self.solver = 'scs'#'proxqp' # 'osqp'
        self.robot = pin.RobotWrapper.BuildFromURDF(
            filename=urdf_path,
            package_dirs=["."],
            root_joint=None,
        )
        self.llim = self.robot.model.lowerPositionLimit.copy()
        self.ulim = self.robot.model.upperPositionLimit.copy()
        print(f"URDF description successfully loaded in {self.robot}")
        self.robot.collision_data = process_collision_pairs(
        self.robot.model, self.robot.collision_model, srdf_path) 
        print(self.robot.collision_data)


        #adjust limit
        #one_deg = 0.017
        #self.robot.model.upperPositionLimit = self.robot.model.upperPositionLimit-one_deg
        #self.robot.model.lowerPositionLimit = self.robot.model.lowerPositionLimit+one_deg

        

        # srdf from description

        #https://github.com/stephane-caron/pink/blob/main/examples/barriers/kukas_self_collision.py
        # lets copy from line 155
         # Pink tasks
        left_end_effector_task = FrameTask(
            "tracker_left",
            position_cost=50.0,  # [cost] / [m]
            orientation_cost=10.0,  # [cost] / [rad]
        )
        right_end_effector_task = FrameTask(
            "tracker_right",
            position_cost=50.0,  # [cost] / [m]
            orientation_cost=10.0,  # [cost] / [rad]
        )

        # Pink barriers
        collision_barrier = SelfCollisionBarrier(
            n_collision_pairs=len(self.robot.collision_model.collisionPairs),
            gain=20.0,
            safe_displacement_gain=1.0,
            d_min=0.02, # i reduced this
        )

        body_orientation_task = FrameTask(
            "link_torso_5",
            position_cost=500.0,  # [cost] / [m]
            orientation_cost=20.0,  # [cost] / [rad]
        )
        posture_task = PostureTask(
            cost=1e-3,  # [cost] / [rad]
        )

        self.barriers = [collision_barrier] # working, but too slow
        self.tasks = [left_end_effector_task, right_end_effector_task, body_orientation_task, posture_task]

    def update_configuration(self, q_now):
        self.configuration = pink.Configuration(
            self.robot.model,
            self.robot.data,
            q_now,
            collision_model=self.robot.collision_model,  # for self-collision barrier
            collision_data=self.robot.collision_data,
        )
        for task in self.tasks:
            task.set_target_from_configuration(self.configuration)
        self.limits = (
                self.configuration.model.velocity_limit
        )
    def update_from_joint_msg(self, joint_msg):
        self.update_configuration(self.q_pin_from_joint_msg(joint_msg))

    def q_pin_from_joint_msg(self, joint_msg): 
        q_pin = pin.neutral((self.robot.model))
        for i in range(len(self.name_list)):
            idx = self.robot.model.getJointId(self.name_list[i]) 
            joint = self.robot.model.joints[idx]
            try:
                msg_ind = joint_msg.name.index(self.name_list[i])
                q_pin[joint.idx_q] = joint_msg.position[msg_ind]
            except ValueError:
                q_pin = pin.neutral((self.robot.model))
                self.is_tracking_first = True # wait for next, without ValueError
        return q_pin 

    def q_20_from_pin(self, q_pin):
        q_20 = np.zeros(20)
        for i in range(len(self.name_list)):
            idx = self.robot.model.getJointId(self.name_list[i]) 
            joint = self.robot.model.joints[idx]
            q_20[i] = q_pin[joint.idx_q]

        return q_20

    def solve_ik_to_q20(self, l_SE3, r_SE3, dt):
        self.tasks[0].set_target(l_SE3)
        self.tasks[1].set_target(r_SE3)

        try:
            velocity = solve_ik(
                self.configuration,
                self.tasks,
                dt,
                solver=self.solver,
                barriers=self.barriers,
                #limits = self.limits, #why? not working now
                safety_break=False,
                
            )
            # here, set body_vel to zero
            for i in range(len(self.torso_name_list)):
                idx = self.robot.model.getJointId(self.torso_name_list[i])
                joint = self.robot.model.joints[idx] 
                velocity[joint.idx_v] = 0

            max_teleop_dq = 1.5 # simple single-value limit? hmm
            max_vel = np.max(velocity)
            min_vel = np.min(velocity)
            max_vel_abs = np.max((max_vel, -min_vel))
            if max_vel_abs > max_teleop_dq:
                velocity  = velocity/max_vel_abs*max_teleop_dq

        except Exception as e:
            print(e)
            print("infeasible solution, vel=0")
            velocity = np.zeros(26)
        
        
        self.configuration.integrate_inplace(velocity, dt)        
        q_20 = self.q_20_from_pin(self.configuration.q)
        
        # trim q_20 here (with limit)

        return q_20