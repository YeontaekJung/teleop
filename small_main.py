## ROS node
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
import threading

import os


from builtin_interfaces.msg import Duration
from std_msgs.msg import Int32, Float32MultiArray, String
from interbotix_xs_msgs.msg import JointGroupCommand
from geometry_msgs.msg import Twist, PoseArray
from sensor_msgs.msg import JointState as ros2JointState
from nav_msgs.msg import Odometry as ros2Odometry
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionServer
from rby1_core_msgs.action import Rby1Command
from lidar_auto_docking_messages.action import Dock

from PySide6.QtCore import QThread, QObject, Signal
from PySide6.QtCore import QSocketNotifier, Slot, Qt
import time
import zmq


## GUI
import sys
from PySide6.QtWidgets import QApplication, QMainWindow
from PySide6.QtCore import QFile, QTimer
import asyncio

import PySide6.QtAsyncio as QtAsyncio

from PySide6.QtUiTools import QUiLoader

import json

import numpy as np
from .rby1_ik_pink import Rby1IkPink
import pinocchio as pin



D2R = np.pi/180.0

q_ready_rarm = np.array([-8.68, -9.86,  1.89, -103.95,  0.37, 22.07, -10.35]) * D2R
q_ready_larm = np.array([-8.68,  9.86, -1.89, -103.95, -0.37, 22.07,  10.35]) * D2R
q_ready_body = np.array([0, 30, -60, 30, 0, 0]) * D2R

q_runbox_rarm_inter = np.array([-66.55, -62.77,  1.8, -72.55,  -212.39, 78.8, -37.62]) * D2R
q_runbox_larm_inter = np.array([-66.55, 62.77,  -1.8, -72.55,  212.39, 78.8, 37.62]) * D2R
q_runbox_rarm = np.array([-66.55, -62.77,  113.14, -72.55,  -212.39, 78.8, -37.62]) * D2R
q_runbox_larm = np.array([-66.55, 62.77,  -113.14, -72.55,  212.39, 78.8, 37.62]) * D2R
        

IDLE_STATE = "background-color: lightgray;"
ACTIVATE_STATE = "background-color: #A6D256;"
FAULT_STATE = "background-color: #ED325A;"

def S2D(sec_in_float):
    s = int(sec_in_float)
    ns = int((sec_in_float-s)*1e9)
    return Duration(sec=s, nanosec =ns)

def D2S(duration):
    return duration.sec*1.0 + duration.nanosec*1e-9



from geometry_msgs.msg import Pose as ros2Pose
from geometry_msgs.msg import Point as ros2Point
from sensor_msgs.msg import JointState as ros2JointState
from ament_index_python.packages import get_package_share_directory
from scipy.spatial.transform import Rotation
def SE3toPose(input_SE3):
    rotation_matrix = input_SE3[:3,:3]
    r = Rotation.from_matrix(rotation_matrix)
    q = r.as_quat()    
    pose_msg = ros2Pose()
    pose_msg.position.x = input_SE3[0,3]
    pose_msg.position.y = input_SE3[1,3]
    pose_msg.position.z = input_SE3[2,3]
    pose_msg.orientation.x = q[0]
    pose_msg.orientation.y = q[1]
    pose_msg.orientation.z = q[2]
    pose_msg.orientation.w = q[3]
    return pose_msg

class Shared():
    def __init__(self):
        self.new_command = False
        self.command_str = ''

        self.goal_handle = None

        self.send_random_teleop_msg = False

        self.send_dock = False
        self.joint_msg = ros2JointState()

        self.pink_teleop_msg = PoseArray()
        self.new_pink_teleop_msg = False

        self.vive_teleop_q20 = None
        self.new_vive_teleop_msg = False

        

class rosThread(QThread):
    def __init__(self, shared):
        super().__init__()
        self.shared = shared
        ########## ROS2 ###############
        rclpy.init(args=None)        
        
        self.node_ = Node('small_test_node')
   
        self.sub_status = self.node_.create_subscription(
            String,
            "/rby1_status",
            self.callback_status,
            2,
        )
        self.sub_joints = self.node_.create_subscription(
            ros2JointState,
            "/rby1_status_joint",
            self.callback_joints,
            2,
        )
        self.pub_teleop_msg = self.node_.create_publisher(
            JointGroupCommand,
            "/rby1_teleop_command",
            2
        )
        self.pub_impedance_teleop_msg = self.node_.create_publisher(
            JointGroupCommand,
            "/rby1_impedance_teleop_command",
            2
        )

        self.teleop_msg = JointGroupCommand()
        self.vive_teleop_msg = JointGroupCommand()

        self.pub_pink_teleop_msg = self.node_.create_publisher(
            PoseArray,
            "/rby1_pink_teleop_command",
            2
        )
        # action_client_task

        self.action_client_ = ActionClient(self.node_, Rby1Command, "/rby1_command")
        self.docking_action_client = ActionClient(self.node_, Dock, "Dock")

        self.result_future_ = None
        self.ready_right_q = np.array([-1.90, -1.16, 1.30, -1.32, -1.47, 1.08, 2.3])
        self.ready_left_q = np.array([-1.90, 1.16, -1.30, -1.32, 1.47, 1.08, -2.3])
        
        self.q0 = np.concatenate((self.ready_right_q, self.ready_left_q, np.array([0.,0.])))
        self.cnt = 0
        

        # self.timer_ = self.node_.create_timer(
        #     0.05, # 20hz
        #     self.send_loop)# too slow.... but ok for testing
        send_thread = threading.Thread(target=self.send_loop)
        send_thread.start()

    def callback_joints(self, msg):
        #will do something
        self.shared.joint_msg = msg
        pass

    def callback_status(self, msg):
        #msg to shared str
        self.shared.status_str = msg.data

    def response_callback(self, future):
        self.goal_handle = future.result()
        if not self.goal_handle.accepted:
            print('goal_rejected')
            return
        print('goal_accepted')
        self._get_result_future = self.goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.result_callback)
    
    def result_callback(self, future):
        response = future.result().result.response
        print("response : " + response)

    def feedback_callback_dock(self, feedback):
        print('dock_fb_pos: {0}'.format(feedback.feedback.dock_pose.pose.position))
        print('dock_fb_quat: {0}'.format(feedback.feedback.dock_pose.pose.orientation))


    def response_callback_dock(self, future):
        self.goal_handle_dock = future.result()
        if not self.goal_handle_dock.accepted:
            print('dock_goal_rejected')
            return
        print('dock_goal_accepted')
        self._get_result_future_dock = self.goal_handle_dock.get_result_async()
        self._get_result_future_dock.add_done_callback(self.result_callback_dock)

    def result_callback_dock(self, future):
        response = str(future.result().result.docked)
        print("dock_response : " + response)


    def send_loop(self):
        while True:
            time.sleep(0.05)
            # check shared, if command or reference is set, send to core node
            if self.shared.new_command:
                self.action_client_.wait_for_server()
                goal_msg = Rby1Command.Goal()
                goal_msg.command = self.shared.command_str
                self.result_future_ = self.action_client_.send_goal_async(goal_msg)
                self.result_future_.add_done_callback(self.response_callback)
                self.shared.new_command = False
            
                    #new send task
            if self.shared.send_dock:
                self.docking_action_client.wait_for_server()
                
                dock_param = [2.28,3.67,1.5708]
                dock_offset = [0.98, 0.0, 0.0]
                #self.docking_action_client.send_command_and_wait(dock_param, "tray", dock_offset)
                goal_msg = Dock.Goal()
                goal_msg.dock_pose.pose.position.x = dock_param[0]
                goal_msg.dock_pose.pose.position.y = dock_param[1]
                tyaw = dock_param[2]
                goal_msg.dock_pose.pose.orientation.z = np.sin(tyaw/2)
                goal_msg.dock_pose.pose.orientation.w = np.cos(tyaw/2)
                goal_msg.dock_id = "tray"
                oyaw = dock_offset[2]
                goal_msg.dock_offset.pose.position.x = dock_offset[0]
                goal_msg.dock_offset.pose.position.y = dock_offset[1]
                goal_msg.dock_offset.pose.orientation.z = np.sin(oyaw/2)
                goal_msg.dock_offset.pose.orientation.w = np.cos(oyaw/2)
                goal_msg.dock_pose.header.frame_id = "map"

                self.finished = False
                self.send_goal_future = self.docking_action_client.send_goal_async(goal_msg,feedback_callback=self.feedback_callback_dock)
                self.send_goal_future.add_done_callback(self.response_callback_dock)
                self.shared.send_dock = False

            if self.shared.send_random_teleop_msg:
                #make teleop msg
                #send~~
                self.teleop_msg.name = 'All'
                #set some appropriate msg
                self.teleop_msg.cmd = self.q0.tolist()

                
                amp = 1./180*np.pi # 1 deg
                freq = 60. # 3 sec
                for i in range(14):
                    theta = (self.cnt+i)/freq*2*np.pi
                    self.teleop_msg.cmd[i]+= amp*np.sin(theta)
                
                self.pub_teleop_msg.publish(self.teleop_msg)
                self.pub_impedance_teleop_msg.publish(self.teleop_msg)

                
                l_R0 = np.array([[-0.929905,  0.162539, -0.329935],[-0.174873, -0.98456, 0.007837],[-0.323567, 0.0649844, 0.943971]])
                l_p0 = np.array([0.708824, 0.183237,  1.24927])
                r_R0 = np.array([[-0.929903, -0.162539, -0.329942],[0.174875, -0.984559, -0.00784258],[-0.323573, -0.0649913, 0.943969]])
                r_p0 = np.array([0.708825, -0.18324,  1.24927])

                t = self.cnt*0.05 #20hz
                mu = (1-np.cos(t*1.))*0.5 # 0~1 
                mu2 = (1-np.cos(t*0.7))*0.5 # 0~1 
                l_pdes = (1-mu) * l_p0 + mu*(l_p0 + np.array([0,-0.3,-0.5]))
                r_pdes = (1-mu2) * r_p0 + mu2*(r_p0 + np.array([0,0.2,-0.5]))


                l_SE3 = pin.SE3(l_R0, l_pdes)
                r_SE3 = pin.SE3(r_R0, r_pdes)

                self.pink_teleop_msg = PoseArray()
                self.pink_teleop_msg.poses = []
                self.pink_teleop_msg.poses.append(SE3toPose(l_SE3.homogeneous))
                self.pink_teleop_msg.poses.append(SE3toPose(r_SE3.homogeneous))
                self.pub_pink_teleop_msg.publish(self.pink_teleop_msg)

                self.cnt+=1
                pass
            if self.shared.new_pink_teleop_msg:
                self.pub_pink_teleop_msg.publish(self.shared.pink_teleop_msg)
                self.shared.new_pink_teleop_msg = False

            if self.shared.new_vive_teleop_msg:
                self.vive_teleop_msg.name = 'All'
                q_arm_with_gripper = np.concatenate((self.shared.vive_teleop_q20, np.array([0.,0.])))
                self.vive_teleop_msg.cmd = q_arm_with_gripper.tolist()
                self.pub_teleop_msg.publish(self.vive_teleop_msg)
                self.pub_impedance_teleop_msg.publish(self.vive_teleop_msg)
                self.shared.new_vive_teleop_msg = False

            pass


    def run(self):
        print("test_core ROS2 node established")
        rclpy.spin(self.node_)
        rclpy.shutdown()


from PySide6.QtWidgets import *
from PySide6.QtCore import *
from PySide6.QtGui import *

class KeyPressFilter(QObject):

    def eventFilter(self, widget, event):
        if event.type() == QEvent.KeyPress:
            text = event.text()
            if event.modifiers():
                text = event.keyCombination().key().name.decode(encoding="utf-8")
            if text=='a':
                widget.pp.teleop_stop()
            if text=='b':
                widget.pp.impedance_teleop_start()
            if text=='c':
                pass
            #widget.BTN_pw_on.setText(text)
        return False

class MyWindow(QMainWindow):
    def __init__(self, shared):
        super().__init__()
        self.shared = shared
        ui_path = get_package_share_directory('rby1_core') + '/smallUI.ui'
        ui_file = QFile(ui_path)
        ui_file.open(QFile.ReadOnly)
        ld = QUiLoader()
        self.ui = ld.load(ui_file)
        # self.ui.setupUi(self)
        ui_file.close()
        self.ui.show()
        self.ui.setStyleSheet("QPushButton:disabled { color: gray }")
        self.ui.pp = self

        self.connected = False
        self.is_stream = False
        self.retry_connect = 0

        self.ui.BTN_connect.clicked.connect(self.connect_robot)
        self.ui.BTN_pw_on.clicked.connect(self.power_on)
        self.ui.BTN_pw_off.clicked.connect(self.power_off)
        self.ui.BTN_servo_on.clicked.connect(self.servo_on)
        #self.ui.BTN_servo_on_2.clicked.connect(self.servo_on_2)
        self.ui.BTN_gripper_init.clicked.connect(self.gripper_init)
        self.ui.BTN_control_enable.clicked.connect(self.control_enable)
        self.ui.BTN_err_reset.clicked.connect(self.error_reset)

        self.ui.BTN_ready_pose.clicked.connect(self.ready_pose)
        self.ui.BTN_zero_pose.clicked.connect(self.zero_pose)
        self.ui.BTN_vla_pose.clicked.connect(self.vla_pose)
        self.ui.BTN_vla_pose2.clicked.connect(self.vla_pose2)
        self.ui.BTN_clean_test.clicked.connect(self.clean_test)
        self.ui.BTN_stop_move.clicked.connect(self.stop_move)


        self.ui.BTN_stream_start.clicked.connect(self.stream_start)
        self.ui.BTN_stream_stop.clicked.connect(self.stream_stop)

        self.ui.BTN_dock_test.clicked.connect(self.dock_test)

        self.ui.BTN_teleop_start.clicked.connect(self.teleop_start)
        self.ui.BTN_impedance_teleop_start.clicked.connect(self.impedance_teleop_start)
        self.ui.BTN_teleop_pink_start.clicked.connect(self.teleop_pink_start)
        self.ui.BTN_teleop_stop.clicked.connect(self.teleop_stop)

        self.ui.CB_send_random_teleop_msg.stateChanged.connect(self.toggle_send_random_teleop)
        self.ui.RB_sim.clicked.connect(self.radio_ip)
        self.ui.RB_real.clicked.connect(self.radio_ip)
        
        self.ui.LE_connection_state.setStyleSheet(IDLE_STATE)
        self.ui.LE_control_state.setStyleSheet(IDLE_STATE)
        self.ui.LE_servo_state.setStyleSheet(IDLE_STATE)
        self.ui.LE_stream_state.setStyleSheet(IDLE_STATE)
        self.ui.LE_gripper_state.setStyleSheet(IDLE_STATE)
        urdf_path = get_package_share_directory('rby1_core') + '/robot.urdf'
        srdf_path = get_package_share_directory('rby1_core') + '/rb_y1.srdf'
        self.ik_pink = Rby1IkPink(urdf_path, srdf_path)


        self.timer = QTimer(self)
        self.timer.timeout.connect(lambda: asyncio.ensure_future(self.update_display()))  # asyncio.ensure_future()로 호출
        self.timer.setInterval(300)
        self.timer.start()
        self.is_tracking_first = True
        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.SUB)
        self.socket.connect("tcp://localhost:5885")
        self.socket.setsockopt_string(zmq.SUBSCRIBE, "") # Subscribe to all topics

        # Get the socket's file descriptor
        try:
            # Use the ZMQ_FD option to get the file descriptor
            fd = self.socket.getsockopt(zmq.FD)
        except zmq.error.ZMQError as e:
            print(f"Error getting ZMQ_FD: {e}")
            sys.exit(1)

        # Create a QSocketNotifier to watch the socket FD for read events
        self.notifier = QSocketNotifier(fd, QSocketNotifier.Read)
        # Connect the 'activated' signal to a custom slot
        self.notifier.activated.connect(self.handle_zmq_message)
        self.eventFilter = KeyPressFilter(parent=self.ui)
        self.ui.installEventFilter(self.eventFilter)



    def string_to_numpy_SE3(self,s):
        numbers_str = s.replace('[', '').replace(']', '').split()
        numbers_flat = [float(x) for x in numbers_str]
        arr = np.array(numbers_flat).reshape(4, 4)
        return arr

    def handle_zmq_message(self):
        """Callback function to process incoming ZMQ messages."""
        # Ensure we can process messages without blocking the GUI loop
        try:
            # trac -> robot            
            v2r_R = np.array([[0.,-1.,0.],[-1.,0.,0.],[0.,0.,-1.]]) # for position
            invv2r_R  = np.linalg.inv(v2r_R)

            pos_scale = 0.7


            # Use zmq.NOBLOCK or check events before calling recv
            # The notifier ensures we only enter this when data is ready
            message = self.socket.recv_string(zmq.NOBLOCK)

            #print(f"Received: {message}")
            arr_strs = message.split(",")
            tr1_SE3 = pin.SE3(self.string_to_numpy_SE3(arr_strs[0]))
            tr2_SE3 = pin.SE3(self.string_to_numpy_SE3(arr_strs[1]))

            tracker_l_SE3 = tr1_SE3
            tracker_r_SE3 = tr2_SE3

            if self.is_tracking_first:
                self.is_tracking_first = False
                #update l_SE3_0 and r_SE3_0 from self.shared.joint_msg 
                self.ik_pink.update_from_joint_msg(self.shared.joint_msg)
                self.l_SE3_0 = self.ik_pink.tasks[0].transform_target_to_world
                self.r_SE3_0 = self.ik_pink.tasks[1].transform_target_to_world

                self.last_l_SE3 =  tracker_l_SE3.copy()
                self.last_r_SE3 =  tracker_r_SE3.copy()
                self.R_l = tracker_l_SE3.rotation.copy()
                self.invR_l = np.linalg.inv(self.R_l)
                self.R_r = tracker_r_SE3.rotation.copy()
                self.invR_r = np.linalg.inv(self.R_r)

                z_ref = np.array([0.,-1.,0.]) # not_changing, vertical
                y_ref = np.array([0.,0.,-1.]) # average(?) from x, y
                y_l = self.R_l[:,1].copy()
                y_r = self.R_r[:,1].copy()
                yaw_l = np.atan2(y_l[0],y_l[2])
                yaw_r = np.atan2(y_r[0],y_r[2])
                yaw_ref = 0.5*(yaw_l+yaw_r)
                
                y_ref[0] = np.sin(yaw_ref)
                y_ref[2] = np.cos(yaw_ref) # +? -?
                #x_l = #slice from left
                #x_r = #slice from right
                x_ref = np.cross(y_ref,z_ref)
                self.R_ref = np.vstack((x_ref,y_ref,z_ref)).T # is this correct?
                self.invR_ref = np.linalg.inv(self.R_ref)

                self.ik_pink.update_lim_from_current_configuration()
                
                # print("R_l")
                # print(self.R_l)
                # print("R_r")
                # print(self.R_r)
                # print("R_ref")
                # print(self.R_ref)


            trans_l = self.invR_ref@(tracker_l_SE3.translation - self.last_l_SE3.translation) # to local left tracker axis
            trans_r = self.invR_ref@(tracker_r_SE3.translation - self.last_r_SE3.translation) # to local left tracker axis

            l_p = self.l_SE3_0.translation + pos_scale*v2r_R@trans_l # to xyz axis(from tracker-local axis)
            r_p = self.r_SE3_0.translation + pos_scale*v2r_R@trans_r # to xyz axis(from tracker-local axis)

            l_R = v2r_R@self.invR_l@tracker_l_SE3.rotation@invv2r_R@self.l_SE3_0.rotation
            r_R = v2r_R@self.invR_r@tracker_r_SE3.rotation@invv2r_R@self.r_SE3_0.rotation
            
            
            l_SE3 = pin.SE3(l_R, l_p)
            r_SE3 = pin.SE3(r_R, r_p)
            if self.is_stream:
                pink_teleop_msg = PoseArray()
                pink_teleop_msg.poses = []
                pink_teleop_msg.poses.append(SE3toPose(l_SE3.homogeneous))
                pink_teleop_msg.poses.append(SE3toPose(r_SE3.homogeneous))
                self.shared.pink_teleop_msg = pink_teleop_msg
                self.shared.new_pink_teleop_msg = True
                q20 = self.ik_pink.solve_ik_to_q20(l_SE3,r_SE3, 0.05) # 20Hz?
                self.shared.vive_teleop_q20 = q20.copy()
                self.shared.new_vive_teleop_msg = True
            else:
                self.is_tracking_first = True

            while True: # flush zmq remaining buffer                
                message = self.socket.recv_string(zmq.NOBLOCK)
                #print("flushing")


        except zmq.error.ZMQError as e:
            if e.errno == zmq.EAGAIN:
                # No more messages pending in this iteration, exit loop
                pass
            else:
                print(f"ZMQ Error during receive: {e}")

    def radio_ip(self):
        if self.ui.RB_sim.isChecked():
            self.ui.LE_ip_port.setText("localhost:50051")
            return
        self.ui.LE_ip_port.setText("192.168.30.1:50051")

    def impedance_teleop_start(self):
        self.is_tracking_first = True 
        self.shared.command_str = "impedance_teleop_start"
        self.shared.new_command = True
        pass

    def teleop_start(self):
        self.is_tracking_first = True 
        self.shared.command_str = "teleop_start"
        self.shared.new_command = True
        pass

    def teleop_pink_start(self):
        self.is_tracking_first = True 
        # init starting position, later, start from current position from pink?
        self.shared.command_str = "teleop_pink_start"
        self.shared.new_command = True
        pass

    def teleop_stop(self):
        self.shared.command_str = "teleop_stop"
        self.shared.new_command = True
        pass

    def toggle_send_random_teleop(self, state):
        self.shared.send_random_teleop_msg = self.ui.CB_send_random_teleop_msg.isChecked()
        pass

    def stream_start(self):
        self.shared.command_str = "stream_start\nJointPosition"
        self.shared.new_command = True
        pass

        
    def stream_stop(self):
        self.shared.command_str = "stream_stop"
        self.shared.new_command = True
        pass

    def connect_robot(self):
        self.shared.command_str = "connect\n" + self.ui.LE_ip_port.text()
        if self.ui.CB_no_gripper.isChecked():
            self.shared.command_str +="\nno_gripper"
        self.shared.new_command = True
        pass

    def power_on(self):
        self.shared.command_str = "power_on"
        self.shared.new_command = True
        pass
        
        
    def power_off(self):
        self.shared.command_str = "power_off"
        self.shared.new_command = True
        pass
        
    def servo_on(self):
        self.shared.command_str = "servo_on"
        self.shared.new_command = True
        pass

    def servo_on_2(self):
        self.shared.command_str = "servo_on_no_wheel"
        self.shared.new_command = True
        pass
    
    def gripper_init(self):
        self.shared.command_str = "gripper_init"
        self.shared.new_command = True
        pass

    def control_enable(self):
        self.shared.command_str = "control_enable"
        self.shared.new_command = True
        pass
        
    def error_reset(self):
        self.shared.command_str = "error_reset"
        self.shared.new_command = True
        pass

    def stop_move(self):
        #self.shared.command_str = "stop_move"
        #self.shared.new_command = True
        self.shared.cancel_command = True
        pass

    def zero_pose(self):
        self.shared.command_str = "zero_pose"
        self.shared.new_command = True
        pass
        
    def ready_pose(self):
        self.shared.command_str = "ready_pose"
        self.shared.new_command = True
        pass

    def vla_pose(self):
        self.shared.command_str = "vla_pose"
        self.shared.new_command = True
        pass
    
    def vla_pose2(self):
        self.shared.command_str = "vla_pose2"
        self.shared.new_command = True
        pass

    def clean_test(self):
        self.shared.command_str = "clean_test"
        self.shared.new_command = True
        pass

    def dock_test(self):
        self.shared.send_dock = True
        
        pass

    
    async def update_display(self):

        try:
            data = json.loads(self.shared.status_str)
            control_state = data['control_state']
            power_state = data['power_state']
            servo_state = data['servo_state']
            stream_state = data['stream_state']
            gripper_state = data['gripper_state']

            if control_state == "State.Enabled":
                self.ui.LE_control_state.setText("Enabled")
                self.ui.LE_control_state.setStyleSheet(ACTIVATE_STATE)
            elif control_state == "State.MajorFault" or control_state == "State.MinorFault":
                self.ui.LE_control_state.setText("Fault")
                self.ui.LE_control_state.setStyleSheet(FAULT_STATE)
            else:
                self.ui.LE_control_state.setText("idle")
                self.ui.LE_control_state.setStyleSheet(IDLE_STATE)
                
            if power_state == "True":
                self.ui.LE_connection_state.setText("Power On")
                self.ui.LE_connection_state.setStyleSheet(ACTIVATE_STATE)
            else:
                self.ui.LE_connection_state.setText("Power Off")
                self.ui.LE_connection_state.setStyleSheet(FAULT_STATE)

            if servo_state == "True":
                self.ui.LE_servo_state.setText("Servo On")
                self.ui.LE_servo_state.setStyleSheet(ACTIVATE_STATE)
            else:
                self.ui.LE_servo_state.setText("Servo Off")
                self.ui.LE_servo_state.setStyleSheet(FAULT_STATE)

            if stream_state == "True":
                self.ui.LE_stream_state.setText("Stream On")
                self.ui.LE_stream_state.setStyleSheet(ACTIVATE_STATE)
                self.is_stream = True
            else:
                self.ui.LE_stream_state.setText("Stream Off")
                self.ui.LE_stream_state.setStyleSheet(FAULT_STATE)
                self.is_stream = False

            if gripper_state == "True":
                self.ui.LE_gripper_state.setText("Gripper Init")
                self.ui.LE_gripper_state.setStyleSheet(ACTIVATE_STATE)
            else:
                self.ui.LE_gripper_state.setText("Gripper Fail")
                self.ui.LE_gripper_state.setStyleSheet(FAULT_STATE)
        except:
            pass
            # do_nothing


def main():
    shared_ = Shared()

    ### ROS2 ####
    ros_thread = rosThread(shared = shared_)
    ros_thread.start()

    app = QApplication(sys.argv)
    myWindow = MyWindow(shared = shared_)
    QtAsyncio.run()


    

if __name__ == "__main__": 
    main()