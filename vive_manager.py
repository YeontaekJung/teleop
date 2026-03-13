from scipy.spatial.transform import Rotation as R
import numpy as np
import time
import threading
import openvr
import zmq


STATION_LEFT = 0
STATION_RIGHT = 1
CONTROLLER_LEFT = 2
CONTROLLER_RIGHT = 3

STATIONS = [STATION_LEFT, STATION_RIGHT]
CONTROLLERS = [CONTROLLER_LEFT, CONTROLLER_RIGHT]

# EXTRA_STATION =

CONTROLLER_NUMBER = 2
ALIGN_STATION_NUMBER = 2
EXTRA_STATION_NUMBER = 0


DEVICE_LEN = CONTROLLER_NUMBER + ALIGN_STATION_NUMBER + EXTRA_STATION_NUMBER


#updated, check L/R
SERIAL_STATION_LEFT = "LHB-E369DC69"
SERIAL_STATION_RIGHT = "LHB-ACA3FF29"
SERIAL_TRACKER_LEFT = "LHR-83AA739B"
SERIAL_TRACKER_RIGHT = "LHR-22E4DDD6"

        # LHR-22E4DDD6
        # LHR-83AA739B
        # LHB-ACA3FF29
        # LHB-E369DC69
# trac -> robot
# x -> y
# y -> z
# z -> x

class ViveDevice:
    def __init__(self):
        pass

    serial: str = None
    name: str = None
    idx: int = None

    T = None

    lin_vel = None
    ang_vel = None

    dict = None

    found_signal = False
    lost_signal = False

    alive = False
    alive_prev = False

    


def openvr_to_ros_coords(p):
    """OpenVR 좌표계를 ROS 좌표계로 변환"""
    x, y, z = p
    return np.array([-z, -x, y])


def openvr_to_ros_velocity(v):
    return np.array([-v[2], -v[0], v[1]])



def is_tracking_reference(vr_system, index):
    return (
        vr_system.getTrackedDeviceClass(index)
        == openvr.TrackedDeviceClass_TrackingReference
    )



def get_rotation_to_align_y(p1, p2):
    """
    두 스테이션의 x좌표가 같아지도록 Z축 회전행렬 반환
    즉, 두 점을 Y축에 정렬되게 회전
    """
    vec = p2 - p1
    # angle = np.arctan2(vec[0], vec[1])  # ROS 좌표계: x: forward, y: left
    angle = np.arctan2(vec[1], vec[0])
    rot_z = R.from_euler("z", -angle).as_matrix()
    return rot_z


def get_device_serial_number(vr_system, index):
    return vr_system.getStringTrackedDeviceProperty(
        index, openvr.Prop_SerialNumber_String
    )


class VivePublisher():
    def __init__(self):
        #zmq_pub
        self.no_hw_test =  False
        

        self.serial_station_left =  SERIAL_STATION_LEFT
        self.serial_station_right =  SERIAL_STATION_RIGHT
        self.serial_tracker_left =  SERIAL_TRACKER_LEFT
        self.serial_tracker_right =  SERIAL_TRACKER_RIGHT

        if self.no_hw_test:
            print("no_hw_test mode! ")
        else:
            print("steamvr mode! ")

        self.serial_list = {
            "station_left": self.serial_station_left,
            "station_right": self.serial_station_right,
            "tracker_left": self.serial_tracker_left,
            "tracker_right": self.serial_tracker_right,
        }

        # Only initialize OpenVR if not using null tester
        if not self.no_hw_test:
            self.vr = openvr.init(openvr.VRApplication_Other)
            self.device = [ViveDevice() for _ in range(DEVICE_LEN)]
        else:
            self.vr = None

        self.detected_size = 0
        self.rotation = None
        self.initialization = False

        #zmq
        context = zmq.Context()
        self.zmq_socket = context.socket(zmq.PUB)
        self.zmq_socket.bind("tcp://localhost:5885")
        #zmq_str = str(T1)+","+str(T2)
        #self.zmq_socket.send_string(zmq_str)


        # Use appropriate timer callback based on configuration
        if self.no_hw_test:
            self.vive_thread = threading.Thread(target=self.timer_callback_no_hw_test)
        else:
            self.vive_thread = threading.Thread(target=self.timer_callback)
        self.vive_thread.start()

    def get_device_info(self):
        poses = self.vr.getDeviceToAbsoluteTrackingPose(
            openvr.TrackingUniverseStanding, 0, openvr.k_unMaxTrackedDeviceCount
        )

        for i in range(DEVICE_LEN):
            self.device[i].alive = False

        valid_device_size = 0

        for i, pose in enumerate(poses):
            if pose.bPoseIsValid:
                valid_device_size += 1

                m = pose.mDeviceToAbsoluteTracking
                transform = np.array(
                    [
                        [m[0][0], m[0][1], m[0][2], m[0][3]],
                        [m[1][0], m[1][1], m[1][2], m[1][3]],
                        [m[2][0], m[2][1], m[2][2], m[2][3]],
                        [0, 0, 0, 1],
                    ]
                )

                serial = get_device_serial_number(self.vr, i)

                # print(serial)

                # Map serial to device index using self.serial_list
                serial_to_name = {v: k for k, v in self.serial_list.items()}
                if serial in serial_to_name:
                    name_key = serial_to_name[serial]
                    # Determine device index based on name_key
                    name_to_idx = {
                        "station_left": 0,
                        "station_right": 1,
                        "tracker_left": 2,
                        "tracker_right": 3,
                    }
                    idx = name_to_idx.get(name_key, None)
                    if idx is not None:
                        self.device[idx].name = name_key.upper()
                        self.device[idx].alive = True
                        self.device[idx].idx = i
                        self.device[idx].T = transform
                        self.device[idx].lin_vel = pose.vVelocity
                        self.device[idx].ang_vel = pose.vAngularVelocity

        all_connected = True

        for i in range(DEVICE_LEN):
            if self.device[i].alive is not self.device[i].alive_prev:
                if self.device[i].alive:
                    print(
                        f"Device connected : {self.device[i].name}, idx : {self.device[i].idx}, serial : {self.device[i].serial}"
                    )

                else:
                    print(
                        f"Device disconnected : {self.device[i].name}"
                    )
            self.device[i].alive_prev = self.device[i].alive

            all_connected = all_connected and self.device[i].alive

        self.detected_size = valid_device_size

        # Check if all expected devices were found
        expected_devices_found = sum(1 for i in range(DEVICE_LEN) if self.device[i].alive)
        if expected_devices_found < DEVICE_LEN:
            # Get all detected serials for warning message
            detected_serials = []
            for i, pose in enumerate(poses):
                if pose.bPoseIsValid:
                    serial = get_device_serial_number(self.vr, i)
                    if serial:
                        detected_serials.append(serial)
            print(
                f"Expected {DEVICE_LEN} devices but only {expected_devices_found} found! "
                f"Expected: {list(self.serial_list.values())}, "
                f"Detected: {detected_serials}"
            )

        connected_controllers = (
            self.device[CONTROLLER_LEFT].alive + self.device[CONTROLLER_RIGHT].alive
        )

        return connected_controllers

    def timer_callback_no_hw_test(self):
        # Get current time for sine wave calculation
        pass


    def timer_callback(self):
        loop_cnt = 0
        while True:
            loop_cnt +=1            
            time.sleep(0.01)

            # if self.get_device_info():
            if not self.get_device_info():
                # self.get_logger().info(f'Waiting for devices ... detected : {self.detected_size}/{DEVICE_LEN}')
                try:                
                    time.sleep(0.1)
                except KeyboardInterrupt:
                    pass
                pass
            else:            
                if self.initialization is False:
                    p1_ros = openvr_to_ros_coords(
                        self.device[STATION_LEFT].T[:3, 3]
                    )
                    p2_ros = openvr_to_ros_coords(
                        self.device[STATION_RIGHT].T[:3, 3]
                    )
                    self.rotation = get_rotation_to_align_y(p1_ros, p2_ros)
                    print("Alignment complete.")
                    self.initialization = True
                else:
                    for i in CONTROLLERS:
                        if self.device[i].alive:
                            

                            pose = self.device[i].T
                            pos_vr = pose[:3, 3]
                            rot_vr = R.from_matrix(pose[:3, :3])

                            # OpenVR → ROS 좌표계
                            pos_ros = openvr_to_ros_coords(pos_vr)
                            rot_mat_ros = np.stack(
                                [
                                    openvr_to_ros_coords(pose[:3, 0]),
                                    openvr_to_ros_coords(pose[:3, 1]),
                                    openvr_to_ros_coords(pose[:3, 2]),
                                ],
                                axis=1,
                            )

                            # rot_ros = R.from_matrix(rot_mat_ros)

                            # Z축 회전 보정
                            # aligned_pos = self.rotation @ pos_ros
                            # aligned_rot = R.from_matrix(self.rotation) * rot_ros

                            # aligned_vel = self.rotation @ openvr_to_ros_velocity(
                            #     self.device[i].lin_vel
                            # )
                            # aligned_omega = self.rotation @ openvr_to_ros_velocity(
                            #     self.device[i].ang_vel
                            # )

                            # quat = aligned_rot.as_quat()  # [x, y, z, w]
                    T1 = self.device[CONTROLLER_LEFT].T
                    T2 = self.device[CONTROLLER_RIGHT].T
                    zmq_str = str(T1)+","+str(T2)
                    
                    self.zmq_socket.send_string(zmq_str)
                    if loop_cnt%20==0:
                        print(zmq_str)
                    



def main(args=None):

    VivePublisher()
    while True:
        time.sleep(0.5)



if __name__ == "__main__":
    main()