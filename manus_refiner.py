import rclpy
from rclpy.node import Node
from manus_ros2_msgs.msg import ManusGlove
from inspire_hand_msgs.msg import InspireHandCtrl

MAX_INSPIRE = 1000


class ManusToInspire(Node):

    def __init__(self):
        super().__init__("manus_to_inspire")

        self.sub_l = self.create_subscription(
            ManusGlove, "/manus_glove_0", self.cb_left, 10)

        self.sub_r = self.create_subscription(
            ManusGlove, "/manus_glove_1", self.cb_right, 10)

        self.pub_l = self.create_publisher(
            InspireHandCtrl, "/rt/inspire_hand/ctrl/l", 2)

        self.pub_r = self.create_publisher(
            InspireHandCtrl, "/rt/inspire_hand/ctrl/r", 2)

        self.get_logger().info("Manus → Inspire Dual Hand Control")

    # --------------------------------------------------
    # 유틸
    # --------------------------------------------------
    def clamp(self, v, min_v, max_v):
        return max(min_v, min(v, max_v))

    def weighted_flex(self, mcp, pip, dip):
        return 0.25*mcp + 0.55*pip + 0.20*dip

    def flex_to_inspire(self, flex_deg, max_deg=95.0, invert=True):
        flex_deg = self.clamp(flex_deg, 0.0, max_deg)
        normalized = flex_deg / max_deg

        if invert:
            return int((1.0 - normalized) * MAX_INSPIRE)
        else:
            return int(normalized * MAX_INSPIRE)

    def build_msg(self, pinky, ring, middle, index, thumb, spread):
        msg = InspireHandCtrl()
        msg.mode = 0b0001
        msg.angle_set = [pinky, ring, middle, index, thumb, spread]
        msg.speed_set = [MAX_INSPIRE] * 6
        msg.force_set = [0] * 6
        return msg

    # --------------------------------------------------
    # Callback
    # --------------------------------------------------
    def cb_left(self, msg):
        ctrl = self.map_manus(msg, is_right=False)
        self.pub_l.publish(ctrl)

    def cb_right(self, msg):
        ctrl = self.map_manus(msg, is_right=True)
        self.pub_r.publish(ctrl)

    # --------------------------------------------------
    # Mapping
    # --------------------------------------------------
    def map_manus(self, msg, is_right=False):

        ergo = {e.type: e.value for e in msg.ergonomics}

        # ---- Finger Flex ----
        index_flex = self.weighted_flex(
            ergo.get("IndexMCPStretch", 0.0),
            ergo.get("IndexPIPStretch", 0.0),
            ergo.get("IndexDIPStretch", 0.0)
        )

        middle_flex = self.weighted_flex(
            ergo.get("MiddleMCPStretch", 0.0),
            ergo.get("MiddlePIPStretch", 0.0),
            ergo.get("MiddleDIPStretch", 0.0)
        )

        ring_flex = self.weighted_flex(
            ergo.get("RingMCPStretch", 0.0),
            ergo.get("RingPIPStretch", 0.0),
            ergo.get("RingDIPStretch", 0.0)
        )

        pinky_flex = self.weighted_flex(
            ergo.get("PinkyMCPStretch", 0.0),
            ergo.get("PinkyPIPStretch", 0.0),
            ergo.get("PinkyDIPStretch", 0.0)
        )

        thumb_flex = self.weighted_flex(
            ergo.get("ThumbMCPStretch", 0.0),
            ergo.get("ThumbPIPStretch", 0.0),
            ergo.get("ThumbDIPStretch", 0.0)
        )

        # ---- Inspire 변환 ----
        index  = self.flex_to_inspire(index_flex, 95.0, invert=True)
        middle = self.flex_to_inspire(middle_flex, 95.0, invert=True)
        ring   = self.flex_to_inspire(ring_flex, 95.0, invert=True)
        pinky  = self.flex_to_inspire(pinky_flex, 95.0, invert=True)

        # Thumb Inversion
        thumb  = self.flex_to_inspire(thumb_flex, 35.0, invert=True)

        # ---- Spread ----
        spread_raw = ergo.get("ThumbMCPSpread", 0.0)
        
        spread_scaled = spread_raw * 1.5  

        spread_norm = (spread_scaled + 50.0) / 70.0
        spread_norm = self.clamp(spread_norm, 0.0, 1.0)

        spread_value = int(spread_norm * MAX_INSPIRE)

        spread_value = MAX_INSPIRE - spread_value

        return self.build_msg(
            pinky, ring, middle, index, thumb, spread_value
        )


def main():
    rclpy.init()
    node = ManusToInspire()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()