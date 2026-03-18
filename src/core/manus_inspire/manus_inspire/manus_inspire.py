import os
import yaml
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from manus_ros2_msgs.msg import ManusGlove
from inspire_hand_msgs.msg import InspireHandCtrl

MAX_INSPIRE = 1000
CALIB_DURATION = 4.0  # seconds per phase

FINGERS = ['index', 'middle', 'ring', 'pinky', 'thumb', 'spread']

ERGO_KEYS = {
    'index':  ('IndexMCPStretch',  'IndexPIPStretch',  'IndexDIPStretch'),
    'middle': ('MiddleMCPStretch', 'MiddlePIPStretch', 'MiddleDIPStretch'),
    'ring':   ('RingMCPStretch',   'RingPIPStretch',   'RingDIPStretch'),
    'pinky':  ('PinkyMCPStretch',  'PinkyPIPStretch',  'PinkyDIPStretch'),
    'thumb':  ('ThumbMCPStretch',  'ThumbPIPStretch',  'ThumbDIPStretch'),
}

# 캘리브레이션 없을 때 fallback 기본값
DEFAULT_CALIB = {
    'index':  {'min': 0.0, 'max': 75.0},
    'middle': {'min': 0.0, 'max': 75.0},
    'ring':   {'min': 0.0, 'max': 75.0},
    'pinky':  {'min': 0.0, 'max': 65.0},
    'thumb':  {'min': 0.0, 'max': 35.0},
    'spread': {'min': -30.0, 'max': 30.0},
}


class ManusInspire(Node):

    def __init__(self):
        super().__init__("manus_inspire")

        self.declare_parameter('calib_file',
                               os.path.expanduser('~/.ros/manus_inspire_calib.yaml'))
        self._calib_file = self.get_parameter('calib_file').get_parameter_value().string_value

        self._calib = self._default_calib()
        self._calib_mode = False
        self._calib_phase = 0
        self._calib_samples = {}
        self._calib_start = 0.0

        if os.path.exists(self._calib_file):
            self._load_calib()
            self.get_logger().info(f"Calibration loaded from {self._calib_file}")
        else:
            self.get_logger().warn("No calibration file found — starting calibration now.")
            self._start_calib()

        # Subscribe to both glove topics with the same callback — side is determined by msg.side
        self.sub_0 = self.create_subscription(ManusGlove, "/manus_glove_0", self.cb_glove, 10)
        self.sub_1 = self.create_subscription(ManusGlove, "/manus_glove_1", self.cb_glove, 10)

        self.pub_l = self.create_publisher(InspireHandCtrl, "/rt/inspire_hand/ctrl/l", 2)
        self.pub_r = self.create_publisher(InspireHandCtrl, "/rt/inspire_hand/ctrl/r", 2)

        self.srv_calib = self.create_service(
            Trigger, '~/calibrate', self._srv_calibrate)

        self.get_logger().info("Manus → Inspire Dual Hand Control")

    # --------------------------------------------------
    # 캘리브레이션
    # --------------------------------------------------
    def _default_calib(self):
        import copy
        return {hand: copy.deepcopy(DEFAULT_CALIB) for hand in ('left', 'right')}

    def _start_calib(self):
        self._calib_mode = True
        self._calib_phase = 0
        self._calib_samples = {
            hand: {f: [] for f in FINGERS}
            for hand in ('left', 'right')
        }
        self._calib_start = self.get_clock().now().nanoseconds * 1e-9
        self.get_logger().warn("=== CALIBRATION 1/2: Open hands fully and hold... ===")

    def _srv_calibrate(self, req, resp):
        self._start_calib()
        resp.success = True
        resp.message = "Calibration started — open hands for 4s, then close fists for 4s."
        return resp

    def _collect_sample(self, hand, ergo):
        s = self._calib_samples[hand]
        for fname, keys in ERGO_KEYS.items():
            val = self.weighted_flex(
                ergo.get(keys[0], 0.0),
                ergo.get(keys[1], 0.0),
                ergo.get(keys[2], 0.0),
            )
            s[fname].append(val)
        s['spread'].append(ergo.get('ThumbMCPSpread', 0.0))

    def _advance_calib(self):
        now = self.get_clock().now().nanoseconds * 1e-9
        if now - self._calib_start < CALIB_DURATION:
            return

        if self._calib_phase == 0:
            # open-hand → min
            for hand in ('left', 'right'):
                for f in FINGERS:
                    s = self._calib_samples[hand][f]
                    if s:
                        self._calib[hand][f]['min'] = float(min(s))
            # reset for phase 2
            self._calib_phase = 1
            self._calib_samples = {
                hand: {f: [] for f in FINGERS}
                for hand in ('left', 'right')
            }
            self._calib_start = now
            self.get_logger().warn("=== CALIBRATION 2/2: Close fists fully and hold... ===")

        elif self._calib_phase == 1:
            # fist → max
            for hand in ('left', 'right'):
                for f in FINGERS:
                    s = self._calib_samples[hand][f]
                    if s:
                        self._calib[hand][f]['max'] = float(max(s))
            self._calib_mode = False
            self._save_calib()
            self.get_logger().info("=== CALIBRATION COMPLETE ===")

    def _load_calib(self):
        with open(self._calib_file, 'r') as f:
            self._calib = yaml.safe_load(f)

    def _save_calib(self):
        os.makedirs(os.path.dirname(self._calib_file) or '.', exist_ok=True)
        with open(self._calib_file, 'w') as f:
            yaml.dump(self._calib, f, default_flow_style=False)
        self.get_logger().info(f"Calibration saved → {self._calib_file}")

    # --------------------------------------------------
    # 유틸
    # --------------------------------------------------
    def clamp(self, v, lo, hi):
        return max(lo, min(v, hi))

    def weighted_flex(self, mcp, pip, dip):
        return 0.25 * mcp + 0.55 * pip + 0.20 * dip

    def flex_to_inspire(self, flex, calib_min, calib_max, invert=True):
        rng = max(calib_max - calib_min, 1.0)
        normalized = self.clamp((flex - calib_min) / rng, 0.0, 1.0)
        if invert:
            return int((1.0 - normalized) * MAX_INSPIRE)
        return int(normalized * MAX_INSPIRE)

    def build_msg(self, pinky, ring, middle, index, thumb, spread):
        msg = InspireHandCtrl()
        msg.mode = 0b0001
        msg.angle_set = [pinky, ring, middle, index, thumb, spread]
        msg.speed_set  = [MAX_INSPIRE] * 6
        msg.force_set  = [0] * 6
        return msg

    # --------------------------------------------------
    # Callback
    # --------------------------------------------------
    def cb_glove(self, msg):
        # Determine hand from msg.side ("Left" / "Right") — not from topic number
        if msg.side == 'Left':
            hand = 'left'
        elif msg.side == 'Right':
            hand = 'right'
        else:
            return  # unknown side, skip

        ergo = {e.type: e.value for e in msg.ergonomics}
        if self._calib_mode:
            self._collect_sample(hand, ergo)
            self._advance_calib()
            return

        ctrl = self.map_manus(ergo, hand=hand)
        if hand == 'left':
            self.pub_l.publish(ctrl)
        else:
            self.pub_r.publish(ctrl)

    # --------------------------------------------------
    # Mapping
    # --------------------------------------------------
    def map_manus(self, ergo, hand='right'):
        c = self._calib[hand]
        is_right = (hand == 'right')

        index_flex  = self.weighted_flex(ergo.get("IndexMCPStretch",  0.0), ergo.get("IndexPIPStretch",  0.0), ergo.get("IndexDIPStretch",  0.0))
        middle_flex = self.weighted_flex(ergo.get("MiddleMCPStretch", 0.0), ergo.get("MiddlePIPStretch", 0.0), ergo.get("MiddleDIPStretch", 0.0))
        ring_flex   = self.weighted_flex(ergo.get("RingMCPStretch",   0.0), ergo.get("RingPIPStretch",   0.0), ergo.get("RingDIPStretch",   0.0))
        pinky_flex  = self.weighted_flex(ergo.get("PinkyMCPStretch",  0.0), ergo.get("PinkyPIPStretch",  0.0), ergo.get("PinkyDIPStretch",  0.0))
        thumb_flex  = self.weighted_flex(ergo.get("ThumbMCPStretch",  0.0), ergo.get("ThumbPIPStretch",  0.0), ergo.get("ThumbDIPStretch",  0.0))

        index  = self.flex_to_inspire(index_flex,  c['index']['min'],  c['index']['max'],  invert=True)
        middle = self.flex_to_inspire(middle_flex, c['middle']['min'], c['middle']['max'], invert=True)
        ring   = self.flex_to_inspire(ring_flex,   c['ring']['min'],   c['ring']['max'],   invert=True)
        pinky  = self.flex_to_inspire(pinky_flex,  c['pinky']['min'],  c['pinky']['max'],  invert=True)
        thumb  = self.flex_to_inspire(thumb_flex,  c['thumb']['min'],  c['thumb']['max'],  invert=True)

        spread_raw  = ergo.get("ThumbMCPSpread", 0.0)
        spread_rng  = max(c['spread']['max'] - c['spread']['min'], 1.0)
        spread_norm = self.clamp((spread_raw - c['spread']['min']) / spread_rng, 0.0, 1.0)
        spread_value = int(spread_norm * MAX_INSPIRE)
        if not is_right:
            spread_value = MAX_INSPIRE - spread_value

        return self.build_msg(pinky, ring, middle, index, thumb, spread_value)


def main():
    rclpy.init()
    node = ManusInspire()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
