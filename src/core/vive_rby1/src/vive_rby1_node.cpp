// vive_rby1_node.cpp — Vive Tracker 입력 → SDK Cartesian/Impedance 명령 변환 (C++ 프로덕션).
// 책임:
//   1) /teleop/tracker/{left,right,body} 트래커 입력 smoothing + 5-state 녹화 상태머신
//   2) engage() 시점에 ref(트래커 base) + ee_*_0_(로봇 EE FK) 캡처 → delta 계산용 기준
//   3) onTimer() 100Hz 루프: 트래커 delta → 로봇 좌표(v2r_R_) → ee target → /rby1/cmd/pose (tf2_msgs/TFMessage)
//   4) doTeleopStart/Stop (detached thread): SetControlMode → MoveToJointPosition → SetNullspaceJointRef → SetStream
//   5) 3개 페달(A=clutch, B=discard, C=episode toggle) + scm_recording core 서비스 호출
//   6) (옵션) body tracker → link_torso_5 (use_torso_, 2026-05-22 추가; runtime toggle)
// EE/torso FK 기준은 hw-core가 발행하는 /rby1/state/pose (tf2_msgs/TFMessage,
// child_frame_id ee_right/ee_left/link_torso_5)를 ROS2로 구독해 얻는다 — 외부 PC에서도
// 동작(로봇 gRPC 불필요), 소스가 hw-core FK로 통일되어 Z 오프셋 보정 불필요.
// 패키지 가이드: DEVELOPER.ko.md
#include <algorithm>
#include <array>
#include <chrono>
#include <cctype>
#include <cmath>
#include <deque>
#include <functional>
#include <future>
#include <limits>
#include <memory>
#include <mutex>
#include <numeric>
#include <optional>
#include <sstream>
#include <string>
#include <thread>
#include <unordered_map>
#include <utility>
#include <vector>

#include "Eigen/Core"
#include "Eigen/Geometry"

#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_msgs/msg/tf_message.hpp"
#include "rby1_core_msgs/srv/set_control_mode.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rby1_core_msgs/srv/move_to_joint_position.hpp"
#include "rby1_core_msgs/srv/set_nullspace_joint_ref.hpp"
#include "rby1_core_msgs/srv/set_stream.hpp"
#include "scm_recording_msgs/srv/end_recording.hpp"
#include "scm_recording_msgs/srv/set_tele_op_pose.hpp"
#include "scm_recording_msgs/srv/start_recording.hpp"
#include "scm_recording_msgs/srv/toggle_pause.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/trigger.hpp"

namespace {

using namespace std::chrono_literals;

// Lightweight rigid transform — drop-in for the subset of SE3 this node ever used
// (construction from (R,t) plus .rotation()/.translation() accessors). FK references
// come from hw-core's /rby1/state/pose over ROS2, so no SDK/pinocchio/URDF is needed.
struct SE3 {
  Eigen::Matrix3d R{Eigen::Matrix3d::Identity()};
  Eigen::Vector3d t{Eigen::Vector3d::Zero()};
  SE3() = default;
  SE3(const Eigen::Matrix3d & rot, const Eigen::Vector3d & trans) : R(rot), t(trans) {}
  const Eigen::Matrix3d & rotation() const { return R; }
  const Eigen::Vector3d & translation() const { return t; }
  Eigen::Vector3d & translation() { return t; }
};

constexpr char kRecIdle[] = "IDLE";
constexpr char kRecArming[] = "ARMING";
constexpr char kRecReady[] = "READY";
constexpr char kRecRecording[] = "RECORDING";
constexpr char kRecPaused[] = "PAUSED";
constexpr double kPi = 3.14159265358979323846;

SE3 poseStampedToSe3(const geometry_msgs::msg::PoseStamped & msg) {
  const auto & p = msg.pose.position;
  const auto & q = msg.pose.orientation;
  Eigen::Quaterniond quat(q.w, q.x, q.y, q.z);
  quat.normalize();
  return SE3(quat.toRotationMatrix(), Eigen::Vector3d(p.x, p.y, p.z));
}

SE3 transformToSe3(const geometry_msgs::msg::Transform & tf) {
  const auto & t = tf.translation;
  const auto & r = tf.rotation;
  Eigen::Quaterniond quat(r.w, r.x, r.y, r.z);
  quat.normalize();
  return SE3(quat.toRotationMatrix(), Eigen::Vector3d(t.x, t.y, t.z));
}

geometry_msgs::msg::Pose se3ToPose(const SE3 & se3) {
  geometry_msgs::msg::Pose pose;
  pose.position.x = se3.translation().x();
  pose.position.y = se3.translation().y();
  pose.position.z = se3.translation().z();
  Eigen::Quaterniond quat(se3.rotation());
  quat.normalize();
  pose.orientation.x = quat.x();
  pose.orientation.y = quat.y();
  pose.orientation.z = quat.z();
  pose.orientation.w = quat.w();
  return pose;
}

geometry_msgs::msg::Transform poseToTransform(const geometry_msgs::msg::Pose & p) {
  geometry_msgs::msg::Transform t;
  t.translation.x = p.position.x;
  t.translation.y = p.position.y;
  t.translation.z = p.position.z;
  t.rotation      = p.orientation;
  return t;
}

geometry_msgs::msg::TransformStamped makeTransformStamped(
    const std::string & child_frame_id,
    const geometry_msgs::msg::Transform & tf,
    const rclcpp::Time & stamp) {
  geometry_msgs::msg::TransformStamped ts;
  ts.header.stamp    = stamp;
  ts.header.frame_id = "base";
  ts.child_frame_id  = child_frame_id;
  ts.transform       = tf;
  return ts;
}

geometry_msgs::msg::TransformStamped se3ToTransformStamped(
    const std::string & child_frame_id,
    const SE3 & se3,
    const rclcpp::Time & stamp) {
  return makeTransformStamped(child_frame_id, poseToTransform(se3ToPose(se3)), stamp);
}

bool isFinite(const SE3 & se3) {
  return se3.translation().allFinite() && se3.rotation().allFinite();
}

class ViveRby1Node : public rclcpp::Node {
 public:
  using StartRecording = scm_recording_msgs::srv::StartRecording;
  using EndRecording = scm_recording_msgs::srv::EndRecording;
  using TogglePause = scm_recording_msgs::srv::TogglePause;
  using SetControlMode = rby1_core_msgs::srv::SetControlMode;
  using MoveToJointPosition = rby1_core_msgs::srv::MoveToJointPosition;
  using SetStream = rby1_core_msgs::srv::SetStream;
  using SetTeleOpPose = scm_recording_msgs::srv::SetTeleOpPose;
  using SetNullspaceJointRef = rby1_core_msgs::srv::SetNullspaceJointRef;

  ViveRby1Node()
  : Node("vive_rby1_node"),
    v2r_R_((Eigen::Matrix3d() << 0., 1., 0., -1., 0., 0., 0., 0., 1.).finished()) {
    declare_parameter("topic_state_pose", "/rby1/state/pose");
    declare_parameter("topic_tracker_left",  "/teleop/tracker/left");
    declare_parameter("topic_tracker_right", "/teleop/tracker/right");
    declare_parameter("topic_tracker_body",  "/teleop/tracker/body");
    declare_parameter("topic_pedal", "/teleop/pedal");
    declare_parameter("pos_scale", 1.0);
    declare_parameter("torso_pos_scale", 1.0);
    declare_parameter("use_torso", false);
    declare_parameter("publish_rate", 20.0);
    declare_parameter("sdk_max_delta_pos", 0.03);
    declare_parameter("sdk_max_delta_rot_deg", 20.0);
    declare_parameter("pedal_engage_index", 0);
    declare_parameter("pedal_discard_index", 1);
    declare_parameter("pedal_episode_index", 2);
    declare_parameter("tracker_smooth_alpha", 0.9);
    declare_parameter("cooldown_sec", 0.5);

    const auto topic_state_pose = get_parameter("topic_state_pose").as_string();
    const auto topic_l = get_parameter("topic_tracker_left").as_string();
    const auto topic_r = get_parameter("topic_tracker_right").as_string();
    const auto topic_b = get_parameter("topic_tracker_body").as_string();
    const auto topic_p = get_parameter("topic_pedal").as_string();

    pos_scale_ = get_parameter("pos_scale").as_double();
    torso_pos_scale_ = get_parameter("torso_pos_scale").as_double();
    use_torso_ = get_parameter("use_torso").as_bool();
    publish_rate_ = get_parameter("publish_rate").as_double();
    sdk_max_delta_pos_ = get_parameter("sdk_max_delta_pos").as_double();
    sdk_max_delta_rot_ = get_parameter("sdk_max_delta_rot_deg").as_double() * kPi / 180.0;
    pedal_engage_idx_  = static_cast<size_t>(get_parameter("pedal_engage_index").as_int());
    pedal_discard_idx_ = static_cast<size_t>(get_parameter("pedal_discard_index").as_int());
    pedal_episode_idx_ = static_cast<size_t>(get_parameter("pedal_episode_index").as_int());
    tracker_smooth_alpha_ = get_parameter("tracker_smooth_alpha").as_double();
    cooldown_sec_ = get_parameter("cooldown_sec").as_double();

    auto stream_qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort();
    // EE/torso FK references come from hw-core over ROS2 (/rby1/state/pose, TFMessage
    // keyed by child_frame_id). No SDK/gRPC/URDF — works from an external PC.
    sub_state_pose_ = create_subscription<tf2_msgs::msg::TFMessage>(
      topic_state_pose, stream_qos,
      std::bind(&ViveRby1Node::onStatePose, this, std::placeholders::_1));
    sub_tracker_l_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      topic_l, stream_qos, std::bind(&ViveRby1Node::onTrackerLeft, this, std::placeholders::_1));
    sub_tracker_r_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      topic_r, stream_qos, std::bind(&ViveRby1Node::onTrackerRight, this, std::placeholders::_1));
    sub_tracker_b_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      topic_b, stream_qos, std::bind(&ViveRby1Node::onTrackerBody, this, std::placeholders::_1));
    sub_pedal_ = create_subscription<sensor_msgs::msg::Joy>(
      topic_p, rclcpp::QoS(rclcpp::KeepLast(5)).reliable(),
      std::bind(&ViveRby1Node::onPedal, this, std::placeholders::_1));
    sub_task_id_ = create_subscription<std_msgs::msg::Int32>(
      "/teleop/task_id", 10, [this](const std_msgs::msg::Int32::SharedPtr msg) {
        rec_task_id_ = msg->data;
      });
    sub_mirror_mode_ = create_subscription<std_msgs::msg::String>(
      "/teleop/mirror_mode", 10, std::bind(&ViveRby1Node::onMirrorMode, this, std::placeholders::_1));
    srv_set_use_torso_ = create_service<std_srvs::srv::SetBool>(
      "/vive_rby1/set_use_torso",
      std::bind(&ViveRby1Node::onSetUseTorso, this, std::placeholders::_1, std::placeholders::_2));

    // /rby1/cmd/pose carries tf2_msgs/TFMessage used as plain data (NOT a TF
    // broadcast). Each TransformStamped names a target link via child_frame_id:
    // "ee_right", "ee_left", and optionally "link_torso_5".
    pub_pose_cmd_ = create_publisher<tf2_msgs::msg::TFMessage>("/rby1/cmd/pose", stream_qos);
    // warmup/cooldown hold is computed from the latest /rby1/state/pose (publishEeHold).
    pub_rec_state_ = create_publisher<std_msgs::msg::String>("/teleop/rec_state", 10);
    pub_rec_episode_ = create_publisher<std_msgs::msg::Int32>("/teleop/rec_episode", 10);
    pub_tracker_status_ = create_publisher<std_msgs::msg::String>("/teleop/tracker_status", 10);
    pub_clutch_state_   = create_publisher<std_msgs::msg::String>("/teleop/clutch_state",   10);

    cli_start_rec_ = create_client<StartRecording>("/scm_recording/start");
    cli_end_rec_ = create_client<EndRecording>("/scm_recording/end");
    cli_toggle_pause_ = create_client<TogglePause>("/scm_recording/toggle_pause");
    cli_set_mode_ = create_client<SetControlMode>("/rby1/ctrl/mode");
    cli_stream_ = create_client<SetStream>("/rby1/stream");
    cli_move_joint_ = create_client<MoveToJointPosition>("/rby1/move_to_joint_position");
    cli_nullspace_joint_ref_ = create_client<SetNullspaceJointRef>("/rby1/set_nullspace_joint_ref");

    srv_teleop_start_ = create_service<std_srvs::srv::Trigger>(
      "/vive_rby1/teleop_start",
      [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
             std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
        on_teleop_start(res); });
    srv_teleop_stop_ = create_service<std_srvs::srv::Trigger>(
      "/vive_rby1/teleop_stop",
      [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
             std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
        on_teleop_stop(res); });
    srv_toggle_clutch_ = create_service<std_srvs::srv::Trigger>(
      "/vive_rby1/toggle_clutch",
      [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
             std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
        on_toggle_clutch(res); });
    srv_set_teleop_pose_ = create_service<SetTeleOpPose>(
      "/vive_rby1/set_teleop_pose",
      [this](const std::shared_ptr<SetTeleOpPose::Request> req,
             std::shared_ptr<SetTeleOpPose::Response> res) {
        on_set_teleop_pose(req, res); });
    srv_toggle_episode_ = create_service<std_srvs::srv::Trigger>(
      "/vive_rby1/toggle_episode",
      [this](
        const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        toggleEpisode();
        response->success = true;
        response->message = "OK";
      });

    timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / std::max(1.0, publish_rate_)),
      std::bind(&ViveRby1Node::onTimer, this));

    RCLCPP_INFO(get_logger(), "[vive_rby1] Ready -- press pedal 0 to engage");
  }

 private:
  // /rby1/state/pose (tf2_msgs/TFMessage) — hw-core FK, keyed by child_frame_id.
  // Cache the latest ee_right/ee_left/link_torso_5 for engage refs and hold publishing.
  void onStatePose(const tf2_msgs::msg::TFMessage::SharedPtr msg) {
    for (const auto & ts : msg->transforms) {
      if      (ts.child_frame_id == "ee_right")     last_ee_r_ = transformToSe3(ts.transform);
      else if (ts.child_frame_id == "ee_left")      last_ee_l_ = transformToSe3(ts.transform);
      else if (ts.child_frame_id == "link_torso_5") last_torso_ = transformToSe3(ts.transform);
    }
  }

  struct TrackerState {
    geometry_msgs::msg::PoseStamped::SharedPtr raw;
    std::optional<SE3> smoothed;
    std::deque<Eigen::Vector3d> buf;
    double stamp_sec{0.0};
  };

  void onTrackerLeft(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    tracker_l_.raw = msg;
    tracker_l_.stamp_sec = nowSec();
    tracker_l_.buf.push_back(Eigen::Vector3d(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z));
    while (tracker_l_.buf.size() > 20) {
      tracker_l_.buf.pop_front();
    }
    tracker_l_.smoothed = smoothTracker(tracker_l_.smoothed, *msg);
  }

  void onTrackerRight(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    tracker_r_.raw = msg;
    tracker_r_.stamp_sec = nowSec();
    tracker_r_.buf.push_back(Eigen::Vector3d(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z));
    while (tracker_r_.buf.size() > 20) {
      tracker_r_.buf.pop_front();
    }
    tracker_r_.smoothed = smoothTracker(tracker_r_.smoothed, *msg);
  }

  void onTrackerBody(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    tracker_b_.raw = msg;
    tracker_b_.stamp_sec = nowSec();
    tracker_b_.buf.push_back(Eigen::Vector3d(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z));
    while (tracker_b_.buf.size() > 20) {
      tracker_b_.buf.pop_front();
    }
    tracker_b_.smoothed = smoothTracker(tracker_b_.smoothed, *msg);
    // Body tracker came online after engage: capture reference now so torso
    // joins smoothly instead of staying disabled until the next re-engage.
    if (engaged_ && use_torso_ && !ref_body_ && tracker_b_.smoothed && last_torso_) {
      ref_body_ = tracker_b_.smoothed;
      torso5_0_ = last_torso_;
    }
  }

  void onMirrorMode(const std_msgs::msg::String::SharedPtr msg) {
    mirror_mode_ = (msg->data == "mirror");
    RCLCPP_INFO(get_logger(), "[vive_rby1] mirror mode -- %s", mirror_mode_ ? "true" : "false");
    if (engaged_) {
      ref_l_ = tracker_l_.smoothed;
      ref_r_ = tracker_r_.smoothed;
      ee_l_0_ = last_ee_l_;
      ee_r_0_ = last_ee_r_;
    }
  }

  // onSetUseTorso — body tracker → link_torso_5 런타임 on/off 토글 (2026-05-22).
  // OFF 전환: ref_body_/torso5_0_ 리셋 → /rby1/cmd/pose에서 link_torso_5 항목 사라짐 → hw-core가
  //          마지막 torso 포즈에서 freeze (CartesianImpedance 빌더가 seeded T_torso로 hold).
  // ON 전환: engage 중이고 트래커 활성이면 현재 torso 포즈 기준으로 재캡처 → 부드러운 합류.
  void onSetUseTorso(
      const std::shared_ptr<std_srvs::srv::SetBool::Request> req,
      std::shared_ptr<std_srvs::srv::SetBool::Response> res)
  {
    if (req->data == use_torso_) {
      res->success = true;
      res->message = use_torso_ ? "already enabled" : "already disabled";
      return;
    }
    use_torso_ = req->data;
    RCLCPP_INFO(get_logger(), "[vive_rby1] use_torso -- %s", use_torso_ ? "true" : "false");
    if (!use_torso_) {
      // Stop sending link_torso_5: hw-core holds the torso at its last pose.
      ref_body_.reset();
      torso5_0_.reset();
    } else if (engaged_ && tracker_b_.smoothed && last_torso_) {
      // Re-enabled mid-stream: recapture reference from the current torso pose.
      ref_body_ = tracker_b_.smoothed;
      torso5_0_ = last_torso_;
    }
    res->success = true;
    res->message = use_torso_ ? "enabled" : "disabled";
  }

  void publishClutchState() {
    std_msgs::msg::String msg;
    msg.data = engaged_ ? "ENGAGED" : "DISENGAGED";
    pub_clutch_state_->publish(msg);
  }

  void onPedal(const sensor_msgs::msg::Joy::SharedPtr msg) {
    const bool engage_pressed =
      pedal_engage_idx_ < msg->buttons.size() && static_cast<bool>(msg->buttons[pedal_engage_idx_]);
    if (engage_pressed && !pedal_engage_prev_) {
      if (!teleop_active_) {
        RCLCPP_WARN(get_logger(), "Cannot engage -- teleop not active");
      } else if (engaged_) {
        disengage();
      } else if (tracker_l_.raw && tracker_r_.raw) {
        engage();
      } else {
        RCLCPP_WARN(get_logger(), "Cannot engage -- Vive trackers not ready");
      }
    }
    pedal_engage_prev_ = engage_pressed;

    const bool discard_pressed =
      pedal_discard_idx_ < msg->buttons.size() && static_cast<bool>(msg->buttons[pedal_discard_idx_]);
    if (discard_pressed && !pedal_discard_prev_) {
      discardEpisode();
    }
    pedal_discard_prev_ = discard_pressed;

    const bool episode_pressed =
      pedal_episode_idx_ < msg->buttons.size() && static_cast<bool>(msg->buttons[pedal_episode_idx_]);
    if (episode_pressed && !pedal_episode_prev_) {
      toggleEpisode();
    }
    pedal_episode_prev_ = episode_pressed;
  }

  // engage — 페달 A 또는 GUI 클러치 토글 시 호출.
  // 트래커 기준점(ref_*)과 로봇 EE 기준점(ee_*_0_/sdk_ee_*_0_)을 캡처해 이후 delta 계산의 origin으로 사용.
  // EE 기준은 hw-core가 발행한 /rby1/state/pose(FK)에서 옴 — 소스가 hw-core와 동일해 Z 오프셋 보정 불필요.
  // engage 시점에 body tracker가 미가용이면 onTrackerBody에서 늦은 재캡처(2026-05-22)됨.
  void engage() {
    if (!tracker_l_.smoothed || !tracker_r_.smoothed) {
      RCLCPP_WARN(get_logger(), "Trackers not ready -- ignoring engage");
      return;
    }
    if (!last_ee_l_ || !last_ee_r_) {
      RCLCPP_WARN(get_logger(), "Cannot engage -- EE pose (/rby1/state/pose) not received yet");
      return;
    }
    ref_l_ = tracker_l_.smoothed;
    ref_r_ = tracker_r_.smoothed;
    ee_l_0_ = last_ee_l_;
    ee_r_0_ = last_ee_r_;
    sdk_ee_l_0_ = last_ee_l_;
    sdk_ee_r_0_ = last_ee_r_;
    if (use_torso_ && tracker_b_.smoothed && last_torso_) {
      ref_body_   = tracker_b_.smoothed;
      torso5_0_   = last_torso_;
    }
    sdk_prev_l_.reset();
    sdk_prev_r_.reset();
    cooldown_ticks_ = 0;  // 진행 중인 disengage hold를 즉시 종료, 재engage가 우선
    engaged_ = true;
    publishClutchState();
    RCLCPP_INFO(get_logger(), "Clutch ENGAGED");
    if (rec_state_ == kRecReady || rec_state_ == kRecPaused) {
      callTogglePause();
    }
  }

  void disengage() {
    engaged_ = false;
    sdk_prev_l_.reset();
    sdk_prev_r_.reset();
    ref_body_.reset();
    torso5_0_.reset();
    // 펌웨어가 마지막 트래커 delta가 반영된 캐시 target까지 velocity-limited로
    // 추종하며 생기는 잔여 모션을 막기 위해, 잠시 현재 FK pose를 hold-publish해
    // hw-core의 캐시 target을 실제 EE 위치로 snap시킨다(대칭: warmup_ticks_).
    cooldown_ticks_ = static_cast<int>(std::round(cooldown_sec_ * publish_rate_));
    publishClutchState();
    RCLCPP_INFO(get_logger(), "Clutch DISENGAGED (cooldown=%d ticks)", cooldown_ticks_);
    if (rec_state_ == kRecRecording) {
      callTogglePause();
    }
  }

  void discardEpisode() {
    if (rec_state_ == kRecIdle) {
      RCLCPP_WARN(get_logger(), "discardEpisode: no active episode");
      return;
    }
    if (!cli_end_rec_->service_is_ready()) {
      RCLCPP_WARN(get_logger(), "EndRecording service not available -- force resetting state");
      rec_state_ = kRecIdle;
      rec_episode_ = -1;
      engaged_ = false;
      publishClutchState();
      publishRecState();
      return;
    }
    if (engaged_) { disengage(); }
    auto req = std::make_shared<EndRecording::Request>();
    req->discard = true;
    cli_end_rec_->async_send_request(
      req, [this](rclcpp::Client<EndRecording>::SharedFuture future) {
        const auto result = future.get();
        if (result->result) {
          rec_state_ = kRecIdle;
          rec_episode_ = -1;
          engaged_ = false;
          RCLCPP_INFO(get_logger(), "[vive_rby1] Episode DISCARDED -- teleop_stop");
          std::weak_ptr<ViveRby1Node> weak = std::static_pointer_cast<ViveRby1Node>(shared_from_this());
          std::thread([weak]() { if (auto self = weak.lock()) self->doTeleopStop(); }).detach();
        } else {
          RCLCPP_ERROR(get_logger(), "discardEpisode failed: %s", result->message.c_str());
        }
        publishRecState();
      });
  }

  void toggleEpisode() {
    if (rec_state_ == kRecIdle) {
      if (!cli_start_rec_->service_is_ready()) {
        RCLCPP_WARN(get_logger(), "StartRecording service not available");
        return;
      }
      auto req = std::make_shared<StartRecording::Request>();
      req->task_id = rec_task_id_;
      cli_start_rec_->async_send_request(
        req, [this](rclcpp::Client<StartRecording>::SharedFuture future) {
          const auto result = future.get();
          if (result->result) {
            rec_state_ = kRecArming;
            rec_episode_ = result->episode_id;
            RCLCPP_INFO(
              get_logger(), "[vive_rby1] ARMING -- task %d ep %d", result->task_id, result->episode_id);
            std::weak_ptr<ViveRby1Node> weak = std::static_pointer_cast<ViveRby1Node>(shared_from_this());
            std::thread([weak]() { if (auto self = weak.lock()) self->doTeleopStart(); }).detach();
          } else {
            RCLCPP_ERROR(get_logger(), "StartRecording failed: %s", result->message.c_str());
          }
          publishRecState();
        });
    } else if (rec_state_ == kRecRecording) {
      RCLCPP_WARN(get_logger(), "EndRecording blocked -- disengage arm first (must be PAUSED)");
    } else {
      if (!cli_end_rec_->service_is_ready()) {
        RCLCPP_WARN(get_logger(), "EndRecording service not available");
        return;
      }
      auto req = std::make_shared<EndRecording::Request>();
      cli_end_rec_->async_send_request(
        req, [this](rclcpp::Client<EndRecording>::SharedFuture future) {
          const auto result = future.get();
          if (result->result) {
            rec_state_ = kRecIdle;
            rec_episode_ = -1;
            engaged_ = false;
            RCLCPP_INFO(get_logger(), "[vive_rby1] Recording ENDED -- teleop_stop");
            std::weak_ptr<ViveRby1Node> weak = std::static_pointer_cast<ViveRby1Node>(shared_from_this());
            std::thread([weak]() { if (auto self = weak.lock()) self->doTeleopStop(); }).detach();
          } else {
            RCLCPP_ERROR(get_logger(), "EndRecording failed: %s", result->message.c_str());
          }
          publishRecState();
        });
    }
  }

  void callTogglePause() {
    auto req = std::make_shared<TogglePause::Request>();
    cli_toggle_pause_->async_send_request(
      req, [this](rclcpp::Client<TogglePause>::SharedFuture future) {
        try {
          const auto result = future.get();
          if (result->result) {
            rec_state_ = result->paused ? kRecPaused : kRecRecording;
            RCLCPP_INFO(get_logger(), "[vive_rby1] %s", rec_state_.c_str());
          } else {
            RCLCPP_ERROR(
              get_logger(), "TogglePause failed -- result=%d paused=%d msg=%s", result->result,
              result->paused, result->message.c_str());
          }
        } catch (const std::exception & e) {
          RCLCPP_ERROR(get_logger(), "TogglePause exception: %s", e.what());
        }
        publishRecState();
      });
  }

  // ── New service handlers ──────────────────────────────────────────────────

  void on_teleop_start(std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
    res->success = true;
    res->message = "teleop start initiated";
    std::weak_ptr<ViveRby1Node> weak = std::static_pointer_cast<ViveRby1Node>(shared_from_this());
    std::thread([weak]() { if (auto self = weak.lock()) self->doTeleopStart(); }).detach();
  }

  void on_teleop_stop(std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
    res->success = true;
    res->message = "teleop stop initiated";
    std::weak_ptr<ViveRby1Node> weak = std::static_pointer_cast<ViveRby1Node>(shared_from_this());
    std::thread([weak]() { if (auto self = weak.lock()) self->doTeleopStop(); }).detach();
  }

  void on_toggle_clutch(std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
    if (!teleop_active_) {
      res->success = false; res->message = "teleop not active"; return;
    }
    if (engaged_) {
      disengage();
    } else if (tracker_l_.raw && tracker_r_.raw) {
      engage();
    } else {
      res->success = false; res->message = "trackers not ready"; return;
    }
    res->success = true; res->message = "ok";
  }

  void on_set_teleop_pose(
      const std::shared_ptr<SetTeleOpPose::Request> req,
      std::shared_ptr<SetTeleOpPose::Response> res) {
    teleop_pose_ = req->pose;
    res->success = true;
    res->message = "teleop pose updated";
    RCLCPP_INFO(get_logger(), "[vive_rby1] teleop_pose updated (%zu joints)", req->pose.name.size());
    // Propagate arm joints to hw-core nullspace reference so IK stays near the new teleop pose.
    if (cli_nullspace_joint_ref_->service_is_ready()) {
      auto nr = std::make_shared<SetNullspaceJointRef::Request>();
      nr->target = req->pose;
      cli_nullspace_joint_ref_->async_send_request(nr);
    }
  }

  // ── Teleop start/stop helpers (run in detached threads) ──────────────────

  void doTeleopStart() {
    using namespace std::chrono_literals;

    // Step 1: Set cartesian impedance mode
    auto r1 = std::make_shared<SetControlMode::Request>();
    r1->source = "cartesian"; r1->control = "impedance";
    auto f1 = cli_set_mode_->async_send_request(r1);
    if (f1.wait_for(5s) != std::future_status::ready) {
      RCLCPP_ERROR(get_logger(), "[vive_rby1] doTeleopStart: SetControlMode timeout");
      return;
    }
    RCLCPP_INFO(get_logger(), "[vive_rby1] doTeleopStart: mode set to CartesianImpedance");

    // Step 2: Move to teleop pose
    auto r2 = std::make_shared<MoveToJointPosition::Request>();
    r2->target = teleop_pose_;
    r2->min_time = 0.0;  // let on_move_to_joint_position use proportional time (floor 1.5 s)
    auto f2 = cli_move_joint_->async_send_request(r2);
    if (f2.wait_for(30s) != std::future_status::ready) {
      RCLCPP_ERROR(get_logger(), "[vive_rby1] doTeleopStart: MoveToJointPosition timeout");
      return;
    }
    if (!f2.get()->success) {
      RCLCPP_ERROR(get_logger(), "[vive_rby1] doTeleopStart: MoveToJointPosition failed");
      if (rec_state_ == kRecArming) {
        rec_state_ = kRecIdle; rec_episode_ = -1; publishRecState();
      }
      return;
    }
    RCLCPP_INFO(get_logger(), "[vive_rby1] doTeleopStart: at teleop pose, starting stream");

    // 2026-05-22 추가: GUI 드롭다운 미조작 시에도 teleop_pose_가 hw-core의 nullspace_ref와
    // 일치하도록 stream 시작 직전에 재전송. 첫 CartesianImpedance tick부터 올바른 nullspace 적용.
    // Re-assert the nullspace reference so it matches the pose we just moved to,
    // even if the GUI never pushed it (or the service wasn't ready at selection
    // time). Sent before the stream so the first CartesianImpedance tick uses it.
    if (cli_nullspace_joint_ref_->service_is_ready()) {
      auto rn = std::make_shared<SetNullspaceJointRef::Request>();
      rn->target = teleop_pose_;
      cli_nullspace_joint_ref_->async_send_request(rn);
      RCLCPP_INFO(get_logger(), "[vive_rby1] doTeleopStart: nullspace ref set to teleop pose");
    } else {
      RCLCPP_WARN(get_logger(), "[vive_rby1] doTeleopStart: nullspace service not ready — skipped");
    }

    // Step 3: Start stream
    warmup_ticks_ = static_cast<int>(publish_rate_);
    auto r3 = std::make_shared<SetStream::Request>();
    r3->enable = true;
    auto f3 = cli_stream_->async_send_request(r3);
    if (f3.wait_for(10s) != std::future_status::ready || !f3.get()->success) {
      RCLCPP_ERROR(get_logger(), "[vive_rby1] doTeleopStart: SetStream failed");
      warmup_ticks_ = 0;
      if (rec_state_ == kRecArming) {
        rec_state_ = kRecIdle; rec_episode_ = -1; publishRecState();
      }
      return;
    }
    teleop_active_ = true;
    sdk_prev_l_.reset(); sdk_prev_r_.reset();
    if (rec_state_ == kRecArming) {
      warmup_ticks_ = 0;
      rec_state_ = kRecReady;
      publishRecState();
      RCLCPP_INFO(get_logger(), "[vive_rby1] Stream open -- READY");
    }
    RCLCPP_INFO(get_logger(), "[vive_rby1] doTeleopStart: complete");
  }

  void doTeleopStop() {
    using namespace std::chrono_literals;
    teleop_active_ = false;
    engaged_ = false;
    publishClutchState();

    auto r1 = std::make_shared<SetStream::Request>();
    r1->enable = false;
    auto f1 = cli_stream_->async_send_request(r1);
    if (f1.wait_for(5s) != std::future_status::ready) {
      RCLCPP_ERROR(get_logger(), "[vive_rby1] doTeleopStop: SetStream(false) timeout");
    }

    auto r2 = std::make_shared<MoveToJointPosition::Request>();
    r2->target = teleop_pose_;
    r2->min_time = 0.0;  // proportional time, floor 1.5 s
    auto f2 = cli_move_joint_->async_send_request(r2);
    if (f2.wait_for(30s) != std::future_status::ready) {
      RCLCPP_ERROR(get_logger(), "[vive_rby1] doTeleopStop: MoveToJointPosition timeout");
      return;
    }
    if (!f2.get()->success) {
      RCLCPP_ERROR(get_logger(), "[vive_rby1] doTeleopStop: MoveToJointPosition failed");
    }
    RCLCPP_INFO(get_logger(), "[vive_rby1] doTeleopStop: complete");
  }

  void publishRecState() {
    std_msgs::msg::String state_msg;
    state_msg.data = rec_state_;
    pub_rec_state_->publish(state_msg);

    std_msgs::msg::Int32 episode_msg;
    episode_msg.data = rec_episode_;
    pub_rec_episode_->publish(episode_msg);
  }

  std::optional<SE3> limitSdkTarget(
    const std::optional<SE3> & prev, const SE3 & target,
    const char * arm_name) {
    if (!isFinite(target)) {
      RCLCPP_WARN(get_logger(), "[vive_rby1] dropping non-finite SDK target for %s", arm_name);
      return prev;
    }
    if (!prev) {
      return target;
    }

    Eigen::Vector3d pos = target.translation();
    const Eigen::Vector3d delta = pos - prev->translation();
    const double delta_norm = delta.norm();
    if (delta_norm > sdk_max_delta_pos_ && sdk_max_delta_pos_ > 0.0) {
      pos = prev->translation() + delta / delta_norm * sdk_max_delta_pos_;
    }

    Eigen::Quaterniond q_prev(prev->rotation());
    Eigen::Quaterniond q_target(target.rotation());
    q_prev.normalize();
    q_target.normalize();
    const double dot = std::clamp(std::abs(q_prev.dot(q_target)), 0.0, 1.0);
    const double angle = 2.0 * std::acos(dot);
    if (!std::isfinite(angle)) {
      RCLCPP_WARN(get_logger(), "[vive_rby1] dropping invalid SDK rotation for %s", arm_name);
      return prev;
    }

    Eigen::Quaterniond q_out = q_target;
    if (angle > sdk_max_delta_rot_ && sdk_max_delta_rot_ > 0.0) {
      const double ratio = sdk_max_delta_rot_ / angle;
      q_out = q_prev.slerp(ratio, q_target);
      q_out.normalize();
    }
    return SE3(q_out.toRotationMatrix(), pos);
  }

  SE3 smoothTracker(
    const std::optional<SE3> & prev, const geometry_msgs::msg::PoseStamped & msg) const {
    SE3 current = poseStampedToSe3(msg);
    if (!prev) {
      return current;
    }

    Eigen::Vector3d pos = current.translation();
    const Eigen::Vector3d raw_delta = pos - prev->translation();
    const double delta_norm = raw_delta.norm();
    constexpr double kMaxDelta = 0.05;
    if (delta_norm > kMaxDelta) {
      pos = prev->translation() + raw_delta / delta_norm * kMaxDelta;
    }

    Eigen::Quaterniond q_prev(prev->rotation());
    Eigen::Quaterniond q_new(current.rotation());
    q_prev.normalize();
    q_new.normalize();
    Eigen::Quaterniond q_smooth = q_prev.slerp(tracker_smooth_alpha_, q_new);
    q_smooth.normalize();
    return SE3(q_smooth.toRotationMatrix(), pos);
  }

  std::string trackerStatus(const TrackerState & tracker) const {
    if (nowSec() - tracker.stamp_sec > 0.5) {
      return "LOST";
    }
    if (tracker.buf.size() >= 10) {
      std::vector<Eigen::Vector3d> velocities;
      velocities.reserve(tracker.buf.size() - 1);
      for (size_t i = 1; i < tracker.buf.size(); ++i) {
        velocities.push_back(tracker.buf[i] - tracker.buf[i - 1]);
      }
      Eigen::Vector3d mean = Eigen::Vector3d::Zero();
      for (const auto & v : velocities) {
        mean += v;
      }
      mean /= static_cast<double>(velocities.size());
      Eigen::Vector3d var = Eigen::Vector3d::Zero();
      for (const auto & v : velocities) {
        const Eigen::Vector3d diff = v - mean;
        var += diff.cwiseProduct(diff);
      }
      var /= static_cast<double>(velocities.size());
      const double max_std = std::sqrt(var.maxCoeff());
      if (max_std > 0.003) {
        return "JITTER";
      }
    }
    return "OK";
  }

  // Publish the latest EE FK pose (from /rby1/state/pose) as a hold command so hw-core
  // snaps its cached target to the real EE position (stops residual motion at
  // warmup/cooldown).
  void publishEeHold(const rclcpp::Time & stamp) {
    if (!last_ee_r_ || !last_ee_l_) {
      return;
    }
    tf2_msgs::msg::TFMessage hold;
    hold.transforms.reserve(2);
    hold.transforms.push_back(se3ToTransformStamped("ee_right", *last_ee_r_, stamp));
    hold.transforms.push_back(se3ToTransformStamped("ee_left",  *last_ee_l_, stamp));
    pub_pose_cmd_->publish(hold);
  }

  void onTimer() {
    std_msgs::msg::String tracker_msg;
    tracker_msg.data = "L:" + trackerStatus(tracker_l_) + " R:" + trackerStatus(tracker_r_);
    if (tracker_b_.raw) {
      tracker_msg.data += " B:" + trackerStatus(tracker_b_);
    }
    pub_tracker_status_->publish(tracker_msg);

    if (warmup_ticks_ > 0) {
      --warmup_ticks_;
      publishEeHold(now());
      return;
    }

    // disengage 직후 cooldown: 현재 FK pose를 hold-publish해 hw-core의 캐시 target을
    // 실제 EE 위치로 덮어써(has_new=true) 펌웨어 잔여 모션을 즉시 정지시킨다.
    if (cooldown_ticks_ > 0) {
      --cooldown_ticks_;
      publishEeHold(now());
      return;
    }

    if (!tracker_l_.raw || !tracker_r_.raw || !tracker_l_.smoothed || !tracker_r_.smoothed) {
      return;
    }
    if (!engaged_ || !ref_l_ || !ref_r_ || !ee_l_0_ || !ee_r_0_ ||
        !sdk_ee_l_0_ || !sdk_ee_r_0_) {
      return;
    }

    const Eigen::Vector3d delta_l = tracker_l_.smoothed->translation() - ref_l_->translation();
    const Eigen::Vector3d delta_r = tracker_r_.smoothed->translation() - ref_r_->translation();

    Eigen::Vector3d target_pos_l;
    Eigen::Vector3d target_pos_r;
    Eigen::Matrix3d dR_l;
    Eigen::Matrix3d dR_r;

    if (mirror_mode_) {
      const Eigen::Matrix3d mirror_flip = (Eigen::Vector3d(1., -1., 1.)).asDiagonal();
      target_pos_l = ee_l_0_->translation() + pos_scale_ * (mirror_flip * v2r_R_ * delta_r);
      target_pos_r = ee_r_0_->translation() + pos_scale_ * (mirror_flip * v2r_R_ * delta_l);
      dR_l = tracker_r_.smoothed->rotation() * ref_r_->rotation().transpose();
      dR_r = tracker_l_.smoothed->rotation() * ref_l_->rotation().transpose();
    } else {
      target_pos_l = ee_l_0_->translation() + pos_scale_ * (v2r_R_ * delta_l);
      target_pos_r = ee_r_0_->translation() + pos_scale_ * (v2r_R_ * delta_r);
      dR_l = tracker_l_.smoothed->rotation() * ref_l_->rotation().transpose();
      dR_r = tracker_r_.smoothed->rotation() * ref_r_->rotation().transpose();
    }

    Eigen::Matrix3d dR_l_robot = v2r_R_ * dR_l * v2r_R_.transpose();
    Eigen::Matrix3d dR_r_robot = v2r_R_ * dR_r * v2r_R_.transpose();
    if (mirror_mode_) {
      const Eigen::Matrix3d mirror_flip_rot = (Eigen::Vector3d(1., -1., 1.)).asDiagonal();
      dR_l_robot = mirror_flip_rot * dR_l_robot * mirror_flip_rot;
      dR_r_robot = mirror_flip_rot * dR_r_robot * mirror_flip_rot;
    }

    const SE3 left_target(dR_l_robot * ee_l_0_->rotation(), target_pos_l);
    const SE3 right_target(dR_r_robot * ee_r_0_->rotation(), target_pos_r);

    // CartesianImpedance: rby1_core targets ee_right/ee_left (not tracker frame).
    // Re-anchor the delta from the tracker frame onto the ee frame reference.
    const SE3 sdk_right_target(
      dR_r_robot * sdk_ee_r_0_->rotation(),
      sdk_ee_r_0_->translation() + (target_pos_r - ee_r_0_->translation()));
    const SE3 sdk_left_target(
      dR_l_robot * sdk_ee_l_0_->rotation(),
      sdk_ee_l_0_->translation() + (target_pos_l - ee_l_0_->translation()));
    const auto sdk_l = limitSdkTarget(sdk_prev_l_, sdk_left_target, "left");
    const auto sdk_r = limitSdkTarget(sdk_prev_r_, sdk_right_target, "right");
    if (!sdk_l || !sdk_r) {
      return;
    }
    sdk_prev_l_ = sdk_l;
    sdk_prev_r_ = sdk_r;

    tf2_msgs::msg::TFMessage msg;
    const rclcpp::Time stamp = now();
    msg.transforms.reserve(3);
    msg.transforms.push_back(se3ToTransformStamped("ee_right", *sdk_r, stamp));
    msg.transforms.push_back(se3ToTransformStamped("ee_left",  *sdk_l, stamp));
    if (use_torso_ && ref_body_ && torso5_0_ && tracker_b_.smoothed) {
      Eigen::Vector3d delta_b = v2r_R_ * (tracker_b_.smoothed->translation() - ref_body_->translation());
      const Eigen::Matrix3d dR_b = tracker_b_.smoothed->rotation() * ref_body_->rotation().transpose();
      Eigen::Matrix3d dR_b_robot = v2r_R_ * dR_b * v2r_R_.transpose();
      if (mirror_mode_) {
        const Eigen::Matrix3d mf = (Eigen::Vector3d(1., -1., 1.)).asDiagonal();
        delta_b = mf * delta_b;
        dR_b_robot = mf * dR_b_robot * mf;
      }
      const SE3 torso_target(dR_b_robot * torso5_0_->rotation(),
                                        torso5_0_->translation() + torso_pos_scale_ * delta_b);
      msg.transforms.push_back(se3ToTransformStamped("link_torso_5", torso_target, stamp));
    }
    pub_pose_cmd_->publish(msg);
  }

  double nowSec() const {
    return static_cast<double>(
      const_cast<rclcpp::Clock &>(*get_clock()).now().nanoseconds()) * 1e-9;
  }

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_tracker_l_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_tracker_r_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_tracker_b_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr sub_pedal_;
  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr sub_state_pose_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr sub_task_id_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_mirror_mode_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr srv_set_use_torso_;

  rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr pub_pose_cmd_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_rec_state_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr pub_rec_episode_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_tracker_status_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_clutch_state_;

  rclcpp::Client<StartRecording>::SharedPtr cli_start_rec_;
  rclcpp::Client<EndRecording>::SharedPtr cli_end_rec_;
  rclcpp::Client<TogglePause>::SharedPtr cli_toggle_pause_;
  rclcpp::Client<SetControlMode>::SharedPtr cli_set_mode_;
  rclcpp::Client<SetStream>::SharedPtr cli_stream_;
  rclcpp::Client<MoveToJointPosition>::SharedPtr cli_move_joint_;
  rclcpp::Client<SetNullspaceJointRef>::SharedPtr cli_nullspace_joint_ref_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_teleop_start_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_teleop_stop_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_toggle_clutch_;
  rclcpp::Service<SetTeleOpPose>::SharedPtr srv_set_teleop_pose_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_toggle_episode_;
  rclcpp::TimerBase::SharedPtr timer_;

  TrackerState tracker_l_;
  TrackerState tracker_r_;
  TrackerState tracker_b_;
  // Latest hw-core FK from /rby1/state/pose (keyed by child_frame_id).
  std::optional<SE3> last_ee_r_;
  std::optional<SE3> last_ee_l_;
  std::optional<SE3> last_torso_;
  std::optional<SE3> ref_l_;
  std::optional<SE3> ref_r_;
  std::optional<SE3> ee_l_0_;
  std::optional<SE3> ee_r_0_;
  std::optional<SE3> sdk_ee_l_0_;   // SDK 모드용 ee_left 초기 참조
  std::optional<SE3> sdk_ee_r_0_;   // SDK 모드용 ee_right 초기 참조
  std::optional<SE3> sdk_prev_l_;
  std::optional<SE3> sdk_prev_r_;
  std::optional<SE3> ref_body_;
  std::optional<SE3> torso5_0_;

  Eigen::Matrix3d v2r_R_;

  std::string rec_state_{kRecIdle};
  int rec_episode_{-1};
  int rec_task_id_{0};

  // Default teleop pose (ready pose; overridden via /vive_rby1/set_teleop_pose)
  sensor_msgs::msg::JointState teleop_pose_ = []() {
    sensor_msgs::msg::JointState js;
    js.name = {"torso_0","torso_1","torso_2","torso_3","torso_4","torso_5",
               "right_arm_0","right_arm_1","right_arm_2","right_arm_3","right_arm_4","right_arm_5","right_arm_6",
               "left_arm_0","left_arm_1","left_arm_2","left_arm_3","left_arm_4","left_arm_5","left_arm_6"};
    constexpr double d2r = 3.14159265358979323846 / 180.0;
    js.position = {0, 30*d2r, -60*d2r, 30*d2r, 0, 0,
                   -8.68*d2r, -9.86*d2r,  1.89*d2r, -103.95*d2r,  0.37*d2r, 22.07*d2r, -10.35*d2r,
                   -8.68*d2r,  9.86*d2r, -1.89*d2r, -103.95*d2r, -0.37*d2r, 22.07*d2r,  10.35*d2r};
    return js;
  }();
  bool mirror_mode_{false};
  int warmup_ticks_{0};
  int cooldown_ticks_{0};   // disengage 직후 현재 FK pose를 hold-publish하는 잔여 틱
  bool teleop_active_{false};
  bool engaged_{false};
  bool pedal_engage_prev_{false};
  bool pedal_discard_prev_{false};
  bool pedal_episode_prev_{false};

  double pos_scale_{1.0};
  double torso_pos_scale_{1.0};
  bool use_torso_{false};
  double publish_rate_{20.0};
  double sdk_max_delta_pos_{0.03};
  double sdk_max_delta_rot_{20.0 * kPi / 180.0};
  double tracker_smooth_alpha_{0.9};
  double cooldown_sec_{0.5};   // disengage hold 지속 시간(초). 0이면 비활성
  size_t pedal_engage_idx_{0};
  size_t pedal_discard_idx_{1};
  size_t pedal_episode_idx_{2};
};

}  // namespace

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ViveRby1Node>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
