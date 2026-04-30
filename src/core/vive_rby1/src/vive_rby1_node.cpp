#include <algorithm>
#include <array>
#include <chrono>
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
#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "interbotix_xs_msgs/msg/joint_group_command.hpp"
#include "pinocchio/algorithm/frames.hpp"
#include "pinocchio/algorithm/jacobian.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/kinematics.hpp"
#include "pinocchio/multibody/model.hpp"
#include "pinocchio/parsers/urdf.hpp"
#include "pinocchio/spatial/explog.hpp"
#include "rby1_core_msgs/action/rby1_command.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "scm_recording_msgs/srv/end_recording.hpp"
#include "scm_recording_msgs/srv/start_recording.hpp"
#include "scm_recording_msgs/srv/toggle_pause.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/trigger.hpp"

namespace {

using namespace std::chrono_literals;

constexpr char kRecIdle[] = "IDLE";
constexpr char kRecReady[] = "READY";
constexpr char kRecRecording[] = "RECORDING";
constexpr char kRecPaused[] = "PAUSED";
constexpr double kPi = 3.14159265358979323846;

std::vector<std::string> bodyJointNames() {
  return {
      "torso_0",   "torso_1",   "torso_2",   "torso_3",  "torso_4",
      "torso_5",   "right_arm_0", "right_arm_1", "right_arm_2", "right_arm_3",
      "right_arm_4", "right_arm_5", "right_arm_6", "left_arm_0",  "left_arm_1",
      "left_arm_2",  "left_arm_3",  "left_arm_4",  "left_arm_5",  "left_arm_6"};
}

std::vector<std::string> torsoJointNames() {
  return {"torso_0", "torso_1", "torso_2", "torso_3", "torso_4", "torso_5"};
}

std::vector<std::string> activeArmJointNames() {
  return {
      "right_arm_0", "right_arm_1", "right_arm_2", "right_arm_3", "right_arm_4",
      "right_arm_5", "right_arm_6", "left_arm_0",  "left_arm_1",  "left_arm_2",
      "left_arm_3",  "left_arm_4",  "left_arm_5",  "left_arm_6"};
}

pinocchio::SE3 poseStampedToSe3(const geometry_msgs::msg::PoseStamped & msg) {
  const auto & p = msg.pose.position;
  const auto & q = msg.pose.orientation;
  Eigen::Quaterniond quat(q.w, q.x, q.y, q.z);
  quat.normalize();
  return pinocchio::SE3(quat.toRotationMatrix(), Eigen::Vector3d(p.x, p.y, p.z));
}

geometry_msgs::msg::Pose se3ToPose(const pinocchio::SE3 & se3) {
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

bool isFinite(const pinocchio::SE3 & se3) {
  return se3.translation().allFinite() && se3.rotation().allFinite();
}

class DifferentialIkSolver {
 public:
  DifferentialIkSolver(const std::string & urdf_path, const std::string & srdf_path)
  : body_joint_names_(bodyJointNames()), torso_joint_names_(torsoJointNames()) {
    (void)srdf_path;
    pinocchio::urdf::buildModel(urdf_path, model_);
    data_ = pinocchio::Data(model_);
    q_current_ = pinocchio::neutral(model_);

    right_frame_id_ = model_.getFrameId("tracker_right");
    left_frame_id_ = model_.getFrameId("tracker_left");

    for (const auto & name : body_joint_names_) {
      const auto joint_id = model_.getJointId(name);
      const auto & joint = model_.joints[joint_id];
      joint_q_index_.emplace(name, joint.idx_q());
    }
    for (const auto & name : activeArmJointNames()) {
      const auto joint_id = model_.getJointId(name);
      const auto & joint = model_.joints[joint_id];
      active_v_indices_.push_back(joint.idx_v());
    }
    pinocchio::forwardKinematics(model_, data_, q_current_);
    pinocchio::updateFramePlacements(model_, data_);
  }

  void updateFromJointState(
    const std::vector<std::string> & names, const std::vector<double> & positions) {
    std::lock_guard<std::mutex> lk(mtx_);
    q_current_ = pinocchio::neutral(model_);
    for (size_t i = 0; i < names.size() && i < positions.size(); ++i) {
      const auto it = joint_q_index_.find(names[i]);
      if (it != joint_q_index_.end()) {
        q_current_[it->second] = positions[i];
      }
    }
    pinocchio::forwardKinematics(model_, data_, q_current_);
    pinocchio::updateFramePlacements(model_, data_);
  }

  pinocchio::SE3 framePlacement(const std::string & frame_name) {
    std::lock_guard<std::mutex> lk(mtx_);
    const auto frame_id = model_.getFrameId(frame_name);
    pinocchio::forwardKinematics(model_, data_, q_current_);
    pinocchio::updateFramePlacements(model_, data_);
    return data_.oMf[frame_id];
  }

  Eigen::VectorXd currentQ20() {
    std::lock_guard<std::mutex> lk(mtx_);
    return qPinToQ20(q_current_);
  }

  Eigen::VectorXd solveToQ20(
    const pinocchio::SE3 & left_target, const pinocchio::SE3 & right_target, double dt) {
    std::lock_guard<std::mutex> lk(mtx_);

    pinocchio::forwardKinematics(model_, data_, q_current_);
    pinocchio::updateFramePlacements(model_, data_);

    const pinocchio::SE3 current_left = data_.oMf[left_frame_id_];
    const pinocchio::SE3 current_right = data_.oMf[right_frame_id_];

    Eigen::Matrix<double, 6, Eigen::Dynamic> jac_left(6, model_.nv);
    Eigen::Matrix<double, 6, Eigen::Dynamic> jac_right(6, model_.nv);
    pinocchio::computeFrameJacobian(
      model_, data_, q_current_, left_frame_id_, pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED,
      jac_left);
    pinocchio::computeFrameJacobian(
      model_, data_, q_current_, right_frame_id_, pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED,
      jac_right);

    Eigen::Matrix<double, 6, 1> err_left = pinocchio::log6(current_left.actInv(left_target)).toVector();
    Eigen::Matrix<double, 6, 1> err_right =
      pinocchio::log6(current_right.actInv(right_target)).toVector();

    err_left.head<3>() *= 50.0;
    err_left.tail<3>() *= 0.5;
    err_right.head<3>() *= 50.0;
    err_right.tail<3>() *= 0.5;

    Eigen::MatrixXd jac(12, active_v_indices_.size());
    for (size_t col = 0; col < active_v_indices_.size(); ++col) {
      jac.block<6, 1>(0, static_cast<Eigen::Index>(col)) = jac_left.col(active_v_indices_[col]);
      jac.block<6, 1>(6, static_cast<Eigen::Index>(col)) = jac_right.col(active_v_indices_[col]);
    }

    Eigen::VectorXd err(12);
    err.head<6>() = err_left / std::max(dt, 1e-3);
    err.tail<6>() = err_right / std::max(dt, 1e-3);

    const double lambda = 1e-2;
    const Eigen::MatrixXd jj_t =
      jac * jac.transpose() + lambda * lambda * Eigen::MatrixXd::Identity(12, 12);
    const Eigen::VectorXd dq_active = jac.transpose() * jj_t.ldlt().solve(err);

    Eigen::VectorXd dq = Eigen::VectorXd::Zero(model_.nv);
    for (size_t i = 0; i < active_v_indices_.size(); ++i) {
      dq[active_v_indices_[i]] = dq_active[static_cast<Eigen::Index>(i)];
    }

    const double max_teleop_dq = 1.5;
    const double max_abs = dq.cwiseAbs().maxCoeff();
    if (max_abs > max_teleop_dq) {
      dq *= (max_teleop_dq / max_abs);
    }

    const Eigen::VectorXd q_next = pinocchio::integrate(model_, q_current_, dq * dt);
    return qPinToQ20(q_next);
  }

 private:
  Eigen::VectorXd qPinToQ20(const Eigen::VectorXd & q_pin) const {
    Eigen::VectorXd q20(20);
    for (size_t i = 0; i < body_joint_names_.size(); ++i) {
      q20[static_cast<Eigen::Index>(i)] = q_pin[joint_q_index_.at(body_joint_names_[i])];
    }
    return q20;
  }

  pinocchio::Model model_;
  pinocchio::Data data_;
  Eigen::VectorXd q_current_;
  pinocchio::FrameIndex right_frame_id_{0};
  pinocchio::FrameIndex left_frame_id_{0};
  std::vector<std::string> body_joint_names_;
  std::vector<std::string> torso_joint_names_;
  std::vector<int> active_v_indices_;
  std::unordered_map<std::string, int> joint_q_index_;
  std::mutex mtx_;
};

class ViveRby1Node : public rclcpp::Node {
 public:
  using Rby1Command = rby1_core_msgs::action::Rby1Command;
  using GoalHandleRby1 = rclcpp_action::ClientGoalHandle<Rby1Command>;
  using StartRecording = scm_recording_msgs::srv::StartRecording;
  using EndRecording = scm_recording_msgs::srv::EndRecording;
  using TogglePause = scm_recording_msgs::srv::TogglePause;

  ViveRby1Node()
  : Node("vive_rby1_node"),
    v2r_R_((Eigen::Matrix3d() << 0., 1., 0., -1., 0., 0., 0., 0., 1.).finished()) {
    declare_parameter("urdf_path", "/home/hss/jyi/2026/robot_description/rby1/rby1.urdf");
    declare_parameter("srdf_path", "/home/hss/jyi/2026/robot_description/rby1/rby1.srdf");
    declare_parameter("topic_tracker_left", "/teleop/tracker/left");
    declare_parameter("topic_tracker_right", "/teleop/tracker/right");
    declare_parameter("topic_pedal", "/teleop/pedal");
    declare_parameter("topic_joint_state", "/rby1_status_joint");
    declare_parameter("topic_teleop_command", "/rby1_teleop_command");
    declare_parameter("pos_scale", 1.0);
    declare_parameter("ik_dt", 0.05);
    declare_parameter("publish_rate", 20.0);
    declare_parameter("sdk_max_delta_pos", 0.03);
    declare_parameter("sdk_max_delta_rot_deg", 20.0);
    declare_parameter("pedal_engage_index", 0);
    declare_parameter("pedal_episode_index", 2);

    const auto urdf_path = get_parameter("urdf_path").as_string();
    const auto srdf_path = get_parameter("srdf_path").as_string();
    const auto topic_l = get_parameter("topic_tracker_left").as_string();
    const auto topic_r = get_parameter("topic_tracker_right").as_string();
    const auto topic_p = get_parameter("topic_pedal").as_string();
    const auto topic_js = get_parameter("topic_joint_state").as_string();
    const auto topic_cmd = get_parameter("topic_teleop_command").as_string();

    pos_scale_ = get_parameter("pos_scale").as_double();
    ik_dt_ = get_parameter("ik_dt").as_double();
    publish_rate_ = get_parameter("publish_rate").as_double();
    sdk_max_delta_pos_ = get_parameter("sdk_max_delta_pos").as_double();
    sdk_max_delta_rot_ = get_parameter("sdk_max_delta_rot_deg").as_double() * kPi / 180.0;
    pedal_engage_idx_ = static_cast<size_t>(get_parameter("pedal_engage_index").as_int());
    pedal_episode_idx_ = static_cast<size_t>(get_parameter("pedal_episode_index").as_int());

    ik_solver_ = std::make_unique<DifferentialIkSolver>(urdf_path, srdf_path);
    RCLCPP_INFO(get_logger(), "[vive_rby1] IK solver ready");

    sub_tracker_l_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      topic_l, 10, std::bind(&ViveRby1Node::onTrackerLeft, this, std::placeholders::_1));
    sub_tracker_r_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      topic_r, 10, std::bind(&ViveRby1Node::onTrackerRight, this, std::placeholders::_1));
    sub_pedal_ = create_subscription<sensor_msgs::msg::Joy>(
      topic_p, 10, std::bind(&ViveRby1Node::onPedal, this, std::placeholders::_1));
    sub_joint_state_ = create_subscription<sensor_msgs::msg::JointState>(
      topic_js, 10, std::bind(&ViveRby1Node::onJointState, this, std::placeholders::_1));
    sub_task_id_ = create_subscription<std_msgs::msg::Int32>(
      "/teleop/task_id", 10, [this](const std_msgs::msg::Int32::SharedPtr msg) {
        rec_task_id_ = msg->data;
      });
    sub_control_mode_ = create_subscription<std_msgs::msg::String>(
      "/teleop/control_mode", 10, std::bind(&ViveRby1Node::onControlMode, this, std::placeholders::_1));
    sub_rby1_command_ = create_subscription<std_msgs::msg::String>(
      "/teleop/rby1_command", 10, std::bind(&ViveRby1Node::onRby1Command, this, std::placeholders::_1));
    sub_mirror_mode_ = create_subscription<std_msgs::msg::String>(
      "/teleop/mirror_mode", 10, std::bind(&ViveRby1Node::onMirrorMode, this, std::placeholders::_1));

    pub_cmd_ = create_publisher<interbotix_xs_msgs::msg::JointGroupCommand>(topic_cmd, 10);
    pub_impedance_cmd_ = create_publisher<interbotix_xs_msgs::msg::JointGroupCommand>(
      "/rby1_impedance_teleop_command", 10);
    pub_sdk_target_ = create_publisher<geometry_msgs::msg::PoseArray>("/rby1_sdk_teleop_command", 10);
    // ── EE Pose Publisher → warmup hold ──────────────────────────────────
    sub_ee_pose_ = create_subscription<geometry_msgs::msg::PoseArray>(
      "/rby1_ee_pose", 10,
      [this](const geometry_msgs::msg::PoseArray::SharedPtr msg) { last_ee_pose_ = *msg; });
    // ─────────────────────────────────────────────────────────────────────
    pub_rec_state_ = create_publisher<std_msgs::msg::String>("/teleop/rec_state", 10);
    pub_rec_episode_ = create_publisher<std_msgs::msg::Int32>("/teleop/rec_episode", 10);
    pub_tracker_status_ = create_publisher<std_msgs::msg::String>("/teleop/tracker_status", 10);
    pub_clutch_state_   = create_publisher<std_msgs::msg::String>("/teleop/clutch_state",   10);

    cli_start_rec_ = create_client<StartRecording>("/scm_recording/start");
    cli_end_rec_ = create_client<EndRecording>("/scm_recording/end");
    cli_toggle_pause_ = create_client<TogglePause>("/scm_recording/toggle_pause");
    rby1_client_ = rclcpp_action::create_client<Rby1Command>(this, "/rby1_command");

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
  struct TrackerState {
    geometry_msgs::msg::PoseStamped::SharedPtr raw;
    std::optional<pinocchio::SE3> smoothed;
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

  void onJointState(const sensor_msgs::msg::JointState::SharedPtr msg) {
    joint_state_ = msg;
    ik_solver_->updateFromJointState(msg->name, msg->position);
  }

  void onControlMode(const std_msgs::msg::String::SharedPtr msg) {
    ik_mode_ = msg->data;
    sdk_prev_l_.reset();
    sdk_prev_r_.reset();
    RCLCPP_INFO(get_logger(), "[vive_rby1] IK mode -- %s", ik_mode_.c_str());
  }

  void onMirrorMode(const std_msgs::msg::String::SharedPtr msg) {
    mirror_mode_ = (msg->data == "mirror");
    RCLCPP_INFO(get_logger(), "[vive_rby1] mirror mode -- %s", mirror_mode_ ? "true" : "false");
    if (engaged_) {
      ref_l_ = tracker_l_.smoothed;
      ref_r_ = tracker_r_.smoothed;
      ee_l_0_ = ik_solver_->framePlacement("tracker_left");
      ee_r_0_ = ik_solver_->framePlacement("tracker_right");
    }
  }

  void onRby1Command(const std_msgs::msg::String::SharedPtr msg) {
    std::string cmd = msg->data;
    if (cmd == "clutch_toggle") {
      if (!teleop_active_) {
        RCLCPP_WARN(get_logger(), "Cannot toggle clutch -- teleop not active");
      } else if (engaged_) {
        disengage();
      } else if (tracker_l_.raw && tracker_r_.raw) {
        engage();
      } else {
        RCLCPP_WARN(get_logger(), "Cannot engage -- Vive trackers not ready");
      }
      return;
    }
    if (cmd == "teleop_start") {
      if (ik_mode_ == "pink_impedance") {
        cmd = "impedance_teleop_start";
      } else if (ik_mode_ == "sdk_position") {
        cmd = "sdk_position_teleop_start";
      } else if (ik_mode_ == "sdk_impedance") {
        cmd = "sdk_impedance_teleop_start";
      }
    }
    sendRby1Command(cmd, "", nullptr);
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

    const bool episode_pressed =
      pedal_episode_idx_ < msg->buttons.size() && static_cast<bool>(msg->buttons[pedal_episode_idx_]);
    if (episode_pressed && !pedal_episode_prev_) {
      toggleEpisode();
    }
    pedal_episode_prev_ = episode_pressed;
  }

  void engage() {
    if (!tracker_l_.smoothed || !tracker_r_.smoothed) {
      RCLCPP_WARN(get_logger(), "Trackers not ready -- ignoring engage");
      return;
    }
    ref_l_ = tracker_l_.smoothed;
    ref_r_ = tracker_r_.smoothed;
    ee_l_0_ = ik_solver_->framePlacement("tracker_left");
    ee_r_0_ = ik_solver_->framePlacement("tracker_right");
    sdk_ee_l_0_ = ik_solver_->framePlacement("ee_left");
    sdk_ee_r_0_ = ik_solver_->framePlacement("ee_right");
    // SDK FK(rby1_rt) vs pinocchio FK(공칭 URDF) 간 Z 오프셋 보정
    // 측정값(ready pose): 우 +2.6cm, 좌 +3.9cm (SDK FK가 더 높음)
    sdk_ee_r_0_->translation().z() += 0.026;
    sdk_ee_l_0_->translation().z() += 0.039;
    sdk_prev_l_.reset();
    sdk_prev_r_.reset();
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
    publishClutchState();
    RCLCPP_INFO(get_logger(), "Clutch DISENGAGED");
    if (rec_state_ == kRecRecording) {
      callTogglePause();
    }
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
            rec_state_ = kRecReady;
            rec_episode_ = result->episode_id;
            RCLCPP_INFO(
              get_logger(), "[vive_rby1] READY -- task %d ep %d", result->task_id, result->episode_id);
            warmup_ticks_ = static_cast<int>(publish_rate_);
            std::string start_cmd = "teleop_start";
            if (ik_mode_ == "pink_impedance") {
              start_cmd = "impedance_teleop_start";
            } else if (ik_mode_ == "sdk_position") {
              start_cmd = "sdk_position_teleop_start";
            } else if (ik_mode_ == "sdk_impedance") {
              start_cmd = "sdk_impedance_teleop_start";
            }
            sendRby1Command("ready_pose", start_cmd, nullptr);
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
            RCLCPP_INFO(get_logger(), "[vive_rby1] Recording ENDED -- teleop_stop -- ready_pose");
            sendRby1Command("teleop_stop", "ready_pose", nullptr);
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

  void sendRby1Command(
    const std::string & command, const std::string & then,
    std::function<void()> on_complete) {
    if (!rby1_client_->wait_for_action_server(0s)) {
      RCLCPP_WARN(get_logger(), "rby1_command server not ready -- skipping \"%s\"", command.c_str());
      if (on_complete) {
        on_complete();
      }
      return;
    }

    if (command == "teleop_stop") {
      teleop_active_ = false;
      engaged_ = false;
      publishClutchState();
    }

    Rby1Command::Goal goal_msg;
    goal_msg.command = command;
    RCLCPP_INFO(get_logger(), "[vive_rby1] sending rby1_command: %s", command.c_str());

    rclcpp_action::Client<Rby1Command>::SendGoalOptions opts;
    opts.goal_response_callback =
      [this, command, then, on_complete](const GoalHandleRby1::SharedPtr & goal_handle) {
        if (!goal_handle) {
          RCLCPP_WARN(get_logger(), "rby1_command \"%s\" rejected", command.c_str());
          if (on_complete) {
            on_complete();
          }
        }
      };
    opts.result_callback =
      [this, command, then, on_complete](const GoalHandleRby1::WrappedResult & result) {
        const bool succeeded = result.code == rclcpp_action::ResultCode::SUCCEEDED;
        if (
          command == "teleop_start" || command == "impedance_teleop_start" ||
          command == "sdk_position_teleop_start" || command == "sdk_impedance_teleop_start")
        {
          teleop_active_ = succeeded;
          if (!succeeded) {
            RCLCPP_ERROR(
              get_logger(), "rby1_command \"%s\" failed -- stream may have expired",
              command.c_str());
          }
        }
        if (on_complete) {
          on_complete();
        }
        if (!then.empty() && succeeded) {
          std::thread([this, then]() {
            std::this_thread::sleep_for(1s);
            sendRby1Command(then, "", nullptr);
          }).detach();
        }
      };

    rby1_client_->async_send_goal(goal_msg, opts);
  }

  void publishRecState() {
    std_msgs::msg::String state_msg;
    state_msg.data = rec_state_;
    pub_rec_state_->publish(state_msg);

    std_msgs::msg::Int32 episode_msg;
    episode_msg.data = rec_episode_;
    pub_rec_episode_->publish(episode_msg);
  }

  std::optional<pinocchio::SE3> limitSdkTarget(
    const std::optional<pinocchio::SE3> & prev, const pinocchio::SE3 & target,
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
    return pinocchio::SE3(q_out.toRotationMatrix(), pos);
  }

  pinocchio::SE3 smoothTracker(
    const std::optional<pinocchio::SE3> & prev, const geometry_msgs::msg::PoseStamped & msg) const {
    pinocchio::SE3 current = poseStampedToSe3(msg);
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
    return pinocchio::SE3(q_smooth.toRotationMatrix(), pos);
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

  void onTimer() {
    std_msgs::msg::String tracker_msg;
    tracker_msg.data = "L:" + trackerStatus(tracker_l_) + " R:" + trackerStatus(tracker_r_);
    pub_tracker_status_->publish(tracker_msg);

    if (warmup_ticks_ > 0) {
      --warmup_ticks_;
      if (ik_mode_.rfind("sdk_", 0) != 0) {
        publishQ20(ik_solver_->currentQ20());
      } else if (last_ee_pose_) {
        // ── EE Pose Publisher → warmup hold ──────────────────────────────
        // Hold current EE pose during SDK mode warmup (FK from rby1_rt)
        pub_sdk_target_->publish(*last_ee_pose_);
        // ─────────────────────────────────────────────────────────────────
      }
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

    const pinocchio::SE3 left_target(dR_l_robot * ee_l_0_->rotation(), target_pos_l);
    const pinocchio::SE3 right_target(dR_r_robot * ee_r_0_->rotation(), target_pos_r);

    if (ik_mode_.rfind("sdk_", 0) == 0) {
      // SDK mode: rby1_rt targets ee_right/ee_left, not tracker frame.
      // tracker_right is offset from ee_right by [0.05, 0, -0.1] (URDF).
      // Re-anchor the already-correct delta onto the ee frame reference.
      const pinocchio::SE3 sdk_right_target(
        dR_r_robot * sdk_ee_r_0_->rotation(),
        sdk_ee_r_0_->translation() + (target_pos_r - ee_r_0_->translation()));
      const pinocchio::SE3 sdk_left_target(
        dR_l_robot * sdk_ee_l_0_->rotation(),
        sdk_ee_l_0_->translation() + (target_pos_l - ee_l_0_->translation()));
      const auto sdk_l = limitSdkTarget(sdk_prev_l_, sdk_left_target, "left");
      const auto sdk_r = limitSdkTarget(sdk_prev_r_, sdk_right_target, "right");
      if (!sdk_l || !sdk_r) {
        return;
      }
      sdk_prev_l_ = sdk_l;
      sdk_prev_r_ = sdk_r;

      geometry_msgs::msg::PoseArray msg;
      msg.header.frame_id = "base";
      msg.header.stamp = now();
      msg.poses.push_back(se3ToPose(*sdk_r));
      msg.poses.push_back(se3ToPose(*sdk_l));
      pub_sdk_target_->publish(msg);
      return;
    }

    publishQ20(ik_solver_->solveToQ20(left_target, right_target, ik_dt_));
  }

  void publishQ20(const Eigen::VectorXd & q20) {
    interbotix_xs_msgs::msg::JointGroupCommand cmd;
    cmd.name = "All";
    cmd.cmd.reserve(22);
    for (Eigen::Index i = 0; i < q20.size(); ++i) {
      cmd.cmd.push_back(q20[i]);
    }
    cmd.cmd.push_back(0.0);
    cmd.cmd.push_back(0.0);
    if (ik_mode_ == "pink_impedance") {
      pub_impedance_cmd_->publish(cmd);
    } else {
      pub_cmd_->publish(cmd);
    }
  }

  double nowSec() const {
    return static_cast<double>(
      const_cast<rclcpp::Clock &>(*get_clock()).now().nanoseconds()) * 1e-9;
  }

  std::unique_ptr<DifferentialIkSolver> ik_solver_;

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_tracker_l_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_tracker_r_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr sub_pedal_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_joint_state_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr sub_task_id_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_control_mode_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_rby1_command_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_mirror_mode_;
  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr sub_ee_pose_;  // EE Pose Publisher → warmup hold

  rclcpp::Publisher<interbotix_xs_msgs::msg::JointGroupCommand>::SharedPtr pub_cmd_;
  rclcpp::Publisher<interbotix_xs_msgs::msg::JointGroupCommand>::SharedPtr pub_impedance_cmd_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr pub_sdk_target_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_rec_state_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr pub_rec_episode_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_tracker_status_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_clutch_state_;

  rclcpp::Client<StartRecording>::SharedPtr cli_start_rec_;
  rclcpp::Client<EndRecording>::SharedPtr cli_end_rec_;
  rclcpp::Client<TogglePause>::SharedPtr cli_toggle_pause_;
  rclcpp_action::Client<Rby1Command>::SharedPtr rby1_client_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_toggle_episode_;
  rclcpp::TimerBase::SharedPtr timer_;

  TrackerState tracker_l_;
  TrackerState tracker_r_;
  sensor_msgs::msg::JointState::SharedPtr joint_state_;
  std::optional<pinocchio::SE3> ref_l_;
  std::optional<pinocchio::SE3> ref_r_;
  std::optional<pinocchio::SE3> ee_l_0_;
  std::optional<pinocchio::SE3> ee_r_0_;
  std::optional<pinocchio::SE3> sdk_ee_l_0_;   // SDK 모드용 ee_left 초기 참조
  std::optional<pinocchio::SE3> sdk_ee_r_0_;   // SDK 모드용 ee_right 초기 참조
  std::optional<pinocchio::SE3> sdk_prev_l_;
  std::optional<pinocchio::SE3> sdk_prev_r_;

  Eigen::Matrix3d v2r_R_;

  std::string rec_state_{kRecIdle};
  int rec_episode_{-1};
  int rec_task_id_{0};
  std::optional<geometry_msgs::msg::PoseArray> last_ee_pose_;  // EE Pose Publisher → warmup hold

  std::string ik_mode_{"pink_position"};
  bool mirror_mode_{false};
  int warmup_ticks_{0};
  bool teleop_active_{false};
  bool engaged_{false};
  bool pedal_engage_prev_{false};
  bool pedal_episode_prev_{false};

  double pos_scale_{1.0};
  double ik_dt_{0.05};
  double publish_rate_{20.0};
  double sdk_max_delta_pos_{0.03};
  double sdk_max_delta_rot_{20.0 * kPi / 180.0};
  double tracker_smooth_alpha_{0.9};
  size_t pedal_engage_idx_{0};
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
