#include <algorithm>
#include <array>
#include <cmath>
#include <cstdlib>
#include <limits>
#include <memory>
#include <mutex>
#include <optional>
#include <regex>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Geometry>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_model/joint_model_group.h>
#include <moveit/robot_state/robot_state.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2/exceptions.h>
#include <tf2/time.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

namespace
{
constexpr double kPi = 3.14159265358979323846;
constexpr double kEps = 1e-9;

double clamp(double value, double low, double high)
{
  return std::max(low, std::min(high, value));
}

double angleBetween(const Eigen::Vector3d & a, const Eigen::Vector3d & b)
{
  const double an = a.norm();
  const double bn = b.norm();
  if (an < kEps || bn < kEps) {
    return std::numeric_limits<double>::infinity();
  }
  const double c = clamp(a.dot(b) / (an * bn), -1.0, 1.0);
  return std::acos(c);
}

double wrapAngleDelta(double value)
{
  return std::remainder(value, 2.0 * kPi);
}

std::string boolText(bool value)
{
  return value ? "true" : "false";
}

std::string jsonString(const std::string & value)
{
  std::ostringstream out;
  out << '"';
  for (const char ch : value) {
    if (ch == '"' || ch == '\\') {
      out << '\\';
    }
    out << ch;
  }
  out << '"';
  return out.str();
}

std::string jsonNumber(double value)
{
  if (!std::isfinite(value)) {
    return "null";
  }
  std::ostringstream out;
  out << value;
  return out.str();
}

std::string jsonVec(const Eigen::Vector3d & v)
{
  std::ostringstream out;
  out << '[' << jsonNumber(v.x()) << ',' << jsonNumber(v.y()) << ',' << jsonNumber(v.z()) << ']';
  return out.str();
}

std::string jsonVec(const std::vector<double> & v)
{
  std::ostringstream out;
  out << '[';
  for (std::size_t i = 0; i < v.size(); ++i) {
    if (i > 0) {
      out << ',';
    }
    out << jsonNumber(v[i]);
  }
  out << ']';
  return out.str();
}

std::string jsonStringVec(const std::vector<std::string> & v)
{
  std::ostringstream out;
  out << '[';
  for (std::size_t i = 0; i < v.size(); ++i) {
    if (i > 0) {
      out << ',';
    }
    out << jsonString(v[i]);
  }
  out << ']';
  return out.str();
}

std::vector<double> parseNumbers(const std::string & text)
{
  std::vector<double> values;
  const char * ptr = text.c_str();
  while (*ptr != '\0') {
    char * end = nullptr;
    const double value = std::strtod(ptr, &end);
    if (end == ptr) {
      ++ptr;
      continue;
    }
    values.push_back(value);
    ptr = end;
  }
  return values;
}

std::optional<Eigen::Vector3d> extractArray3(const std::string & payload, const std::string & key)
{
  const std::regex re("\"" + key + "\"\\s*:\\s*\\[([^\\]]+)\\]");
  std::smatch match;
  if (!std::regex_search(payload, match, re) || match.size() < 2) {
    return std::nullopt;
  }
  const auto values = parseNumbers(match[1].str());
  if (values.size() < 3) {
    return std::nullopt;
  }
  return Eigen::Vector3d(values[0], values[1], values[2]);
}

Eigen::Isometry3d transformMsgToEigen(const geometry_msgs::msg::TransformStamped & tf)
{
  Eigen::Isometry3d out = Eigen::Isometry3d::Identity();
  Eigen::Quaterniond q(
    tf.transform.rotation.w,
    tf.transform.rotation.x,
    tf.transform.rotation.y,
    tf.transform.rotation.z);
  if (q.norm() < kEps) {
    q = Eigen::Quaterniond::Identity();
  } else {
    q.normalize();
  }
  out.linear() = q.toRotationMatrix();
  out.translation() = Eigen::Vector3d(
    tf.transform.translation.x,
    tf.transform.translation.y,
    tf.transform.translation.z);
  return out;
}
}  // namespace

class ViewGoalSolver : public rclcpp::Node
{
public:
  ViewGoalSolver()
  : Node("view_goal_solver"),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_)
  {
    declareParameters();
    loadParameters();

    cylinder_sub_ = create_subscription<std_msgs::msg::String>(
      cylinder_semantics_topic_, 10,
      std::bind(&ViewGoalSolver::onCylinderSemantics, this, std::placeholders::_1));
    target_id_sub_ = create_subscription<std_msgs::msg::Int32>(
      target_id_topic_, 10,
      std::bind(&ViewGoalSolver::onTargetId, this, std::placeholders::_1));

    joint_target_pub_ = create_publisher<sensor_msgs::msg::JointState>(joint_target_topic_, 10);
    debug_pub_ = create_publisher<std_msgs::msg::String>(debug_topic_, 10);

    RCLCPP_INFO(
      get_logger(),
      "view_goal_solver created. input=%s, joint_target=%s, debug=%s",
      cylinder_semantics_topic_.c_str(),
      joint_target_topic_.c_str(),
      debug_topic_.c_str());
  }

  void initializeMoveGroup()
  {
    moveit::planning_interface::MoveGroupInterface::Options options(
      planning_group_, "robot_description");
    move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
      shared_from_this(), options);
    robot_model_ = move_group_->getRobotModel();
    if (!robot_model_) {
      throw std::runtime_error("MoveGroup returned null RobotModel");
    }
    joint_model_group_ = robot_model_->getJointModelGroup(planning_group_);
    if (joint_model_group_ == nullptr) {
      throw std::runtime_error("Planning group not found: " + planning_group_);
    }

    joint_names_ = joint_model_group_->getVariableNames();
    bounds_.clear();
    bounds_.reserve(joint_names_.size());
    for (const auto & name : joint_names_) {
      const auto & b = robot_model_->getVariableBounds(name);
      bounds_.push_back(
        JointBound{b.position_bounded_, b.min_position_, b.max_position_});
    }

    if (!robot_model_->hasLinkModel(effector_frame_)) {
      const std::string fallback = move_group_->getEndEffectorLink();
      if (!fallback.empty() && robot_model_->hasLinkModel(fallback)) {
        RCLCPP_WARN(
          get_logger(),
          "effector_frame '%s' is not a RobotModel link. Falling back to MoveGroup end effector '%s'.",
          effector_frame_.c_str(),
          fallback.c_str());
        effector_frame_ = fallback;
      } else {
        throw std::runtime_error("Effector link not found in RobotModel: " + effector_frame_);
      }
    }

    RCLCPP_INFO(
      get_logger(),
      "MoveGroup initialized. group=%s, effector_frame=%s, camera_frame=%s, joints=%zu",
      planning_group_.c_str(),
      effector_frame_.c_str(),
      camera_frame_.c_str(),
      joint_names_.size());
  }

private:
  struct JointBound
  {
    bool bounded {false};
    double lower {-kPi};
    double upper {kPi};
  };

  struct Measurement
  {
    Eigen::Vector3d center {Eigen::Vector3d::Zero()};
    Eigen::Vector3d axis_raw {Eigen::Vector3d::UnitZ()};
  };

  struct Score
  {
    double total {std::numeric_limits<double>::infinity()};
    double gaze_error {std::numeric_limits<double>::infinity()};
    double axis_error {std::numeric_limits<double>::infinity()};
    double standoff_error {std::numeric_limits<double>::infinity()};
    double motion_cost {std::numeric_limits<double>::infinity()};
    double limit_cost {std::numeric_limits<double>::infinity()};
    double wrist_cost {0.0};
    double joint_limit_margin {std::numeric_limits<double>::infinity()};
    double distance_to_center {std::numeric_limits<double>::infinity()};
    Eigen::Vector3d camera_position {Eigen::Vector3d::Zero()};
    Eigen::Vector3d camera_z {Eigen::Vector3d::UnitZ()};
    bool finite {false};
  };

  struct Candidate
  {
    std::vector<double> q;
    Score score;
    int seed_index {0};
    int iterations {0};
  };

  void declareParameters()
  {
    declare_parameter("planning_group", "arm");
    declare_parameter("base_frame", "base_link");
    declare_parameter("effector_frame", "tcp_link");
    declare_parameter("camera_frame", "camera_color_optical_frame");
    declare_parameter("cylinder_semantics_topic", "/cylinder_semantics_base");
    declare_parameter("target_id_topic", "/target_id");
    declare_parameter("joint_target_topic", "/view_goal_joint_target");
    declare_parameter("debug_topic", "/view_goal_solver_debug");
    declare_parameter("require_target_id", true);
    declare_parameter("solve_once_per_selection", true);
    declare_parameter("axis_into_tube_sign", 1.0);
    declare_parameter("depth_proxy_m", 0.05);
    declare_parameter("standoff_desired_m", 0.10);
    declare_parameter("standoff_min_m", 0.06);
    declare_parameter("standoff_max_m", 0.18);
    declare_parameter("gaze_max_rad", 0.35);
    declare_parameter("axis_max_rad", 0.70);
    declare_parameter("weight_gaze", 8.0);
    declare_parameter("weight_axis", 2.0);
    declare_parameter("weight_standoff", 4.0);
    declare_parameter("weight_motion", 0.08);
    declare_parameter("weight_joint_limit", 0.35);
    declare_parameter("weight_wrist_motion", 0.15);
    declare_parameter("hard_constraint_weight", 80.0);
    declare_parameter("joint_limit_margin_threshold_rad", 0.12);
    declare_parameter("multi_start_count", 7);
    declare_parameter("max_iterations_per_seed", 120);
    declare_parameter("initial_step_rad", 0.10);
    declare_parameter("min_step_rad", 0.003);
    declare_parameter("step_shrink", 0.55);
    declare_parameter("seed_perturbation_rad", 0.35);
    declare_parameter("current_state_wait_sec", 1.0);
    declare_parameter("tf_lookup_timeout_sec", 0.5);
    declare_parameter("allow_identity_effector_camera_fallback", false);
    declare_parameter("publish_if_constraints_fail", false);
  }

  void loadParameters()
  {
    planning_group_ = get_parameter("planning_group").as_string();
    base_frame_ = get_parameter("base_frame").as_string();
    effector_frame_ = get_parameter("effector_frame").as_string();
    camera_frame_ = get_parameter("camera_frame").as_string();
    cylinder_semantics_topic_ = get_parameter("cylinder_semantics_topic").as_string();
    target_id_topic_ = get_parameter("target_id_topic").as_string();
    joint_target_topic_ = get_parameter("joint_target_topic").as_string();
    debug_topic_ = get_parameter("debug_topic").as_string();
    require_target_id_ = get_parameter("require_target_id").as_bool();
    solve_once_per_selection_ = get_parameter("solve_once_per_selection").as_bool();
    axis_into_tube_sign_ = get_parameter("axis_into_tube_sign").as_double();
    depth_proxy_m_ = get_parameter("depth_proxy_m").as_double();
    standoff_desired_m_ = get_parameter("standoff_desired_m").as_double();
    standoff_min_m_ = get_parameter("standoff_min_m").as_double();
    standoff_max_m_ = get_parameter("standoff_max_m").as_double();
    gaze_max_rad_ = get_parameter("gaze_max_rad").as_double();
    axis_max_rad_ = get_parameter("axis_max_rad").as_double();
    weight_gaze_ = get_parameter("weight_gaze").as_double();
    weight_axis_ = get_parameter("weight_axis").as_double();
    weight_standoff_ = get_parameter("weight_standoff").as_double();
    weight_motion_ = get_parameter("weight_motion").as_double();
    weight_joint_limit_ = get_parameter("weight_joint_limit").as_double();
    weight_wrist_motion_ = get_parameter("weight_wrist_motion").as_double();
    hard_constraint_weight_ = get_parameter("hard_constraint_weight").as_double();
    joint_limit_margin_threshold_rad_ =
      get_parameter("joint_limit_margin_threshold_rad").as_double();
    multi_start_count_ = std::max(1, static_cast<int>(get_parameter("multi_start_count").as_int()));
    max_iterations_per_seed_ =
      std::max(1, static_cast<int>(get_parameter("max_iterations_per_seed").as_int()));
    initial_step_rad_ = std::max(1e-4, get_parameter("initial_step_rad").as_double());
    min_step_rad_ = std::max(1e-5, get_parameter("min_step_rad").as_double());
    step_shrink_ = clamp(get_parameter("step_shrink").as_double(), 0.1, 0.95);
    seed_perturbation_rad_ = std::max(0.0, get_parameter("seed_perturbation_rad").as_double());
    current_state_wait_sec_ = std::max(0.0, get_parameter("current_state_wait_sec").as_double());
    tf_lookup_timeout_sec_ = std::max(0.0, get_parameter("tf_lookup_timeout_sec").as_double());
    allow_identity_effector_camera_fallback_ =
      get_parameter("allow_identity_effector_camera_fallback").as_bool();
    publish_if_constraints_fail_ = get_parameter("publish_if_constraints_fail").as_bool();
  }

  void onTargetId(const std_msgs::msg::Int32::SharedPtr msg)
  {
    if (!msg) {
      return;
    }
    selected_target_id_ = msg->data;
    has_target_id_ = true;
    solved_current_selection_ = false;
    RCLCPP_INFO(get_logger(), "View goal target id switched to %d", selected_target_id_);
  }

  void onCylinderSemantics(const std_msgs::msg::String::SharedPtr msg)
  {
    if (!msg) {
      return;
    }
    if (require_target_id_ && !has_target_id_) {
      return;
    }
    if (solve_once_per_selection_ && solved_current_selection_) {
      return;
    }

    const auto measurement = parseMeasurement(msg->data);
    if (!measurement) {
      publishDebugFailure("parse_failed", "missing top_center_base or axis_base");
      RCLCPP_WARN(get_logger(), "Failed to parse /cylinder_semantics_base");
      return;
    }

    std::lock_guard<std::mutex> lock(solve_mutex_);
    solveAndPublish(*measurement);
  }

  std::optional<Measurement> parseMeasurement(const std::string & payload) const
  {
    const auto center = extractArray3(payload, "top_center_base");
    const auto axis = extractArray3(payload, "axis_base");
    if (!center || !axis) {
      return std::nullopt;
    }
    if (!std::isfinite(center->x()) || !std::isfinite(center->y()) || !std::isfinite(center->z())) {
      return std::nullopt;
    }
    if (axis->norm() < kEps) {
      return std::nullopt;
    }
    Measurement out;
    out.center = *center;
    out.axis_raw = axis->normalized();
    return out;
  }

  std::optional<Eigen::Isometry3d> lookupEffectorToCamera()
  {
    if (effector_frame_ == camera_frame_) {
      return Eigen::Isometry3d::Identity();
    }

    try {
      const auto tf_msg = tf_buffer_.lookupTransform(
        effector_frame_,
        camera_frame_,
        tf2::TimePointZero,
        tf2::durationFromSec(tf_lookup_timeout_sec_));
      return transformMsgToEigen(tf_msg);
    } catch (const tf2::TransformException & ex) {
      if (allow_identity_effector_camera_fallback_) {
        RCLCPP_WARN(
          get_logger(),
          "TF %s <- %s unavailable (%s). Using identity fallback.",
          effector_frame_.c_str(),
          camera_frame_.c_str(),
          ex.what());
        return Eigen::Isometry3d::Identity();
      }
      RCLCPP_WARN(
        get_logger(),
        "TF %s <- %s unavailable: %s",
        effector_frame_.c_str(),
        camera_frame_.c_str(),
        ex.what());
      return std::nullopt;
    }
  }

  Eigen::Vector3d resolveViewAxis(
    const Eigen::Vector3d & center,
    const Eigen::Vector3d & raw_axis,
    const std::optional<Eigen::Vector3d> & current_camera_position) const
  {
    Eigen::Vector3d axis = raw_axis.normalized();
    if (!current_camera_position) {
      return axis * ((axis_into_tube_sign_ >= 0.0) ? 1.0 : -1.0);
    }

    Eigen::Vector3d camera_side = *current_camera_position - center;
    if (camera_side.norm() < 1e-6) {
      return axis * ((axis_into_tube_sign_ >= 0.0) ? 1.0 : -1.0);
    }
    camera_side.normalize();

    const double same_score = camera_side.dot(-axis);
    const double flipped_score = camera_side.dot(axis);
    if (flipped_score > same_score) {
      axis = -axis;
    }
    return axis;
  }

  void solveAndPublish(const Measurement & measurement)
  {
    if (!move_group_ || joint_model_group_ == nullptr) {
      publishDebugFailure("not_ready", "MoveGroup not initialized");
      return;
    }

    const auto effector_to_camera = lookupEffectorToCamera();
    if (!effector_to_camera) {
      publishDebugFailure("tf_failed", "missing effector to camera transform");
      return;
    }

    const auto current_state = move_group_->getCurrentState(current_state_wait_sec_);
    if (!current_state) {
      publishDebugFailure("state_failed", "MoveGroup current state unavailable");
      return;
    }

    std::vector<double> q0;
    current_state->copyJointGroupPositions(joint_model_group_, q0);
    if (q0.size() != joint_names_.size()) {
      publishDebugFailure("state_failed", "joint vector size mismatch");
      return;
    }

    const Eigen::Isometry3d current_effector =
      current_state->getGlobalLinkTransform(effector_frame_);
    const Eigen::Isometry3d current_camera = current_effector * (*effector_to_camera);
    const Eigen::Vector3d view_axis =
      resolveViewAxis(measurement.center, measurement.axis_raw, current_camera.translation());

    const Candidate best = optimize(measurement, view_axis, *effector_to_camera, *current_state, q0);
    const bool constraints_ok = constraintsSatisfied(best.score);
    const bool should_publish = best.score.finite && (constraints_ok || publish_if_constraints_fail_);

    publishDebug(measurement, view_axis, q0, best, constraints_ok, should_publish);

    if (!should_publish) {
      RCLCPP_WARN(
        get_logger(),
        "Rejected q-space view goal. finite=%s, constraints_ok=%s, score=%.6f",
        boolText(best.score.finite).c_str(),
        boolText(constraints_ok).c_str(),
        best.score.total);
      return;
    }

    sensor_msgs::msg::JointState joint_target;
    joint_target.header.stamp = now();
    joint_target.header.frame_id = base_frame_;
    joint_target.name = joint_names_;
    joint_target.position = best.q;
    joint_target_pub_->publish(joint_target);

    solved_current_selection_ = true;
    RCLCPP_INFO(
      get_logger(),
      "Published q-space view goal. score=%.6f, gaze=%.4f rad, axis=%.4f rad, dist=%.4f m",
      best.score.total,
      best.score.gaze_error,
      best.score.axis_error,
      best.score.distance_to_center);
  }

  Candidate optimize(
    const Measurement & measurement,
    const Eigen::Vector3d & view_axis,
    const Eigen::Isometry3d & effector_to_camera,
    const moveit::core::RobotState & current_state,
    const std::vector<double> & q0)
  {
    Candidate best;
    best.q = q0;

    const auto seeds = makeSeeds(q0);
    for (std::size_t i = 0; i < seeds.size(); ++i) {
      Candidate candidate;
      candidate.seed_index = static_cast<int>(i);
      candidate.q = seeds[i];
      clampToBounds(candidate.q);
      candidate.score = evaluate(
        candidate.q,
        q0,
        measurement,
        view_axis,
        effector_to_camera,
        current_state);

      double step = initial_step_rad_;
      int iter = 0;
      while (iter < max_iterations_per_seed_ && step >= min_step_rad_) {
        bool improved = false;
        for (std::size_t joint_idx = 0; joint_idx < candidate.q.size(); ++joint_idx) {
          auto trial_plus = candidate.q;
          trial_plus[joint_idx] += step;
          clampToBounds(trial_plus);
          const Score plus_score = evaluate(
            trial_plus,
            q0,
            measurement,
            view_axis,
            effector_to_camera,
            current_state);

          auto trial_minus = candidate.q;
          trial_minus[joint_idx] -= step;
          clampToBounds(trial_minus);
          const Score minus_score = evaluate(
            trial_minus,
            q0,
            measurement,
            view_axis,
            effector_to_camera,
            current_state);

          if (plus_score.total + 1e-12 < candidate.score.total &&
            plus_score.total <= minus_score.total)
          {
            candidate.q = std::move(trial_plus);
            candidate.score = plus_score;
            improved = true;
          } else if (minus_score.total + 1e-12 < candidate.score.total) {
            candidate.q = std::move(trial_minus);
            candidate.score = minus_score;
            improved = true;
          }
        }

        ++iter;
        if (!improved) {
          step *= step_shrink_;
        }
      }
      candidate.iterations = iter;

      if (candidate.score.total < best.score.total) {
        best = candidate;
      }
    }
    return best;
  }

  std::vector<std::vector<double>> makeSeeds(const std::vector<double> & q0) const
  {
    std::vector<std::vector<double>> seeds;
    seeds.reserve(static_cast<std::size_t>(multi_start_count_));
    seeds.push_back(q0);

    if (multi_start_count_ <= 1) {
      return seeds;
    }

    std::vector<double> centered = q0;
    for (std::size_t i = 0; i < centered.size(); ++i) {
      if (bounds_[i].bounded) {
        centered[i] = 0.5 * (bounds_[i].lower + bounds_[i].upper);
      }
    }
    seeds.push_back(centered);

    for (int seed_idx = 2; seed_idx < multi_start_count_; ++seed_idx) {
      std::vector<double> seed = q0;
      const double phase = static_cast<double>(seed_idx);
      for (std::size_t joint_idx = 0; joint_idx < seed.size(); ++joint_idx) {
        const double sign = ((seed_idx + static_cast<int>(joint_idx)) % 2 == 0) ? 1.0 : -1.0;
        const double scale = 0.35 + 0.65 * std::abs(std::sin(phase * (joint_idx + 1.0)));
        seed[joint_idx] += sign * seed_perturbation_rad_ * scale;
      }
      seeds.push_back(seed);
    }

    return seeds;
  }

  Score evaluate(
    const std::vector<double> & q,
    const std::vector<double> & q0,
    const Measurement & measurement,
    const Eigen::Vector3d & view_axis,
    const Eigen::Isometry3d & effector_to_camera,
    const moveit::core::RobotState & template_state) const
  {
    Score score;
    if (q.size() != joint_names_.size() || q0.size() != joint_names_.size()) {
      return score;
    }

    moveit::core::RobotState state(template_state);
    state.setJointGroupPositions(joint_model_group_, q);
    state.enforceBounds(joint_model_group_);
    state.update();

    if (!state.satisfiesBounds(joint_model_group_, 0.0)) {
      return score;
    }

    const Eigen::Isometry3d base_to_effector = state.getGlobalLinkTransform(effector_frame_);
    const Eigen::Isometry3d base_to_camera = base_to_effector * effector_to_camera;
    const Eigen::Vector3d p_cam = base_to_camera.translation();
    const Eigen::Vector3d z_cam = base_to_camera.linear() * Eigen::Vector3d::UnitZ();
    const Eigen::Vector3d bottom_proxy = measurement.center + depth_proxy_m_ * view_axis;
    const Eigen::Vector3d to_bottom = bottom_proxy - p_cam;

    if (to_bottom.norm() < kEps || z_cam.norm() < kEps) {
      return score;
    }

    score.camera_position = p_cam;
    score.camera_z = z_cam.normalized();
    score.distance_to_center = (measurement.center - p_cam).norm();
    score.gaze_error = angleBetween(score.camera_z, to_bottom.normalized());
    score.axis_error = angleBetween(score.camera_z, view_axis);
    score.standoff_error = score.distance_to_center - standoff_desired_m_;
    score.motion_cost = motionCost(q, q0);
    score.limit_cost = jointLimitCost(q, score.joint_limit_margin);
    score.wrist_cost = wristMotionCost(q, q0);

    const double gaze_alignment = 1.0 - clamp(score.camera_z.dot(to_bottom.normalized()), -1.0, 1.0);
    const double axis_alignment = 1.0 - clamp(score.camera_z.dot(view_axis), -1.0, 1.0);
    const double standoff_scale = std::max(0.03, standoff_desired_m_);
    const double standoff_cost =
      (score.standoff_error * score.standoff_error) / (standoff_scale * standoff_scale);

    double hard_penalty = 0.0;
    if (score.distance_to_center < standoff_min_m_) {
      const double v = (standoff_min_m_ - score.distance_to_center) / standoff_scale;
      hard_penalty += v * v;
    }
    if (score.distance_to_center > standoff_max_m_) {
      const double v = (score.distance_to_center - standoff_max_m_) / standoff_scale;
      hard_penalty += v * v;
    }
    if (score.gaze_error > gaze_max_rad_) {
      const double v = score.gaze_error - gaze_max_rad_;
      hard_penalty += v * v;
    }
    if (score.axis_error > axis_max_rad_) {
      const double v = score.axis_error - axis_max_rad_;
      hard_penalty += v * v;
    }

    score.total =
      weight_gaze_ * gaze_alignment +
      weight_axis_ * axis_alignment +
      weight_standoff_ * standoff_cost +
      weight_motion_ * score.motion_cost +
      weight_joint_limit_ * score.limit_cost +
      weight_wrist_motion_ * score.wrist_cost +
      hard_constraint_weight_ * hard_penalty;
    score.finite = std::isfinite(score.total);
    return score;
  }

  double motionCost(const std::vector<double> & q, const std::vector<double> & q0) const
  {
    double cost = 0.0;
    for (std::size_t i = 0; i < q.size(); ++i) {
      const double span = jointSpan(i);
      const double d = wrapAngleDelta(q[i] - q0[i]) / span;
      cost += d * d;
    }
    return cost;
  }

  double wristMotionCost(const std::vector<double> & q, const std::vector<double> & q0) const
  {
    if (q.size() < 3) {
      return 0.0;
    }
    double cost = 0.0;
    const std::size_t start = q.size() - 3;
    for (std::size_t i = start; i < q.size(); ++i) {
      const double span = jointSpan(i);
      const double d = wrapAngleDelta(q[i] - q0[i]) / span;
      cost += d * d;
    }
    return cost;
  }

  double jointLimitCost(const std::vector<double> & q, double & min_margin) const
  {
    double cost = 0.0;
    min_margin = std::numeric_limits<double>::infinity();
    const double threshold = std::max(1e-4, joint_limit_margin_threshold_rad_);
    for (std::size_t i = 0; i < q.size(); ++i) {
      if (!bounds_[i].bounded) {
        continue;
      }
      const double margin = std::min(q[i] - bounds_[i].lower, bounds_[i].upper - q[i]);
      min_margin = std::min(min_margin, margin);
      const double violation = std::max(0.0, threshold - margin) / threshold;
      cost += violation * violation;
    }

    if (!std::isfinite(min_margin)) {
      min_margin = std::numeric_limits<double>::infinity();
    }
    return cost;
  }

  double jointSpan(std::size_t index) const
  {
    if (index < bounds_.size() && bounds_[index].bounded) {
      return std::max(0.1, bounds_[index].upper - bounds_[index].lower);
    }
    return 2.0 * kPi;
  }

  void clampToBounds(std::vector<double> & q) const
  {
    for (std::size_t i = 0; i < q.size() && i < bounds_.size(); ++i) {
      if (bounds_[i].bounded) {
        q[i] = clamp(q[i], bounds_[i].lower, bounds_[i].upper);
      } else {
        q[i] = std::remainder(q[i], 2.0 * kPi);
      }
    }
  }

  bool constraintsSatisfied(const Score & score) const
  {
    if (!score.finite) {
      return false;
    }
    return
      score.distance_to_center >= standoff_min_m_ &&
      score.distance_to_center <= standoff_max_m_ &&
      score.gaze_error <= gaze_max_rad_ &&
      score.axis_error <= axis_max_rad_ &&
      score.joint_limit_margin >= 0.0;
  }

  void publishDebugFailure(const std::string & reason, const std::string & detail)
  {
    std_msgs::msg::String msg;
    std::ostringstream out;
    out << "{"
        << "\"mode\":\"q_space_fk_optimization\","
        << "\"selected_target_id\":" << (has_target_id_ ? std::to_string(selected_target_id_) : "null") << ','
        << "\"ik_success\":false,"
        << "\"plan_success\":false,"
        << "\"plan_success_checked\":false,"
        << "\"selected_reason\":" << jsonString(reason) << ','
        << "\"detail\":" << jsonString(detail)
        << "}";
    msg.data = out.str();
    debug_pub_->publish(msg);
  }

  void publishDebug(
    const Measurement & measurement,
    const Eigen::Vector3d & view_axis,
    const std::vector<double> & q0,
    const Candidate & best,
    bool constraints_ok,
    bool published)
  {
    std_msgs::msg::String msg;
    std::ostringstream out;
    out << "{"
        << "\"mode\":\"q_space_fk_coordinate_descent\","
        << "\"selected_target_id\":" << (has_target_id_ ? std::to_string(selected_target_id_) : "null") << ','
        << "\"center_base\":" << jsonVec(measurement.center) << ','
        << "\"axis_raw_base\":" << jsonVec(measurement.axis_raw) << ','
        << "\"view_axis_base\":" << jsonVec(view_axis) << ','
        << "\"camera_position_base\":" << jsonVec(best.score.camera_position) << ','
        << "\"camera_z_base\":" << jsonVec(best.score.camera_z) << ','
        << "\"joint_names\":" << jsonStringVec(joint_names_) << ','
        << "\"q_start\":" << jsonVec(q0) << ','
        << "\"q_goal\":" << jsonVec(best.q) << ','
        << "\"score\":" << jsonNumber(best.score.total) << ','
        << "\"gaze_error\":" << jsonNumber(best.score.gaze_error) << ','
        << "\"axis_error\":" << jsonNumber(best.score.axis_error) << ','
        << "\"standoff_error\":" << jsonNumber(best.score.standoff_error) << ','
        << "\"distance_to_center\":" << jsonNumber(best.score.distance_to_center) << ','
        << "\"fov_score\":-1.0,"
        << "\"ik_success\":" << boolText(best.score.finite) << ','
        << "\"plan_success\":false,"
        << "\"plan_success_checked\":false,"
        << "\"joint_limit_margin\":" << jsonNumber(best.score.joint_limit_margin) << ','
        << "\"joint_limit_cost\":" << jsonNumber(best.score.limit_cost) << ','
        << "\"motion_cost\":" << jsonNumber(best.score.motion_cost) << ','
        << "\"wrist_motion_cost\":" << jsonNumber(best.score.wrist_cost) << ','
        << "\"collision_checked\":false,"
        << "\"constraints_satisfied\":" << boolText(constraints_ok) << ','
        << "\"published\":" << boolText(published) << ','
        << "\"seed_index\":" << best.seed_index << ','
        << "\"iterations\":" << best.iterations << ','
        << "\"selected_reason\":" << jsonString(published ? "optimized_q_goal" : "constraints_failed")
        << "}";
    msg.data = out.str();
    debug_pub_->publish(msg);
  }

  std::string planning_group_;
  std::string base_frame_;
  std::string effector_frame_;
  std::string camera_frame_;
  std::string cylinder_semantics_topic_;
  std::string target_id_topic_;
  std::string joint_target_topic_;
  std::string debug_topic_;

  bool require_target_id_ {true};
  bool solve_once_per_selection_ {true};
  bool allow_identity_effector_camera_fallback_ {false};
  bool publish_if_constraints_fail_ {false};

  double axis_into_tube_sign_ {1.0};
  double depth_proxy_m_ {0.05};
  double standoff_desired_m_ {0.10};
  double standoff_min_m_ {0.06};
  double standoff_max_m_ {0.18};
  double gaze_max_rad_ {0.35};
  double axis_max_rad_ {0.70};
  double weight_gaze_ {8.0};
  double weight_axis_ {2.0};
  double weight_standoff_ {4.0};
  double weight_motion_ {0.08};
  double weight_joint_limit_ {0.35};
  double weight_wrist_motion_ {0.15};
  double hard_constraint_weight_ {80.0};
  double joint_limit_margin_threshold_rad_ {0.12};
  double initial_step_rad_ {0.10};
  double min_step_rad_ {0.003};
  double step_shrink_ {0.55};
  double seed_perturbation_rad_ {0.35};
  double current_state_wait_sec_ {1.0};
  double tf_lookup_timeout_sec_ {0.5};
  int multi_start_count_ {7};
  int max_iterations_per_seed_ {120};

  int selected_target_id_ {0};
  bool has_target_id_ {false};
  bool solved_current_selection_ {false};

  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
  moveit::core::RobotModelConstPtr robot_model_;
  const moveit::core::JointModelGroup * joint_model_group_ {nullptr};
  std::vector<std::string> joint_names_;
  std::vector<JointBound> bounds_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr cylinder_sub_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr target_id_sub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_target_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr debug_pub_;

  std::mutex solve_mutex_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ViewGoalSolver>();
  node->initializeMoveGroup();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
