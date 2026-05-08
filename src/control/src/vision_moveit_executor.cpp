#include <algorithm>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/string.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

class VisionMoveItExecutor : public rclcpp::Node
{
public:
  VisionMoveItExecutor()
  : Node("vision_moveit_executor")
  {
    this->declare_parameter("planning_group", "arm");
    this->declare_parameter("base_frame", "base_link");
    this->declare_parameter("target_pose_stamped_topic", "/target_pose_stamped");
    this->declare_parameter("joint_target_topic", "/view_goal_joint_target");
    this->declare_parameter("joint_candidate_topic", "/view_goal_joint_candidates");
    this->declare_parameter("status_topic", "/vision_exec_status");
    this->declare_parameter("enable_pose_target_subscription", false);
    this->declare_parameter("enable_joint_target_subscription", false);
    this->declare_parameter("enable_joint_candidate_subscription", true);
    this->declare_parameter("planning_time", 3.0);
    this->declare_parameter("num_planning_attempts", 3);
    this->declare_parameter("max_velocity_scaling_factor", 0.25);
    this->declare_parameter("max_acceleration_scaling_factor", 0.25);
    this->declare_parameter("goal_joint_tolerance", 0.005);
    this->declare_parameter("goal_position_tolerance", 0.01);
    this->declare_parameter("goal_orientation_tolerance", 0.6);

    planning_group_ = this->get_parameter("planning_group").as_string();
    base_frame_ = this->get_parameter("base_frame").as_string();
    target_pose_stamped_topic_ = this->get_parameter("target_pose_stamped_topic").as_string();
    joint_target_topic_ = this->get_parameter("joint_target_topic").as_string();
    joint_candidate_topic_ = this->get_parameter("joint_candidate_topic").as_string();
    enable_pose_target_subscription_ =
      this->get_parameter("enable_pose_target_subscription").as_bool();
    enable_joint_target_subscription_ =
      this->get_parameter("enable_joint_target_subscription").as_bool();
    enable_joint_candidate_subscription_ =
      this->get_parameter("enable_joint_candidate_subscription").as_bool();
    planning_time_ = this->get_parameter("planning_time").as_double();
    num_planning_attempts_ = std::max(
      1, static_cast<int>(this->get_parameter("num_planning_attempts").as_int()));
    max_velocity_scaling_factor_ = this->get_parameter("max_velocity_scaling_factor").as_double();
    max_acceleration_scaling_factor_ = this->get_parameter("max_acceleration_scaling_factor").as_double();
    goal_joint_tolerance_ = this->get_parameter("goal_joint_tolerance").as_double();
    goal_position_tolerance_ = this->get_parameter("goal_position_tolerance").as_double();
    goal_orientation_tolerance_ = this->get_parameter("goal_orientation_tolerance").as_double();

    status_pub_ = this->create_publisher<std_msgs::msg::String>(
      this->get_parameter("status_topic").as_string(), 10);

    if (enable_pose_target_subscription_) {
      target_pose_stamped_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        target_pose_stamped_topic_, 10,
        std::bind(&VisionMoveItExecutor::onTargetPoseStamped, this, std::placeholders::_1));
    }

    if (enable_joint_target_subscription_) {
      joint_target_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
        joint_target_topic_, 10,
        std::bind(&VisionMoveItExecutor::onJointTarget, this, std::placeholders::_1));
    }

    if (enable_joint_candidate_subscription_) {
      joint_candidate_sub_ = this->create_subscription<trajectory_msgs::msg::JointTrajectory>(
        joint_candidate_topic_, 10,
        std::bind(&VisionMoveItExecutor::onJointCandidates, this, std::placeholders::_1));
    }

    RCLCPP_INFO(
      this->get_logger(),
      "vision_moveit_executor created. group=%s, pose_enabled=%s, joint_target_enabled=%s, joint_candidates_enabled=%s",
      planning_group_.c_str(),
      boolText(enable_pose_target_subscription_).c_str(),
      boolText(enable_joint_target_subscription_).c_str(),
      boolText(enable_joint_candidate_subscription_).c_str());
  }

  void initialize_move_group()
  {
    moveit::planning_interface::MoveGroupInterface::Options options(
      planning_group_, "robot_description");
    move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
      shared_from_this(), options);

    move_group_->setPlanningTime(planning_time_);
    move_group_->setNumPlanningAttempts(std::max(1, num_planning_attempts_));
    move_group_->setMaxVelocityScalingFactor(clamp01(max_velocity_scaling_factor_));
    move_group_->setMaxAccelerationScalingFactor(clamp01(max_acceleration_scaling_factor_));
    move_group_->setGoalJointTolerance(std::max(1e-5, goal_joint_tolerance_));
    move_group_->setGoalPositionTolerance(std::max(1e-4, goal_position_tolerance_));
    move_group_->setGoalOrientationTolerance(std::max(1e-4, goal_orientation_tolerance_));

    publishStatus("ready", "MoveGroup initialized");
    RCLCPP_INFO(
      this->get_logger(),
      "MoveGroup initialized for planning group '%s'.",
      planning_group_.c_str());
    RCLCPP_INFO(
      this->get_logger(),
      "MoveGroup end effector link: %s",
      move_group_->getEndEffectorLink().c_str());
  }

private:
  static std::string boolText(bool value)
  {
    return value ? "true" : "false";
  }

  static double clamp01(double value)
  {
    if (value < 0.0) {
      return 0.0;
    }
    if (value > 1.0) {
      return 1.0;
    }
    return value;
  }

  void onTargetPoseStamped(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    if (!msg) {
      return;
    }

    geometry_msgs::msg::PoseStamped target = *msg;
    if (target.header.frame_id.empty()) {
      target.header.frame_id = base_frame_;
    }
    tryExecuteSingleShot(target);
  }

  void onJointTarget(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    if (!msg) {
      return;
    }
    if (msg->name.empty() || msg->position.empty() || msg->name.size() != msg->position.size()) {
      publishStatus("error", "Invalid joint target message");
      return;
    }

    tryExecuteJointTarget(*msg);
  }

  void onJointCandidates(const trajectory_msgs::msg::JointTrajectory::SharedPtr msg)
  {
    if (!msg) {
      return;
    }
    if (msg->joint_names.empty() || msg->points.empty()) {
      publishStatus("error", "Invalid joint candidate pool");
      return;
    }

    tryExecuteJointCandidates(*msg);
  }

  void tryExecuteSingleShot(const geometry_msgs::msg::PoseStamped & target)
  {
    std::lock_guard<std::mutex> lock(exec_mutex_);

    if (!move_group_) {
      publishStatus("error", "MoveGroup not initialized");
      return;
    }

    if (executing_) {
      // 单次定位测试模式不保留 pending 目标；执行中收到的新目标直接忽略。
      RCLCPP_INFO(this->get_logger(), "Executor busy, ignore new target in single-shot mode");
      return;
    }

    executing_ = true;
    std::thread(&VisionMoveItExecutor::executePoseTarget, this, target).detach();
  }

  void tryExecuteJointTarget(const sensor_msgs::msg::JointState & target)
  {
    std::lock_guard<std::mutex> lock(exec_mutex_);

    if (!move_group_) {
      publishStatus("error", "MoveGroup not initialized");
      return;
    }

    if (executing_) {
      RCLCPP_INFO(this->get_logger(), "Executor busy, ignore new joint target in single-shot mode");
      return;
    }

    executing_ = true;
    std::thread(&VisionMoveItExecutor::executeJointTarget, this, target).detach();
  }

  void tryExecuteJointCandidates(const trajectory_msgs::msg::JointTrajectory & candidates)
  {
    std::lock_guard<std::mutex> lock(exec_mutex_);

    if (!move_group_) {
      publishStatus("error", "MoveGroup not initialized");
      return;
    }

    if (executing_) {
      RCLCPP_INFO(this->get_logger(), "Executor busy, ignore new joint candidate pool");
      return;
    }

    executing_ = true;
    std::thread(&VisionMoveItExecutor::executeJointCandidates, this, candidates).detach();
  }

  bool planAndExecute(const geometry_msgs::msg::PoseStamped & target, std::string & detail)
  {
    // 最简链路：对单个目标直接 plan + execute，不做候选扩展、排队和二次修正。
    move_group_->clearPoseTargets();
    move_group_->clearPathConstraints();
    move_group_->setStartStateToCurrentState();
    move_group_->setGoalJointTolerance(std::max(1e-5, goal_joint_tolerance_));
    move_group_->setGoalOrientationTolerance(std::max(1e-4, goal_orientation_tolerance_));
    move_group_->setPoseTarget(target);

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    const auto plan_ret = move_group_->plan(plan);
    if (plan_ret != moveit::core::MoveItErrorCode::SUCCESS) {
      detail = "Plan failed";
      return false;
    }

    publishStatus("executing", "Plan success, start execution");
    const auto exec_ret = move_group_->execute(plan);
    if (exec_ret != moveit::core::MoveItErrorCode::SUCCESS) {
      detail = "Execution failed";
      return false;
    }

    detail = "Execution success";
    return true;
  }

  bool planAndExecuteJoint(const sensor_msgs::msg::JointState & target, std::string & detail)
  {
    move_group_->clearPoseTargets();
    move_group_->clearPathConstraints();
    move_group_->setStartStateToCurrentState();
    move_group_->setGoalJointTolerance(std::max(1e-5, goal_joint_tolerance_));

    if (!move_group_->setJointValueTarget(target)) {
      detail = "Joint target rejected by MoveGroup";
      return false;
    }

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    const auto plan_ret = move_group_->plan(plan);
    if (plan_ret != moveit::core::MoveItErrorCode::SUCCESS) {
      detail = "Joint plan failed";
      return false;
    }

    publishStatus("executing", "Joint plan success, start execution");
    const auto exec_ret = move_group_->execute(plan);
    if (exec_ret != moveit::core::MoveItErrorCode::SUCCESS) {
      detail = "Joint execution failed";
      return false;
    }

    detail = "Joint execution success";
    return true;
  }

  bool planAndExecuteJointCandidates(
    const trajectory_msgs::msg::JointTrajectory & candidates,
    std::string & detail)
  {
    int failed_count = 0;
    std::string last_failure;
    const int total_count = static_cast<int>(candidates.points.size());

    for (int i = 0; i < total_count; ++i) {
      const auto & point = candidates.points[static_cast<std::size_t>(i)];
      if (point.positions.size() != candidates.joint_names.size()) {
        ++failed_count;
        last_failure = "candidate " + std::to_string(i) + " joint size mismatch";
        continue;
      }

      sensor_msgs::msg::JointState target;
      target.header.stamp = this->now();
      target.header.frame_id = candidates.header.frame_id.empty() ? base_frame_ :
        candidates.header.frame_id;
      target.name = candidates.joint_names;
      target.position = point.positions;

      move_group_->clearPoseTargets();
      move_group_->clearPathConstraints();
      move_group_->setStartStateToCurrentState();
      move_group_->setGoalJointTolerance(std::max(1e-5, goal_joint_tolerance_));

      if (!move_group_->setJointValueTarget(target)) {
        ++failed_count;
        last_failure = "candidate " + std::to_string(i) + " rejected by MoveGroup";
        continue;
      }

      moveit::planning_interface::MoveGroupInterface::Plan plan;
      const auto plan_ret = move_group_->plan(plan);
      if (plan_ret != moveit::core::MoveItErrorCode::SUCCESS) {
        ++failed_count;
        last_failure = "candidate " + std::to_string(i) + " plan failed";
        continue;
      }

      publishStatus(
        "executing",
        "Joint candidate " + std::to_string(i) + "/" + std::to_string(total_count) +
        " plan success, start execution");
      const auto exec_ret = move_group_->execute(plan);
      if (exec_ret != moveit::core::MoveItErrorCode::SUCCESS) {
        detail = "Joint candidate " + std::to_string(i) + " execution failed";
        return false;
      }

      detail =
        "Joint candidate " + std::to_string(i) + " execution success after " +
        std::to_string(failed_count) + " failed plan attempts";
      return true;
    }

    detail =
      "all_candidates_plan_failed total=" + std::to_string(total_count) +
      " failed=" + std::to_string(failed_count) +
      " last=" + last_failure;
    return false;
  }

  void executePoseTarget(geometry_msgs::msg::PoseStamped target)
  {
    publishStatus("planning", "Start pose planning");

    bool ok = false;
    std::string message;
    {
      std::lock_guard<std::mutex> lock(move_group_mutex_);
      ok = planAndExecute(target, message);
      move_group_->clearPoseTargets();
      move_group_->clearPathConstraints();
      move_group_->setGoalJointTolerance(std::max(1e-5, goal_joint_tolerance_));
      move_group_->setGoalOrientationTolerance(std::max(1e-4, goal_orientation_tolerance_));
    }

    if (!ok && message.empty()) {
      message = "Plan failed";
    }

    publishStatus(ok ? "success" : "failed", message);

    {
      std::lock_guard<std::mutex> lock(exec_mutex_);
      executing_ = false;
    }
  }

  void executeJointTarget(sensor_msgs::msg::JointState target)
  {
    publishStatus("planning", "Start joint planning");

    bool ok = false;
    std::string message;
    {
      std::lock_guard<std::mutex> lock(move_group_mutex_);
      ok = planAndExecuteJoint(target, message);
      move_group_->clearPoseTargets();
      move_group_->clearPathConstraints();
      move_group_->setGoalJointTolerance(std::max(1e-5, goal_joint_tolerance_));
      move_group_->setGoalOrientationTolerance(std::max(1e-4, goal_orientation_tolerance_));
    }

    if (!ok && message.empty()) {
      message = "Joint plan failed";
    }

    publishStatus(ok ? "success" : "failed", message);

    {
      std::lock_guard<std::mutex> lock(exec_mutex_);
      executing_ = false;
    }
  }

  void executeJointCandidates(trajectory_msgs::msg::JointTrajectory candidates)
  {
    publishStatus(
      "planning",
      "Start joint candidate planning count=" + std::to_string(candidates.points.size()));

    bool ok = false;
    std::string message;
    {
      std::lock_guard<std::mutex> lock(move_group_mutex_);
      ok = planAndExecuteJointCandidates(candidates, message);
      move_group_->clearPoseTargets();
      move_group_->clearPathConstraints();
      move_group_->setGoalJointTolerance(std::max(1e-5, goal_joint_tolerance_));
      move_group_->setGoalOrientationTolerance(std::max(1e-4, goal_orientation_tolerance_));
    }

    if (!ok && message.empty()) {
      message = "all_candidates_plan_failed";
    }

    publishStatus(ok ? "success" : "failed", message);

    {
      std::lock_guard<std::mutex> lock(exec_mutex_);
      executing_ = false;
    }
  }

  void publishStatus(const std::string & state, const std::string & detail)
  {
    std_msgs::msg::String msg;
    msg.data = "{\"state\":\"" + state + "\",\"detail\":\"" + detail + "\"}";
    status_pub_->publish(msg);
    RCLCPP_INFO(this->get_logger(), "[vision_exec] %s: %s", state.c_str(), detail.c_str());
  }

  std::string planning_group_;
  std::string base_frame_;
  std::string target_pose_stamped_topic_;
  std::string joint_target_topic_;
  std::string joint_candidate_topic_;

  bool enable_pose_target_subscription_ {false};
  bool enable_joint_target_subscription_ {false};
  bool enable_joint_candidate_subscription_ {true};

  double planning_time_ {3.0};
  int num_planning_attempts_ {3};
  double max_velocity_scaling_factor_ {0.25};
  double max_acceleration_scaling_factor_ {0.25};
  double goal_joint_tolerance_ {0.005};
  double goal_position_tolerance_ {0.01};
  double goal_orientation_tolerance_ {0.6};

  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr target_pose_stamped_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_target_sub_;
  rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr joint_candidate_sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;

  std::mutex move_group_mutex_;
  std::mutex exec_mutex_;
  bool executing_ {false};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<VisionMoveItExecutor>();
  node->initialize_move_group();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
