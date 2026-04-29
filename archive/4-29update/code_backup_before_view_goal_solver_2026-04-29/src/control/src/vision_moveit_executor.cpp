#include <algorithm>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

class VisionMoveItExecutor : public rclcpp::Node
{
public:
  VisionMoveItExecutor()
  : Node("vision_moveit_executor")
  {
    this->declare_parameter("planning_group", "arm");
    this->declare_parameter("base_frame", "base_link");
    this->declare_parameter("target_pose_stamped_topic", "/target_pose_stamped");
    this->declare_parameter("status_topic", "/vision_exec_status");
    this->declare_parameter("planning_time", 3.0);
    this->declare_parameter("num_planning_attempts", 3);
    this->declare_parameter("max_velocity_scaling_factor", 0.25);
    this->declare_parameter("max_acceleration_scaling_factor", 0.25);
    this->declare_parameter("goal_position_tolerance", 0.01);
    this->declare_parameter("goal_orientation_tolerance", 0.6);

    planning_group_ = this->get_parameter("planning_group").as_string();
    base_frame_ = this->get_parameter("base_frame").as_string();
    target_pose_stamped_topic_ = this->get_parameter("target_pose_stamped_topic").as_string();
    planning_time_ = this->get_parameter("planning_time").as_double();
    num_planning_attempts_ = std::max(
      1, static_cast<int>(this->get_parameter("num_planning_attempts").as_int()));
    max_velocity_scaling_factor_ = this->get_parameter("max_velocity_scaling_factor").as_double();
    max_acceleration_scaling_factor_ = this->get_parameter("max_acceleration_scaling_factor").as_double();
    goal_position_tolerance_ = this->get_parameter("goal_position_tolerance").as_double();
    goal_orientation_tolerance_ = this->get_parameter("goal_orientation_tolerance").as_double();

    status_pub_ = this->create_publisher<std_msgs::msg::String>(
      this->get_parameter("status_topic").as_string(), 10);

    target_pose_stamped_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      target_pose_stamped_topic_, 10,
      std::bind(&VisionMoveItExecutor::onTargetPoseStamped, this, std::placeholders::_1));

    RCLCPP_INFO(
      this->get_logger(),
      "vision_moveit_executor created. group=%s, target_pose_stamped=%s",
      planning_group_.c_str(),
      target_pose_stamped_topic_.c_str());
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
    std::thread(&VisionMoveItExecutor::executeTarget, this, target).detach();
  }

  bool planAndExecute(const geometry_msgs::msg::PoseStamped & target, std::string & detail)
  {
    // 最简链路：对单个目标直接 plan + execute，不做候选扩展、排队和二次修正。
    move_group_->clearPoseTargets();
    move_group_->clearPathConstraints();
    move_group_->setStartStateToCurrentState();
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

  void executeTarget(geometry_msgs::msg::PoseStamped target)
  {
    publishStatus("planning", "Start planning");

    bool ok = false;
    std::string message;
    {
      std::lock_guard<std::mutex> lock(move_group_mutex_);
      ok = planAndExecute(target, message);
      move_group_->clearPoseTargets();
      move_group_->clearPathConstraints();
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

  double planning_time_ {3.0};
  int num_planning_attempts_ {3};
  double max_velocity_scaling_factor_ {0.25};
  double max_acceleration_scaling_factor_ {0.25};
  double goal_position_tolerance_ {0.01};
  double goal_orientation_tolerance_ {0.6};

  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr target_pose_stamped_sub_;
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
