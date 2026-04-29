#include <memory>
#include <string>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2/exceptions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

class TfCompatBroadcasterNode : public rclcpp::Node
{
public:
  TfCompatBroadcasterNode()
  : Node("tf_compat_broadcaster")
  {
    this->declare_parameter("enable_tf_compat", false);
    this->declare_parameter("compat_parent_frame", "base_link");
    this->declare_parameter("source_ee_frame", "Joint6");
    this->declare_parameter("compat_child_frame", "Joint6_compat");
    this->declare_parameter("publish_rate_hz", 30.0);

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    const double rate_hz = std::max(1.0, this->get_parameter("publish_rate_hz").as_double());
    const auto period = std::chrono::duration<double>(1.0 / rate_hz);
    timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::milliseconds>(period),
      std::bind(&TfCompatBroadcasterNode::onTimer, this));

    RCLCPP_INFO(this->get_logger(), "tf_compat_broadcaster started (enable_tf_compat=%s)",
      this->get_parameter("enable_tf_compat").as_bool() ? "true" : "false");
  }

private:
  void onTimer()
  {
    const bool enabled = this->get_parameter("enable_tf_compat").as_bool();
    if (!enabled) {
      return;
    }

    const std::string parent = this->get_parameter("compat_parent_frame").as_string();
    const std::string source = this->get_parameter("source_ee_frame").as_string();
    const std::string child = this->get_parameter("compat_child_frame").as_string();

    if (parent.empty() || source.empty() || child.empty()) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 5000,
        "TF compat is enabled but one or more frame parameters are empty.");
      return;
    }

    if (child == source) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 5000,
        "compat_child_frame equals source_ee_frame ('%s'). Skip to avoid duplicate TF publisher.",
        child.c_str());
      return;
    }

    try {
      const auto src_tf = tf_buffer_->lookupTransform(parent, source, tf2::TimePointZero);

      geometry_msgs::msg::TransformStamped out_tf;
      out_tf.header.stamp = this->now();
      out_tf.header.frame_id = parent;
      out_tf.child_frame_id = child;
      out_tf.transform = src_tf.transform;

      tf_broadcaster_->sendTransform(out_tf);
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "TF compat lookup failed (%s -> %s): %s",
        parent.c_str(), source.c_str(), ex.what());
    }
  }

  rclcpp::TimerBase::SharedPtr timer_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TfCompatBroadcasterNode>());
  rclcpp::shutdown();
  return 0;
}
