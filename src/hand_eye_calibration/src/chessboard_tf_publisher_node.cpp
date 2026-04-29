#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>

#include <Eigen/Geometry>

#include <tf2_ros/transform_broadcaster.h>

class ChessboardTfPublisherNode : public rclcpp::Node
{
public:
  ChessboardTfPublisherNode()
  : Node("chessboard_tf_publisher_node")
  {
    declare_parameter<std::string>("image_topic", "/camera/camera/color/image_raw");
    declare_parameter<std::string>("camera_info_topic", "/camera/camera/color/camera_info");
    declare_parameter<std::string>("camera_frame", "");
    declare_parameter<std::string>("board_frame", "calib_board");
    declare_parameter<int>("pattern_rows", 8);
    declare_parameter<int>("pattern_cols", 8);
    declare_parameter<double>("square_size_m", 0.033);
    declare_parameter<std::string>("pose_topic", "/calib_board_pose");
    declare_parameter<bool>("use_ros_now_stamp", true);

    image_topic_ = get_parameter("image_topic").as_string();
    camera_info_topic_ = get_parameter("camera_info_topic").as_string();
    camera_frame_ = get_parameter("camera_frame").as_string();
    board_frame_ = get_parameter("board_frame").as_string();
    pose_topic_ = get_parameter("pose_topic").as_string();
    pattern_rows_ = get_parameter("pattern_rows").as_int();
    pattern_cols_ = get_parameter("pattern_cols").as_int();
    square_size_m_ = get_parameter("square_size_m").as_double();
    use_ros_now_stamp_ = get_parameter("use_ros_now_stamp").as_bool();

    if (pattern_rows_ <= 0 || pattern_cols_ <= 0 || square_size_m_ <= 0.0) {
      throw std::runtime_error("Invalid chessboard parameters: rows/cols > 0 and square_size_m > 0 are required");
    }

    board_points_ = createBoardPoints(pattern_rows_, pattern_cols_, square_size_m_);

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(pose_topic_, 10);

    camera_info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
      camera_info_topic_,
      rclcpp::SensorDataQoS(),
      std::bind(&ChessboardTfPublisherNode::cameraInfoCallback, this, std::placeholders::_1));

    image_sub_ = create_subscription<sensor_msgs::msg::Image>(
      image_topic_,
      rclcpp::SensorDataQoS(),
      std::bind(&ChessboardTfPublisherNode::imageCallback, this, std::placeholders::_1));

    RCLCPP_INFO(
      get_logger(),
      "Started chessboard tf publisher. image=%s, camera_info=%s, pattern=%dx%d, square=%.4fm, tf=%s->%s",
      image_topic_.c_str(),
      camera_info_topic_.c_str(),
      pattern_rows_,
      pattern_cols_,
      square_size_m_,
      camera_frame_.empty() ? "<auto from camera_info.header.frame_id>" : camera_frame_.c_str(),
      board_frame_.c_str());
    RCLCPP_INFO(get_logger(), "TF timestamp mode: %s", use_ros_now_stamp_ ? "node_clock_now" : "image_header_stamp");
  }

private:
  std::vector<cv::Point3f> createBoardPoints(int rows, int cols, double square_size) const
  {
    std::vector<cv::Point3f> points;
    points.reserve(static_cast<size_t>(rows * cols));

    for (int r = 0; r < rows; ++r) {
      for (int c = 0; c < cols; ++c) {
        points.emplace_back(
          static_cast<float>(c * square_size),
          static_cast<float>(r * square_size),
          0.0F);
      }
    }
    return points;
  }

  void cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
  {
    if (msg->k.size() != 9U) {
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        2000,
        "CameraInfo K matrix size is invalid: %zu",
        msg->k.size());
      return;
    }

    camera_matrix_ = cv::Mat::eye(3, 3, CV_64F);
    camera_matrix_.at<double>(0, 0) = msg->k[0];
    camera_matrix_.at<double>(0, 1) = msg->k[1];
    camera_matrix_.at<double>(0, 2) = msg->k[2];
    camera_matrix_.at<double>(1, 0) = msg->k[3];
    camera_matrix_.at<double>(1, 1) = msg->k[4];
    camera_matrix_.at<double>(1, 2) = msg->k[5];
    camera_matrix_.at<double>(2, 0) = msg->k[6];
    camera_matrix_.at<double>(2, 1) = msg->k[7];
    camera_matrix_.at<double>(2, 2) = msg->k[8];

    if (msg->d.empty()) {
      dist_coeffs_ = cv::Mat::zeros(1, 5, CV_64F);
    } else {
      dist_coeffs_ = cv::Mat::zeros(1, static_cast<int>(msg->d.size()), CV_64F);
      for (size_t i = 0; i < msg->d.size(); ++i) {
        dist_coeffs_.at<double>(0, static_cast<int>(i)) = msg->d[i];
      }
    }

    if (camera_frame_.empty()) {
      camera_frame_ = msg->header.frame_id;
      RCLCPP_INFO(get_logger(), "camera_frame resolved from camera_info: %s", camera_frame_.c_str());
    }
    has_camera_info_ = true;
  }

  void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    if (!has_camera_info_) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Waiting for camera_info...");
      return;
    }

    cv_bridge::CvImageConstPtr cv_ptr;
    try {
      cv_ptr = cv_bridge::toCvShare(msg, msg->encoding);
    } catch (const cv_bridge::Exception & e) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "cv_bridge conversion failed: %s", e.what());
      return;
    }

    cv::Mat gray;
    if (cv_ptr->image.channels() == 3) {
      cv::cvtColor(cv_ptr->image, gray, cv::COLOR_BGR2GRAY);
    } else if (cv_ptr->image.channels() == 1) {
      gray = cv_ptr->image;
    } else {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "Unsupported image channels: %d", cv_ptr->image.channels());
      return;
    }

    std::vector<cv::Point2f> corners;
    const bool found = cv::findChessboardCorners(
      gray,
      cv::Size(pattern_cols_, pattern_rows_),
      corners,
      cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE);

    if (!found) {
      RCLCPP_DEBUG_THROTTLE(get_logger(), *get_clock(), 1000, "Chessboard not found in current frame");
      return;
    }

    cv::cornerSubPix(
      gray,
      corners,
      cv::Size(11, 11),
      cv::Size(-1, -1),
      cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 30, 1e-3));

    cv::Mat rvec;
    cv::Mat tvec;
    const bool pnp_ok = cv::solvePnP(
      board_points_,
      corners,
      camera_matrix_,
      dist_coeffs_,
      rvec,
      tvec,
      false,
      cv::SOLVEPNP_ITERATIVE);

    if (!pnp_ok) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "solvePnP failed");
      return;
    }

    cv::Mat rotation_cv;
    cv::Rodrigues(rvec, rotation_cv);

    Eigen::Matrix3d rotation_eigen = Eigen::Matrix3d::Identity();
    for (int r = 0; r < 3; ++r) {
      for (int c = 0; c < 3; ++c) {
        rotation_eigen(r, c) = rotation_cv.at<double>(r, c);
      }
    }
    Eigen::Quaterniond q(rotation_eigen);
    q.normalize();

    geometry_msgs::msg::TransformStamped tf_msg;
    if (use_ros_now_stamp_) {
      tf_msg.header.stamp = this->get_clock()->now();
    } else {
      tf_msg.header.stamp = msg->header.stamp;
    }
    tf_msg.header.frame_id = camera_frame_;
    tf_msg.child_frame_id = board_frame_;
    tf_msg.transform.translation.x = tvec.at<double>(0, 0);
    tf_msg.transform.translation.y = tvec.at<double>(1, 0);
    tf_msg.transform.translation.z = tvec.at<double>(2, 0);
    tf_msg.transform.rotation.x = q.x();
    tf_msg.transform.rotation.y = q.y();
    tf_msg.transform.rotation.z = q.z();
    tf_msg.transform.rotation.w = q.w();
    tf_broadcaster_->sendTransform(tf_msg);

    geometry_msgs::msg::PoseStamped pose_msg;
    pose_msg.header = tf_msg.header;
    pose_msg.pose.position.x = tf_msg.transform.translation.x;
    pose_msg.pose.position.y = tf_msg.transform.translation.y;
    pose_msg.pose.position.z = tf_msg.transform.translation.z;
    pose_msg.pose.orientation = tf_msg.transform.rotation;
    pose_pub_->publish(pose_msg);
  }

  std::string image_topic_;
  std::string camera_info_topic_;
  std::string camera_frame_;
  std::string board_frame_;
  std::string pose_topic_;

  int pattern_rows_ = 8;
  int pattern_cols_ = 8;
  double square_size_m_ = 0.033;
  bool use_ros_now_stamp_ = true;

  bool has_camera_info_ = false;
  cv::Mat camera_matrix_;
  cv::Mat dist_coeffs_;
  std::vector<cv::Point3f> board_points_;

  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ChessboardTfPublisherNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
