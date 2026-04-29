#include <memory>
#include <cmath>
#include <algorithm>
#include <numeric>
#include <deque>
#include <sstream>
#include <iomanip>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <cv_bridge/cv_bridge.h>
#include <hand_eye_calibration/calibration_utils.hpp>

using namespace hand_eye_calibration;

/**
 * @brief 手眼标定节点
 * 
 * 功能：
 * - 订阅机械臂末端位姿 (/tool_pose)
 * - 订阅相机图像 (/camera/color/image_raw)
 * - 同步采集标定样本
 * - 手动触发计算手眼标定
 * - 发布标定结果
 */
class HandEyeCalibrationNode : public rclcpp::Node {
public:
    HandEyeCalibrationNode() : Node("hand_eye_calibration_node")
    {
        // 参数声明（允许从 YAML 加载）
        this->declare_parameter<int>("checkerboard_rows", 6);
        this->declare_parameter<int>("checkerboard_cols", 9);
        this->declare_parameter<double>("square_size_mm", 30.0);
        this->declare_parameter<double>("camera_fx", 906.948);
        this->declare_parameter<double>("camera_fy", 905.906);
        this->declare_parameter<double>("camera_cx", 648.379);
        this->declare_parameter<double>("camera_cy", 383.873);
        this->declare_parameter<int>("min_samples", 10);
        this->declare_parameter<std::string>("output_file", "/tmp/hand_eye_calibration_result.yaml");
        this->declare_parameter<double>("max_pose_age_sec", 1.0);
        this->declare_parameter<double>("max_image_age_sec", 1.0);
        this->declare_parameter<int>("board_margin_px", 20);
        this->declare_parameter<double>("stability_window_sec", 2.0);
        this->declare_parameter<double>("stable_pos_thresh_m", 0.0012);
        this->declare_parameter<double>("stable_rot_thresh_deg", 0.2);
        this->declare_parameter<int>("min_stability_samples", 8);
        this->declare_parameter<bool>("require_motion_after_target", true);
        this->declare_parameter<double>("motion_start_pos_thresh_m", 0.0015);
        this->declare_parameter<double>("motion_start_rot_thresh_deg", 0.2);
        this->declare_parameter<double>("motion_start_timeout_sec", 2.0);
        
        // 获取参数（从 yaml 配置文件加载）
        auto rows_param = this->get_parameter("checkerboard_rows");
        auto cols_param = this->get_parameter("checkerboard_cols");
        auto square_size_param = this->get_parameter("square_size_mm");
        auto fx_param = this->get_parameter("camera_fx");
        auto fy_param = this->get_parameter("camera_fy");
        auto cx_param = this->get_parameter("camera_cx");
        auto cy_param = this->get_parameter("camera_cy");
        auto min_samples_param = this->get_parameter("min_samples");
        auto output_file_param = this->get_parameter("output_file");
        auto max_pose_age_param = this->get_parameter("max_pose_age_sec");
        auto max_image_age_param = this->get_parameter("max_image_age_sec");
        auto board_margin_param = this->get_parameter("board_margin_px");
        auto stability_window_param = this->get_parameter("stability_window_sec");
        auto stable_pos_thresh_param = this->get_parameter("stable_pos_thresh_m");
        auto stable_rot_thresh_param = this->get_parameter("stable_rot_thresh_deg");
        auto min_stability_samples_param = this->get_parameter("min_stability_samples");
        auto require_motion_after_target_param = this->get_parameter("require_motion_after_target");
        auto motion_start_pos_thresh_param = this->get_parameter("motion_start_pos_thresh_m");
        auto motion_start_rot_thresh_param = this->get_parameter("motion_start_rot_thresh_deg");
        auto motion_start_timeout_param = this->get_parameter("motion_start_timeout_sec");
        
        int rows = rows_param.as_int();
        int cols = cols_param.as_int();
        double square_size = square_size_param.as_double() / 1000.0;  // mm -> m
        
        double fx = fx_param.as_double();
        double fy = fy_param.as_double();
        double cx = cx_param.as_double();
        double cy = cy_param.as_double();
        
        min_samples_ = min_samples_param.as_int();
        output_file_ = output_file_param.as_string();
        max_pose_age_sec_ = max_pose_age_param.as_double();
        max_image_age_sec_ = max_image_age_param.as_double();
        board_margin_px_ = board_margin_param.as_int();
        stability_window_sec_ = std::max(0.1, stability_window_param.as_double());
        stable_pos_thresh_m_ = std::max(0.0, stable_pos_thresh_param.as_double());
        stable_rot_thresh_deg_ = std::max(0.0, stable_rot_thresh_param.as_double());
        min_stability_samples_ = std::max(3, static_cast<int>(min_stability_samples_param.as_int()));
        require_motion_after_target_ = require_motion_after_target_param.as_bool();
        motion_start_pos_thresh_m_ = std::max(0.0, motion_start_pos_thresh_param.as_double());
        motion_start_rot_thresh_deg_ = std::max(0.0, motion_start_rot_thresh_param.as_double());
        motion_start_timeout_sec_ = std::max(0.0, motion_start_timeout_param.as_double());
        
        // 初始化棋盘检测器
        detector_ = std::make_shared<CheckerboardDetector>(fx, fy, cx, cy);
        detector_->setCheckerboardSize(rows, cols, square_size);
        
        // 初始化求解器
        solver_ = std::make_shared<TsaiLenzSolver>();
        
        // 订阅
        // 方式1：订阅 /target_pose (Pose 类型，推荐用于自动采集)
        target_pose_sub_ = this->create_subscription<geometry_msgs::msg::Pose>(
            "/target_pose", 10,
            std::bind(&HandEyeCalibrationNode::targetPoseCallback, this, std::placeholders::_1));
        
        // 方式2：订阅 /tool_pose (PoseStamped 类型，备选)
        arm_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/tool_pose", 10,
            std::bind(&HandEyeCalibrationNode::armPoseCallback, this, std::placeholders::_1));
        
        camera_image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera/camera/color/image_raw", 10,
            std::bind(&HandEyeCalibrationNode::cameraImageCallback, this, std::placeholders::_1));
        
        // 发布
        calibration_pub_ = this->create_publisher<geometry_msgs::msg::TransformStamped>(
            "calibration_result", 10);
        
        // 服务：手动采集单个样本
        calibration_capture_srv_ = this->create_service<std_srvs::srv::Trigger>(
            "/calibration/capture",
            std::bind(&HandEyeCalibrationNode::captureCallback, this, std::placeholders::_1, std::placeholders::_2));

        calibration_is_stable_srv_ = this->create_service<std_srvs::srv::Trigger>(
            "/calibration/is_stable",
            std::bind(&HandEyeCalibrationNode::isStableCallback, this, std::placeholders::_1, std::placeholders::_2));

        calibration_test_board_pose_srv_ = this->create_service<std_srvs::srv::Trigger>(
            "/calibration/test_board_pose",
            std::bind(&HandEyeCalibrationNode::testBoardPoseCallback, this, std::placeholders::_1, std::placeholders::_2));

        calibration_load_result_srv_ = this->create_service<std_srvs::srv::Trigger>(
            "/calibration/load_result",
            std::bind(&HandEyeCalibrationNode::loadResultCallback, this, std::placeholders::_1, std::placeholders::_2));

        // 服务：手动触发标定计算
        calibration_compute_srv_ = this->create_service<std_srvs::srv::Trigger>(
            "/calibration/compute",
            std::bind(&HandEyeCalibrationNode::computeCallback, this, std::placeholders::_1, std::placeholders::_2));
        
        RCLCPP_INFO(this->get_logger(), "Manual capture service available at /calibration/capture");
        RCLCPP_INFO(this->get_logger(), "Stability query service available at /calibration/is_stable");
        RCLCPP_INFO(this->get_logger(), "Board pose test service available at /calibration/test_board_pose");
        RCLCPP_INFO(this->get_logger(), "Load calibration service available at /calibration/load_result");
        RCLCPP_INFO(this->get_logger(), "Manual compute service available at /calibration/compute");
        RCLCPP_INFO(this->get_logger(),
            "Checkerboard: %dx%d, %.1f mm, Camera: fx=%.2f, fy=%.2f",
            rows, cols, square_size * 1000, fx, fy);
        RCLCPP_INFO(this->get_logger(),
            "Sample guards: max_pose_age=%.2fs, max_image_age=%.2fs, board_margin=%dpx",
            max_pose_age_sec_, max_image_age_sec_, board_margin_px_);
        RCLCPP_INFO(this->get_logger(),
            "Stability gate: window=%.2fs, pos<=%.4fm, rot<=%.3fdeg, min_samples=%d",
            stability_window_sec_, stable_pos_thresh_m_, stable_rot_thresh_deg_, min_stability_samples_);
        RCLCPP_INFO(this->get_logger(),
            "Motion-start gate: require_after_target=%s, start_pos>=%.4fm, start_rot>=%.3fdeg, timeout=%.2fs",
            require_motion_after_target_ ? "true" : "false",
            motion_start_pos_thresh_m_, motion_start_rot_thresh_deg_, motion_start_timeout_sec_);
    }

private:
    std::shared_ptr<CheckerboardDetector> detector_;
    std::shared_ptr<TsaiLenzSolver> solver_;
    
    std::vector<CalibrationSample> samples_;
    geometry_msgs::msg::PoseStamped::SharedPtr latest_arm_pose_;
    sensor_msgs::msg::Image::SharedPtr latest_image_;
    geometry_msgs::msg::Pose::SharedPtr latest_target_pose_;
    
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr arm_pose_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr target_pose_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr camera_image_sub_;
    rclcpp::Publisher<geometry_msgs::msg::TransformStamped>::SharedPtr calibration_pub_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr calibration_capture_srv_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr calibration_is_stable_srv_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr calibration_test_board_pose_srv_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr calibration_load_result_srv_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr calibration_compute_srv_;
    
    int min_samples_;
    std::string output_file_;
    double max_pose_age_sec_;
    double max_image_age_sec_;
    int board_margin_px_;
    double stability_window_sec_;
    double stable_pos_thresh_m_;
    double stable_rot_thresh_deg_;
    int min_stability_samples_;
    bool require_motion_after_target_;
    double motion_start_pos_thresh_m_;
    double motion_start_rot_thresh_deg_;
    double motion_start_timeout_sec_;

    struct TimedPose {
        rclcpp::Time stamp;
        Eigen::Isometry3d pose;
    };
    std::deque<TimedPose> tool_pose_history_;
    bool target_pose_received_ = false;
    bool motion_observed_since_target_ = false;
    bool have_pose_snapshot_at_target_ = false;
    rclcpp::Time last_target_pose_time_{0, 0, RCL_ROS_TIME};
    Eigen::Isometry3d pose_snapshot_at_target_ = Eigen::Isometry3d::Identity();
    CalibrationResult last_calibration_result_;
    bool has_calibration_result_ = false;

    double rotationDistanceRad(const Eigen::Matrix3d& R_a, const Eigen::Matrix3d& R_b)
    {
        const Eigen::Matrix3d dR = R_a.transpose() * R_b;
        const double trace_val = dR.trace();
        const double cos_theta = std::clamp((trace_val - 1.0) * 0.5, -1.0, 1.0);
        return std::acos(cos_theta);
    }

    struct ConsistencyStats {
        Eigen::Vector3d pos_mean = Eigen::Vector3d::Zero();
        Eigen::Vector3d pos_std = Eigen::Vector3d::Zero();
        double pos_max_radius = 0.0;
        double rot_mean_deg = 0.0;
        double rot_std_deg = 0.0;
        double rot_max_deg = 0.0;
    };

    ConsistencyStats evaluateBoardConsistency(
        const std::vector<CalibrationSample>& samples,
        const Eigen::Isometry3d& T_tool_cam)
    {
        ConsistencyStats stats;
        if (samples.empty()) {
            return stats;
        }

        std::vector<Eigen::Isometry3d> board_poses;
        board_poses.reserve(samples.size());
        for (const auto& s : samples) {
            board_poses.push_back(s.T_base_tool * T_tool_cam * s.T_cam_board);
        }

        for (const auto& T : board_poses) {
            stats.pos_mean += T.translation();
        }
        stats.pos_mean /= static_cast<double>(board_poses.size());

        Eigen::Vector3d pos_var = Eigen::Vector3d::Zero();
        std::vector<double> rot_deg;
        rot_deg.reserve(board_poses.size());
        const Eigen::Matrix3d R_ref = board_poses.front().rotation();

        for (const auto& T : board_poses) {
            const Eigen::Vector3d delta = T.translation() - stats.pos_mean;
            pos_var += delta.cwiseProduct(delta);
            stats.pos_max_radius = std::max(stats.pos_max_radius, delta.norm());

            const double deg = rotationDistanceRad(R_ref, T.rotation()) * 180.0 / M_PI;
            rot_deg.push_back(deg);
            stats.rot_max_deg = std::max(stats.rot_max_deg, deg);
        }
        stats.pos_std = (pos_var / static_cast<double>(board_poses.size())).cwiseSqrt();

        if (!rot_deg.empty()) {
            const double sum = std::accumulate(rot_deg.begin(), rot_deg.end(), 0.0);
            stats.rot_mean_deg = sum / static_cast<double>(rot_deg.size());
            double var = 0.0;
            for (double d : rot_deg) {
                const double e = d - stats.rot_mean_deg;
                var += e * e;
            }
            stats.rot_std_deg = std::sqrt(var / static_cast<double>(rot_deg.size()));
        }

        return stats;
    }

    bool isRobotStable(std::string* reason = nullptr, double* out_pos_m = nullptr, double* out_rot_deg = nullptr)
    {
        if (tool_pose_history_.size() < static_cast<size_t>(min_stability_samples_)) {
            if (reason) *reason = "Not enough pose history";
            return false;
        }

        const rclcpp::Time newest_t = tool_pose_history_.back().stamp;
        const rclcpp::Time oldest_allowed = newest_t - rclcpp::Duration::from_seconds(stability_window_sec_);

        // 防止“目标刚发布就判定稳定”
        if (require_motion_after_target_ && target_pose_received_) {
            const double since_target = std::max(0.0, (newest_t - last_target_pose_time_).seconds());
            if (!motion_observed_since_target_ && since_target < motion_start_timeout_sec_) {
                if (reason) {
                    *reason = "Waiting motion start after target update (" + std::to_string(since_target) + "s)";
                }
                return false;
            }
        }

        std::vector<const TimedPose*> window;
        window.reserve(tool_pose_history_.size());
        for (const auto& tp : tool_pose_history_) {
            if (tp.stamp >= oldest_allowed) {
                window.push_back(&tp);
            }
        }

        if (window.size() < static_cast<size_t>(min_stability_samples_)) {
            if (reason) *reason = "Insufficient samples within stability window";
            return false;
        }

        const Eigen::Vector3d p_ref = window.front()->pose.translation();
        const Eigen::Matrix3d R_ref = window.front()->pose.rotation();
        double max_pos = 0.0;
        double max_rot_deg = 0.0;

        for (const auto* tp : window) {
            max_pos = std::max(max_pos, (tp->pose.translation() - p_ref).norm());
            const double rot_rad = rotationDistanceRad(R_ref, tp->pose.rotation());
            max_rot_deg = std::max(max_rot_deg, rot_rad * 180.0 / M_PI);
        }

        if (out_pos_m) *out_pos_m = max_pos;
        if (out_rot_deg) *out_rot_deg = max_rot_deg;

        const bool stable = (max_pos <= stable_pos_thresh_m_) && (max_rot_deg <= stable_rot_thresh_deg_);
        if (!stable && reason) {
            *reason = "Unstable: dpos=" + std::to_string(max_pos) + "m, drot=" + std::to_string(max_rot_deg) + "deg";
        }
        return stable;
    }

    void targetPoseCallback(const geometry_msgs::msg::Pose::SharedPtr msg)
    {
        // 仅用于日志显示，不作为标定采样姿态来源
        // 标定采样必须来自 /tool_pose（真实末端位姿）
        latest_target_pose_ = msg;
        target_pose_received_ = true;
        motion_observed_since_target_ = false;
        last_target_pose_time_ = this->now();
        have_pose_snapshot_at_target_ = false;
        if (latest_arm_pose_) {
            pose_snapshot_at_target_ = poseToIsometry(latest_arm_pose_->pose);
            have_pose_snapshot_at_target_ = true;
        }
        
        // 计算欧拉角用于日志显示
        Eigen::Quaterniond q(msg->orientation.w, msg->orientation.x,
                            msg->orientation.y, msg->orientation.z);
        double roll = std::atan2(2*(q.w()*q.x() + q.y()*q.z()), 1 - 2*(q.x()*q.x() + q.y()*q.y())) * 180 / M_PI;
        double pitch = std::asin(2*(q.w()*q.y() - q.z()*q.x())) * 180 / M_PI;
        double yaw = std::atan2(2*(q.w()*q.z() + q.x()*q.y()), 1 - 2*(q.y()*q.y() + q.z()*q.z())) * 180 / M_PI;
        
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
            "Target pose received: Pos=[%.3f, %.3f, %.3f] Rot=[%.1f°, %.1f°, %.1f°] Quat=[%.3f, %.3f, %.3f, %.3f]",
            msg->position.x, msg->position.y, msg->position.z,
            roll, pitch, yaw,
            q.x(), q.y(), q.z(), q.w());
    }
    
    void armPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        if (!msg->header.frame_id.empty() && msg->header.frame_id != "base_link") {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                "/tool_pose frame_id is '%s' (expected 'base_link').", msg->header.frame_id.c_str());
        }
        latest_arm_pose_ = msg;

        rclcpp::Time stamp = this->now();
        if (!(msg->header.stamp.sec == 0 && msg->header.stamp.nanosec == 0)) {
            stamp = rclcpp::Time(msg->header.stamp);
        }

        TimedPose tp;
        tp.stamp = stamp;
        tp.pose = poseToIsometry(msg->pose);
        tool_pose_history_.push_back(tp);

        if (require_motion_after_target_ && target_pose_received_ && !motion_observed_since_target_ && have_pose_snapshot_at_target_) {
            const double dpos = (tp.pose.translation() - pose_snapshot_at_target_.translation()).norm();
            const double drot_deg = rotationDistanceRad(pose_snapshot_at_target_.rotation(), tp.pose.rotation()) * 180.0 / M_PI;
            if (dpos >= motion_start_pos_thresh_m_ || drot_deg >= motion_start_rot_thresh_deg_) {
                motion_observed_since_target_ = true;
            }
        }

        const rclcpp::Time cutoff = stamp - rclcpp::Duration::from_seconds(stability_window_sec_ + 1.0);
        while (!tool_pose_history_.empty() && tool_pose_history_.front().stamp < cutoff) {
            tool_pose_history_.pop_front();
        }
        while (tool_pose_history_.size() > 400) {
            tool_pose_history_.pop_front();
        }
    }
    
    void cameraImageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        latest_image_ = msg;
    }
    
    void captureCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                        std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        (void)request;
        
        if (!latest_arm_pose_ || !latest_image_) {
            response->success = false;
            response->message = "No /tool_pose or image data available";
            return;
        }

        // 新鲜度检查：避免使用陈旧位姿/图像
        rclcpp::Time now_t = this->now();

        auto pose_stamp = latest_arm_pose_->header.stamp;
        if (pose_stamp.sec == 0 && pose_stamp.nanosec == 0) {
            pose_stamp = now_t;
        }
        rclcpp::Time pose_t(pose_stamp);
        const double pose_age = std::max(0.0, (now_t - pose_t).seconds());
        if (pose_age > max_pose_age_sec_) {
            response->success = false;
            response->message = "Stale /tool_pose: age=" + std::to_string(pose_age) + "s";
            RCLCPP_WARN(this->get_logger(), "%s", response->message.c_str());
            return;
        }

        auto image_stamp = latest_image_->header.stamp;
        if (image_stamp.sec == 0 && image_stamp.nanosec == 0) {
            image_stamp = now_t;
        }
        rclcpp::Time image_t(image_stamp);
        const double image_age = std::max(0.0, (now_t - image_t).seconds());
        if (image_age > max_image_age_sec_) {
            response->success = false;
            response->message = "Stale image: age=" + std::to_string(image_age) + "s";
            RCLCPP_WARN(this->get_logger(), "%s", response->message.c_str());
            return;
        }

        // 机械臂静稳门控：必须在时间窗内位姿变化很小才允许采样
        std::string stable_reason;
        double max_dpos = 0.0;
        double max_drot = 0.0;
        if (!isRobotStable(&stable_reason, &max_dpos, &max_drot)) {
            response->success = false;
            response->message = "Robot not stable: " + stable_reason;
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 500,
                "Capture rejected by stability gate (dpos=%.6fm, drot=%.4fdeg)", max_dpos, max_drot);
            return;
        }
        
        cv_bridge::CvImagePtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvCopy(*latest_image_, sensor_msgs::image_encodings::BGR8);
        } catch (cv_bridge::Exception& e) {
            response->success = false;
            response->message = std::string("Image conversion failed: ") + e.what();
            return;
        }
        
        Eigen::Isometry3d T_cam_board;
        std::vector<cv::Point2f> corners;
        
        if (!detector_->detectAndEstimatePose(cv_ptr->image, T_cam_board, corners)) {
            response->success = false;
            response->message = "Checkerboard not detected";
            RCLCPP_WARN(this->get_logger(), "Checkerboard not detected in current frame");
            return;
        }

        // 边界安全检查：角点靠边时拒绝采样，降低PnP不稳定风险
        if (!corners.empty()) {
            float min_u = corners[0].x, max_u = corners[0].x;
            float min_v = corners[0].y, max_v = corners[0].y;
            for (const auto& c : corners) {
                min_u = std::min(min_u, c.x);
                max_u = std::max(max_u, c.x);
                min_v = std::min(min_v, c.y);
                max_v = std::max(max_v, c.y);
            }
            const int w = cv_ptr->image.cols;
            const int h = cv_ptr->image.rows;
            if (min_u < board_margin_px_ || min_v < board_margin_px_ ||
                max_u > (w - board_margin_px_) || max_v > (h - board_margin_px_)) {
                response->success = false;
                response->message = "Checkerboard too close to image border";
                RCLCPP_WARN(this->get_logger(),
                    "Rejected sample: board near border [u:(%.1f, %.1f), v:(%.1f, %.1f)], img=(%d,%d), margin=%d",
                    min_u, max_u, min_v, max_v, w, h, board_margin_px_);
                return;
            }
        }
        
        CalibrationSample sample;
        sample.T_base_tool = poseToIsometry(latest_arm_pose_->pose);
        sample.T_cam_board = T_cam_board;
        sample.image = cv_ptr->image.clone();

        samples_.push_back(sample);
        
        Eigen::Quaterniond q(sample.T_base_tool.rotation());
        double roll = std::atan2(2*(q.w()*q.x() + q.y()*q.z()), 1 - 2*(q.x()*q.x() + q.y()*q.y())) * 180 / M_PI;
        double pitch = std::asin(2*(q.w()*q.y() - q.z()*q.x())) * 180 / M_PI;
        double yaw = std::atan2(2*(q.w()*q.z() + q.x()*q.y()), 1 - 2*(q.y()*q.y() + q.z()*q.z())) * 180 / M_PI;
        
        RCLCPP_INFO(this->get_logger(),
            "[Sample %zu/%d] Pos=[%.3f, %.3f, %.3f] Rot=[%.1f°, %.1f°, %.1f°] Quat=[%.3f, %.3f, %.3f, %.3f]",
            samples_.size(), min_samples_,
            sample.T_base_tool.translation().x(),
            sample.T_base_tool.translation().y(),
            sample.T_base_tool.translation().z(),
            roll, pitch, yaw,
            q.x(), q.y(), q.z(), q.w());
        
        response->success = true;
        response->message = "Sample " + std::to_string(samples_.size()) + "/" + std::to_string(min_samples_) + " captured";
    }

    void isStableCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                         std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        (void)request;
        std::string reason;
        double dpos = 0.0;
        double drot = 0.0;
        const bool stable = isRobotStable(&reason, &dpos, &drot);
        response->success = stable;
        if (stable) {
            response->message = "stable: dpos=" + std::to_string(dpos) + "m, drot=" + std::to_string(drot) + "deg";
        } else {
            response->message = "not stable: " + reason;
        }
    }

    void testBoardPoseCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                               std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        (void)request;

        if (!has_calibration_result_) {
            response->success = false;
            response->message = "No calibration result available. Run /calibration/compute first.";
            return;
        }
        if (!latest_arm_pose_ || !latest_image_) {
            response->success = false;
            response->message = "No /tool_pose or image data available";
            return;
        }

        std::string stable_reason;
        double max_dpos = 0.0;
        double max_drot = 0.0;
        if (!isRobotStable(&stable_reason, &max_dpos, &max_drot)) {
            response->success = false;
            response->message = "Robot not stable for test: " + stable_reason;
            return;
        }

        cv_bridge::CvImagePtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvCopy(*latest_image_, sensor_msgs::image_encodings::BGR8);
        } catch (cv_bridge::Exception& e) {
            response->success = false;
            response->message = std::string("Image conversion failed: ") + e.what();
            return;
        }

        Eigen::Isometry3d T_cam_board;
        std::vector<cv::Point2f> corners;
        if (!detector_->detectAndEstimatePose(cv_ptr->image, T_cam_board, corners)) {
            response->success = false;
            response->message = "Checkerboard not detected";
            return;
        }

        const Eigen::Isometry3d T_base_tool = poseToIsometry(latest_arm_pose_->pose);
        const Eigen::Isometry3d T_base_board = T_base_tool * last_calibration_result_.T_tool_cam * T_cam_board;

        Eigen::Quaterniond q_cam_board(T_cam_board.rotation());
        Eigen::Quaterniond q_base_board(T_base_board.rotation());

        RCLCPP_INFO(this->get_logger(),
            "[BoardPoseTest] camera<-board: t=[%.4f, %.4f, %.4f] q=[%.4f, %.4f, %.4f, %.4f]",
            T_cam_board.translation().x(), T_cam_board.translation().y(), T_cam_board.translation().z(),
            q_cam_board.x(), q_cam_board.y(), q_cam_board.z(), q_cam_board.w());

        RCLCPP_INFO(this->get_logger(),
            "[BoardPoseTest] base<-board:   t=[%.4f, %.4f, %.4f] q=[%.4f, %.4f, %.4f, %.4f]",
            T_base_board.translation().x(), T_base_board.translation().y(), T_base_board.translation().z(),
            q_base_board.x(), q_base_board.y(), q_base_board.z(), q_base_board.w());

        std::ostringstream ss;
        ss << std::fixed << std::setprecision(6)
           << "cam_t=[" << T_cam_board.translation().x() << "," << T_cam_board.translation().y() << "," << T_cam_board.translation().z() << "],"
           << "cam_q=[" << q_cam_board.x() << "," << q_cam_board.y() << "," << q_cam_board.z() << "," << q_cam_board.w() << "],"
           << "base_t=[" << T_base_board.translation().x() << "," << T_base_board.translation().y() << "," << T_base_board.translation().z() << "],"
           << "base_q=[" << q_base_board.x() << "," << q_base_board.y() << "," << q_base_board.z() << "," << q_base_board.w() << "]";

        response->success = true;
        response->message = ss.str();
        RCLCPP_INFO(this->get_logger(), "[BoardPoseTest][Structured] %s", response->message.c_str());
    }

    void loadResultCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                            std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        (void)request;

        CalibrationResult loaded;
        if (!io::loadCalibrationResultYAML(output_file_, loaded)) {
            response->success = false;
            response->message = "Failed to load calibration from: " + output_file_;
            RCLCPP_WARN(this->get_logger(), "%s", response->message.c_str());
            return;
        }

        last_calibration_result_ = loaded;
        has_calibration_result_ = true;
        publishCalibrationResult(last_calibration_result_);

        Eigen::Quaterniond q(last_calibration_result_.T_tool_cam.rotation());
        response->success = true;
        response->message =
            "Loaded calibration from: " + output_file_ +
            ", t=[" + std::to_string(last_calibration_result_.T_tool_cam.translation().x()) +
            "," + std::to_string(last_calibration_result_.T_tool_cam.translation().y()) +
            "," + std::to_string(last_calibration_result_.T_tool_cam.translation().z()) +
            "], q=[" + std::to_string(q.x()) +
            "," + std::to_string(q.y()) +
            "," + std::to_string(q.z()) +
            "," + std::to_string(q.w()) + "]";

        RCLCPP_INFO(this->get_logger(), "%s", response->message.c_str());
    }

    void computeCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                        std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        (void)request;

        const size_t n = samples_.size();
        if (n < 3) {
            response->success = false;
            response->message = "Too few samples to solve calibration: " + std::to_string(n) + " (need at least 3)";
            RCLCPP_WARN(this->get_logger(), "%s", response->message.c_str());
            return;
        }

        if (n < static_cast<size_t>(min_samples_)) {
            RCLCPP_WARN(this->get_logger(),
                "Sample count %zu is below configured min_samples=%d. Will still try to compute.",
                n, min_samples_);
        }

        bool ok = computeCalibration();
        response->success = ok;
        if (ok) {
            response->message = "Calibration computed and saved. Samples used: " + std::to_string(n) +
                                ", configured min_samples=" + std::to_string(min_samples_);
        } else {
            response->message = "Calibration solver failed. Check sample diversity and board detections.";
        }
    }
    
    void syncTimerCallback()
    {
        // 定时器保留但不进行采集，采集由服务手动触发
    }
    
    bool computeCalibration()
    {
        // ========== 采集完毕判定点 ==========
        // 位置：computeCalibration() 函数开始处
        // 条件：samples_.size() >= min_samples_
        RCLCPP_INFO(this->get_logger(),
            "\n"
            "╔══════════════════════════════════════════╗\n"
            "║  采集完毕！开始计算手眼标定             ║\n"
            "║  样本数: %zu/%d                      ║\n"
            "╚══════════════════════════════════════════╝",
            samples_.size(), min_samples_);
        RCLCPP_INFO(this->get_logger(),
            "Computing calibration with %zu samples...", samples_.size());
        
        // 单次求解（全量样本），避免复杂黑盒筛选
        CalibrationResult result = solver_->solve(samples_);
        
        if (!result.success) {
            RCLCPP_ERROR(this->get_logger(), "Calibration failed!");
            return false;
        }

        const ConsistencyStats stats = evaluateBoardConsistency(samples_, result.T_tool_cam);
        
        // 打印结果
        Eigen::Quaterniond q(result.T_tool_cam.rotation());
        RCLCPP_INFO(this->get_logger(),
            "Calibration successful!\n"
            "  Translation: [%.4f, %.4f, %.4f] m\n"
            "  Rotation (quat): [%.4f, %.4f, %.4f, %.4f]\n"
            "  Reprojection error: %.6f m\n"
            "  Selected variant: %s\n"
            "  Variant errors A/B/C/D: %.6f / %.6f / %.6f / %.6f m\n"
            "  Samples used: %zu",
            result.T_tool_cam.translation().x(),
            result.T_tool_cam.translation().y(),
            result.T_tool_cam.translation().z(),
            q.x(), q.y(), q.z(), q.w(),
            result.reprojection_error,
            result.selected_variant.c_str(),
            result.variant_error_A,
            result.variant_error_B,
            result.variant_error_C,
            result.variant_error_D,
            samples_.size());

        RCLCPP_INFO(this->get_logger(),
            "Consistency (base<-board):\n"
            "  mean xyz: [%.4f, %.4f, %.4f] m\n"
            "  std  xyz: [%.4f, %.4f, %.4f] m\n"
            "  max radius from mean: %.4f m\n"
            "  rot wrt first: mean/std/max = %.3f / %.3f / %.3f deg",
            stats.pos_mean.x(), stats.pos_mean.y(), stats.pos_mean.z(),
            stats.pos_std.x(), stats.pos_std.y(), stats.pos_std.z(),
            stats.pos_max_radius,
            stats.rot_mean_deg, stats.rot_std_deg, stats.rot_max_deg);
        
        // 保存
        if (io::saveCalibrationResultYAML(output_file_, result)) {
            RCLCPP_INFO(this->get_logger(),
                "Calibration saved to: %s", output_file_.c_str());
        }

        last_calibration_result_ = result;
        has_calibration_result_ = true;
        
        // 发布
        publishCalibrationResult(result);
        
        // 清空样本，可以继续采集
        samples_.clear();
        RCLCPP_INFO(this->get_logger(), 
            "Samples cleared. Ready for new calibration session.\n"
            "Use: ros2 service call /calibration/capture std_srvs/srv/Trigger");

        return true;
    }
    
    void publishCalibrationResult(const CalibrationResult& result)
    {
        geometry_msgs::msg::TransformStamped tf;
        tf.header.stamp = this->now();
        tf.header.frame_id = "tool0";
        tf.child_frame_id = "camera";
        
        tf.transform.translation.x = result.T_tool_cam.translation().x();
        tf.transform.translation.y = result.T_tool_cam.translation().y();
        tf.transform.translation.z = result.T_tool_cam.translation().z();
        
        Eigen::Quaterniond q(result.T_tool_cam.rotation());
        tf.transform.rotation.x = q.x();
        tf.transform.rotation.y = q.y();
        tf.transform.rotation.z = q.z();
        tf.transform.rotation.w = q.w();
        
        calibration_pub_->publish(tf);
    }
    
    Eigen::Isometry3d poseToIsometry(const geometry_msgs::msg::Pose& pose)
    {
        Eigen::Isometry3d iso = Eigen::Isometry3d::Identity();
        Eigen::Vector3d trans(pose.position.x, pose.position.y, pose.position.z);
        Eigen::Quaterniond quat(pose.orientation.w, pose.orientation.x,
                               pose.orientation.y, pose.orientation.z);
        
        Eigen::Matrix4d mat = Eigen::Matrix4d::Identity();
        mat.block<3, 3>(0, 0) = quat.toRotationMatrix();
        mat.block<3, 1>(0, 3) = trans;
        iso = Eigen::Isometry3d(mat);
        return iso;
    }
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<HandEyeCalibrationNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
