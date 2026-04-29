#include "hand_eye_calibration/calibration_utils.hpp"
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <iostream>
#include <limits>

namespace hand_eye_calibration {

// ============ CheckerboardDetector 实现 ============

CheckerboardDetector::CheckerboardDetector(double fx, double fy, double cx, double cy)
    : board_rows_(6), board_cols_(9), square_size_m_(0.03)
{
    camera_matrix_ = cv::Mat::eye(3, 3, CV_64F);
    camera_matrix_.at<double>(0, 0) = fx;  // fx
    camera_matrix_.at<double>(1, 1) = fy;  // fy
    camera_matrix_.at<double>(0, 2) = cx;  // cx
    camera_matrix_.at<double>(1, 2) = cy;  // cy
    
    // D435i 已校正，无畸变
    dist_coeffs_ = cv::Mat::zeros(5, 1, CV_64F);
}

void CheckerboardDetector::setCheckerboardSize(int rows, int cols, double square_size)
{
    board_rows_ = rows;
    board_cols_ = cols;
    square_size_m_ = square_size;
}

std::vector<cv::Point3f> CheckerboardDetector::createBoardPoints()
{
    std::vector<cv::Point3f> points;
    for (int i = 0; i < board_rows_; ++i) {
        for (int j = 0; j < board_cols_; ++j) {
            points.push_back(cv::Point3f(
                j * square_size_m_,
                i * square_size_m_,
                0.0f
            ));
        }
    }
    return points;
}

bool CheckerboardDetector::detectAndEstimatePose(const cv::Mat& image,
                                                  Eigen::Isometry3d& T_cam_board,
                                                  std::vector<cv::Point2f>& corners)
{
    // 转灰度图
    cv::Mat gray;
    if (image.channels() == 3) {
        cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
    } else {
        gray = image.clone();
    }
    
    // 检测棋盘角点
    bool found = cv::findChessboardCorners(
        gray,
        cv::Size(board_cols_, board_rows_),
        corners,
        cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE
    );
    
    if (!found) {
        return false;
    }
    
    // 细化角点位置
    cv::cornerSubPix(gray, corners, cv::Size(11, 11), cv::Size(-1, -1),
        cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 30, 0.001)
    );
    
    // 创建棋盘3D点
    std::vector<cv::Point3f> board_3d = createBoardPoints();
    
    // PnP 求解
    cv::Mat rvec, tvec;
    bool pnp_success = cv::solvePnP(
        board_3d, corners, camera_matrix_, dist_coeffs_,
        rvec, tvec, false, cv::SOLVEPNP_ITERATIVE
    );
    
    if (!pnp_success) {
        return false;
    }
    
    // 转换为 Eigen 格式
    cv::Mat R;
    cv::Rodrigues(rvec, R);
    
    T_cam_board = Eigen::Isometry3d::Identity();
    Eigen::Matrix3d eig_R;
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            eig_R(i, j) = R.at<double>(i, j);
        }
    }
    
    // Build transformation matrix
    Eigen::Matrix4d T_mat = Eigen::Matrix4d::Identity();
    T_mat.block<3, 3>(0, 0) = eig_R;
    T_mat(0, 3) = tvec.at<double>(0);
    T_mat(1, 3) = tvec.at<double>(1);
    T_mat(2, 3) = tvec.at<double>(2);
    T_cam_board = Eigen::Isometry3d(T_mat);
    
    return true;
}

// ============ TsaiLenzSolver 实现 ============

CalibrationResult TsaiLenzSolver::solve(const std::vector<CalibrationSample>& samples)
{
    CalibrationResult result;
    result.success = false;
    result.num_samples = samples.size();
    result.reprojection_error = -1.0;
    result.selected_variant = "";
    result.variant_error_A = -1.0;
    result.variant_error_B = -1.0;
    result.variant_error_C = -1.0;
    result.variant_error_D = -1.0;
    
    if (samples.size() < 3) {
        return result;
    }

    auto toCvRt = [](const Eigen::Isometry3d& T, cv::Mat& R, cv::Mat& t) {
        R = cv::Mat::zeros(3, 3, CV_64F);
        t = cv::Mat::zeros(3, 1, CV_64F);
        const Eigen::Matrix3d Re = T.rotation();
        const Eigen::Vector3d te = T.translation();
        for (int r = 0; r < 3; ++r) {
            for (int c = 0; c < 3; ++c) {
                R.at<double>(r, c) = Re(r, c);
            }
            t.at<double>(r, 0) = te(r);
        }
    };

    struct Variant {
        const char* name;
        bool invert_gripper;
        bool invert_target;
    };

    const std::vector<Variant> variants = {
        {"A:g2b=T_base_tool, t2c=T_cam_board", false, false},
        {"B:g2b=inv(T_base_tool), t2c=T_cam_board", true, false},
        {"C:g2b=T_base_tool, t2c=inv(T_cam_board)", false, true},
        {"D:g2b=inv(T_base_tool), t2c=inv(T_cam_board)", true, true},
    };

    auto setVariantError = [&](const std::string& name, double err) {
        if (!std::isfinite(err)) return;
        if (!name.empty()) {
            switch (name[0]) {
                case 'A': result.variant_error_A = err; break;
                case 'B': result.variant_error_B = err; break;
                case 'C': result.variant_error_C = err; break;
                case 'D': result.variant_error_D = err; break;
                default: break;
            }
        }
    };

    double best_err = std::numeric_limits<double>::infinity();
    Eigen::Isometry3d best_T_tool_cam = Eigen::Isometry3d::Identity();
    std::string best_name;

    for (const auto& v : variants) {
        std::vector<cv::Mat> R_gripper2base, t_gripper2base;
        std::vector<cv::Mat> R_target2cam, t_target2cam;
        R_gripper2base.reserve(samples.size());
        t_gripper2base.reserve(samples.size());
        R_target2cam.reserve(samples.size());
        t_target2cam.reserve(samples.size());

        for (const auto& s : samples) {
            const Eigen::Isometry3d T_g2b = v.invert_gripper ? s.T_base_tool.inverse() : s.T_base_tool;
            const Eigen::Isometry3d T_t2c = v.invert_target ? s.T_cam_board.inverse() : s.T_cam_board;
            cv::Mat Rg, tg, Rt, tt;
            toCvRt(T_g2b, Rg, tg);
            toCvRt(T_t2c, Rt, tt);
            R_gripper2base.push_back(Rg);
            t_gripper2base.push_back(tg);
            R_target2cam.push_back(Rt);
            t_target2cam.push_back(tt);
        }

        cv::Mat R_cam2gripper, t_cam2gripper;
        cv::calibrateHandEye(
            R_gripper2base,
            t_gripper2base,
            R_target2cam,
            t_target2cam,
            R_cam2gripper,
            t_cam2gripper,
            cv::CALIB_HAND_EYE_PARK
        );

        Eigen::Matrix4d T_mat = Eigen::Matrix4d::Identity();
        for (int r = 0; r < 3; ++r) {
            for (int c = 0; c < 3; ++c) {
                T_mat(r, c) = R_cam2gripper.at<double>(r, c);
            }
            T_mat(r, 3) = t_cam2gripper.at<double>(r, 0);
        }

        // 正交化旋转，避免数值误差导致非单位四元数
        Eigen::Matrix3d R_raw = T_mat.block<3, 3>(0, 0);
        Eigen::JacobiSVD<Eigen::Matrix3d> svd(R_raw, Eigen::ComputeFullU | Eigen::ComputeFullV);
        Eigen::Matrix3d R_ortho = svd.matrixU() * svd.matrixV().transpose();
        if (R_ortho.determinant() < 0.0) {
            Eigen::Matrix3d U = svd.matrixU();
            U.col(2) *= -1.0;
            R_ortho = U * svd.matrixV().transpose();
        }
        T_mat.block<3, 3>(0, 0) = R_ortho;

        const Eigen::Isometry3d T_tool_cam_candidate(T_mat);
        const double err = validateCalibration(samples, T_tool_cam_candidate);

        std::cout << "[HandEyeSolveDiag] " << v.name
                  << ", consistency_error=" << err
                  << ", |t|=" << T_tool_cam_candidate.translation().norm()
                  << std::endl;

        setVariantError(v.name, err);

        if (std::isfinite(err) && err < best_err) {
            best_err = err;
            best_T_tool_cam = T_tool_cam_candidate;
            best_name = v.name;
        }
    }

    if (!std::isfinite(best_err)) {
        return result;
    }

    std::cout << "[HandEyeSolveDiag] selected_variant=" << best_name
              << ", best_error=" << best_err
              << ", best_|t|=" << best_T_tool_cam.translation().norm()
              << std::endl;

    result.T_tool_cam = best_T_tool_cam;
    result.reprojection_error = best_err;
    result.selected_variant = best_name;
    result.success = true;
    
    return result;
}

double TsaiLenzSolver::validateCalibration(const std::vector<CalibrationSample>& samples,
                                           const Eigen::Isometry3d& T_tool_cam)
{
    if (samples.empty()) {
        return -1.0;
    }

    // 对于理想标定，^bT_board(i) = ^bT_tool(i) * ^tT_cam * ^cT_board(i) 应该在所有样本中一致
    std::vector<Eigen::Vector3d> board_positions_base;
    board_positions_base.reserve(samples.size());

    for (const auto& sample : samples) {
        Eigen::Isometry3d T_base_board = sample.T_base_tool * T_tool_cam * sample.T_cam_board;
        board_positions_base.push_back(T_base_board.translation());
    }

    Eigen::Vector3d mean = Eigen::Vector3d::Zero();
    for (const auto& p : board_positions_base) {
        mean += p;
    }
    mean /= static_cast<double>(board_positions_base.size());

    double total_error = 0.0;
    for (const auto& p : board_positions_base) {
        total_error += (p - mean).norm();
    }

    return total_error / static_cast<double>(board_positions_base.size());
}

// ============ I/O 函数实现 ============

namespace io {

bool saveCalibrationResultYAML(const std::string& file_path,
                               const CalibrationResult& result)
{
    YAML::Node node;

    Eigen::Quaterniond q(result.T_tool_cam.rotation());
    q.normalize();
    const Eigen::Matrix4d T = result.T_tool_cam.matrix();
    
    // 输出字段统一为 tool -> camera（与代码变量 T_tool_cam、一致于 easy_handeye2 发布语义）
    node["tool_to_camera"]["translation"]["x"] = result.T_tool_cam.translation().x();
    node["tool_to_camera"]["translation"]["y"] = result.T_tool_cam.translation().y();
    node["tool_to_camera"]["translation"]["z"] = result.T_tool_cam.translation().z();
    node["tool_to_camera"]["rotation"]["x"] = q.x();
    node["tool_to_camera"]["rotation"]["y"] = q.y();
    node["tool_to_camera"]["rotation"]["z"] = q.z();
    node["tool_to_camera"]["rotation"]["w"] = q.w();

    node["tool_to_camera"]["matrix_4x4"] = std::vector<double>{
        T(0, 0), T(0, 1), T(0, 2), T(0, 3),
        T(1, 0), T(1, 1), T(1, 2), T(1, 3),
        T(2, 0), T(2, 1), T(2, 2), T(2, 3),
        T(3, 0), T(3, 1), T(3, 2), T(3, 3)};
    
    // 元数据
    node["metadata"]["reprojection_error"] = result.reprojection_error;
    node["metadata"]["num_samples"] = static_cast<int>(result.num_samples);
    node["metadata"]["timestamp"] = std::time(nullptr);
    node["metadata"]["units_translation"] = "meter";
    node["metadata"]["rotation_format"] = "quaternion_xyzw";
    node["metadata"]["transform_semantics"] = "tool_to_camera";
    node["metadata"]["selected_variant"] = result.selected_variant;
    node["metadata"]["variant_error_A"] = result.variant_error_A;
    node["metadata"]["variant_error_B"] = result.variant_error_B;
    node["metadata"]["variant_error_C"] = result.variant_error_C;
    node["metadata"]["variant_error_D"] = result.variant_error_D;
    
    std::ofstream fout(file_path);
    if (!fout.is_open()) {
        return false;
    }
    
    fout << node;
    fout.close();
    return true;
}

bool loadCalibrationResultYAML(const std::string& file_path,
                               CalibrationResult& result)
{
    try {
        YAML::Node config = YAML::LoadFile(file_path);

        YAML::Node tf_node = config["tool_to_camera"];
        if (!tf_node) {
            return false;
        }
        
        double tx = tf_node["translation"]["x"].as<double>();
        double ty = tf_node["translation"]["y"].as<double>();
        double tz = tf_node["translation"]["z"].as<double>();
        
        double qx = tf_node["rotation"]["x"].as<double>();
        double qy = tf_node["rotation"]["y"].as<double>();
        double qz = tf_node["rotation"]["z"].as<double>();
        double qw = tf_node["rotation"]["w"].as<double>();
        
        // Create rotation from quaternion
        Eigen::Quaterniond quat(qw, qx, qy, qz);
        Eigen::Matrix3d rotation = quat.toRotationMatrix();
        Eigen::Vector3d translation(tx, ty, tz);
        
        // Create the transform using matrix operations
        Eigen::Matrix4d mat = Eigen::Matrix4d::Identity();
        mat.block<3, 3>(0, 0) = rotation;
        mat.block<3, 1>(0, 3) = translation;
        result.T_tool_cam = Eigen::Isometry3d(mat);
        
        result.reprojection_error = config["metadata"]["reprojection_error"].as<double>();
        result.num_samples = config["metadata"]["num_samples"].as<int>();
        if (config["metadata"]["selected_variant"]) {
            result.selected_variant = config["metadata"]["selected_variant"].as<std::string>();
        } else {
            result.selected_variant = "";
        }
        result.variant_error_A = config["metadata"]["variant_error_A"]
            ? config["metadata"]["variant_error_A"].as<double>() : -1.0;
        result.variant_error_B = config["metadata"]["variant_error_B"]
            ? config["metadata"]["variant_error_B"].as<double>() : -1.0;
        result.variant_error_C = config["metadata"]["variant_error_C"]
            ? config["metadata"]["variant_error_C"].as<double>() : -1.0;
        result.variant_error_D = config["metadata"]["variant_error_D"]
            ? config["metadata"]["variant_error_D"].as<double>() : -1.0;
        result.success = true;
        
        return true;
    } catch (const std::exception& e) {
        return false;
    }
}

} // namespace io

} // namespace hand_eye_calibration
