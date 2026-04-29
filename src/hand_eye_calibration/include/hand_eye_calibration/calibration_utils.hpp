#pragma once

#include <vector>
#include <string>
#include <Eigen/Dense>
#include <opencv2/core.hpp>

namespace hand_eye_calibration {

/**
 * @brief 标定样本数据结构
 * 
 * 存储每一个标定采样点的数据：
 * - 机械臂末端位姿
 * - 相机观测到的棋盘位姿
 * - 对应的图像
 */
struct CalibrationSample {
    /// 机械臂基座 → 末端工具的变换
    Eigen::Isometry3d T_base_tool;
    
    /// 相机 → 棋盘的变换
    Eigen::Isometry3d T_cam_board;
    
    /// 原始图像（用于验证和调试）
    cv::Mat image;
};

/**
 * @brief 标定结果数据结构
 */
struct CalibrationResult {
    /// 末端工具 → 相机的变换（标定结果）
    Eigen::Isometry3d T_tool_cam;
    
    /// 重投影误差（米）
    double reprojection_error;
    
    /// 采样数量
    size_t num_samples;

    /// 求解时选中的坐标约定分支
    std::string selected_variant;

    /// 四种约定分支的一致性误差（米）
    double variant_error_A;
    double variant_error_B;
    double variant_error_C;
    double variant_error_D;
    
    /// 是否标定成功
    bool success;
};

/**
 * @brief 棋盘检测和位姿估计工具类
 * 
 * 负责：
 * - 检测棋盘角点
 * - 使用 PnP 方法估计相机观测到的棋盘位姿
 */
class CheckerboardDetector {
public:
    /**
     * @brief 构造函数
     * @param fx, fy 相机焦距（像素）
     * @param cx, cy 相机主点（像素）
     */
    CheckerboardDetector(double fx, double fy, double cx, double cy);
    
    /**
     * @brief 设置棋盘参数
     * @param rows 棋盘行数
     * @param cols 棋盘列数
     * @param square_size 棋盘格子边长（米）
     */
    void setCheckerboardSize(int rows, int cols, double square_size);
    
    /**
     * @brief 检测棋盘并估计位姿
     * @param image 输入图像
     * @param T_cam_board [输出] 相机到棋盘的变换
     * @param corners [输出] 检测到的角点（像素坐标）
     * @return 是否成功检测
     */
    bool detectAndEstimatePose(const cv::Mat& image,
                               Eigen::Isometry3d& T_cam_board,
                               std::vector<cv::Point2f>& corners);
    
    /**
     * @brief 获取相机内参矩阵
     */
    cv::Mat getCameraMatrix() const { return camera_matrix_; }

private:
    cv::Mat camera_matrix_;
    cv::Mat dist_coeffs_;
    int board_rows_, board_cols_;
    double square_size_m_;
    
    /// 创建棋盘的3D点（棋盘坐标系）
    std::vector<cv::Point3f> createBoardPoints();
};

/**
 * @brief 手眼标定求解器（基于 OpenCV calibrateHandEye）
 */
class TsaiLenzSolver {
public:
    /**
     * @brief 计算标定结果
     * @param samples 标定样本集合
     * @return 标定结果
     */
    CalibrationResult solve(const std::vector<CalibrationSample>& samples);
    
    /**
     * @brief 验证标定结果的准确度
     * @param samples 标定样本
     * @param T_tool_cam 标定得到的变换
     * @return 平均重投影误差（米）
     */
    double validateCalibration(const std::vector<CalibrationSample>& samples,
                               const Eigen::Isometry3d& T_tool_cam);
};

/**
 * @brief 文件 I/O 工具函数
 */
namespace io {
    /**
     * @brief 保存标定结果为 YAML 格式
     * @param file_path 输出文件路径
     * @param result 标定结果
     * @return 是否成功
     */
    bool saveCalibrationResultYAML(const std::string& file_path,
                                   const CalibrationResult& result);
    
    /**
     * @brief 从 YAML 文件加载标定结果
     * @param file_path 输入文件路径
     * @param result [输出] 标定结果
     * @return 是否成功
     */
    bool loadCalibrationResultYAML(const std::string& file_path,
                                   CalibrationResult& result);
}

} // namespace hand_eye_calibration
