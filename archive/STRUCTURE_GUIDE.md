# 🗂️ 完整项目结构说明

## 整体工作区结构

```
arm_ws/
│
├── 📄 README.md                          ← 项目总体说明
├── 📄 INTEGRATION_GUIDE.md               ← 系统集成指南 ⭐ 从这里开始
├── 📄 QUICK_REFERENCE.md                ← 快速参考卡
│
├── 📂 build/                            (编译输出)
├── 📂 install/                          (安装输出)
├── 📂 log/                              (日志)
│
└── 📂 src/
    │
    ├── 📂 control/                      [现有包] 机械臂控制
    │   ├── CMakeLists.txt
    │   ├── package.xml
    │   ├── src/
    │   │   ├── pose_mover.cpp           ← 已修改：发布 /tool_pose
    │   │   └── ...
    │   ├── launch/
    │   └── config/
    │
    └── 📂 hand_eye_calibration/        [新建包] 手眼标定 ⭐ 重点
        │
        ├── 📄 CMakeLists.txt            [编译配置]
        │   └─ 配置了 3 个目标：
        │      ├─ calibration_utils (库)
        │      ├─ hand_eye_calibration_node (节点)
        │      └─ visual_servo_controller_node (节点)
        │
        ├── 📄 package.xml               [包声明]
        │   └─ 声明了所有依赖
        │
        ├── 📄 README.md                 [详细文档] ⭐ 必读
        │   ├─ 包结构说明
        │   ├─ 组件介绍
        │   ├─ 快速开始
        │   └─ API 示例
        │
        ├── 📂 include/
        │   └── 📂 hand_eye_calibration/
        │       └── 📄 calibration_utils.hpp  [库头文件]
        │           ├─ CheckerboardDetector 类
        │           ├─ TsaiLenzSolver 类
        │           ├─ CalibrationSample 结构
        │           ├─ CalibrationResult 结构
        │           └─ io 命名空间 (文件 I/O)
        │
        ├── 📂 src/
        │   ├── 📄 calibration_utils.cpp      [库实现]
        │   │   ├─ 棋盘检测 (PnP)
        │   │   ├─ Tsai-Lenz 求解
        │   │   └─ YAML I/O
        │   │
        │   ├── 📄 hand_eye_calibration_node.cpp [标定节点]
        │   │   └─ ROS2 节点实现
        │   │       ├─ 订阅: /tool_pose, /camera/color/image_raw
        │   │       ├─ 发布: /calibration_result
        │   │       └─ 功能：同步采样 → 自动标定
        │   │
        │   └── 📄 visual_servo_controller_node.cpp [伺服节点]
        │       └─ ROS2 节点实现
        │           ├─ 订阅: 相机图像, /tool_pose
        │           ├─ 发布: /target_pose
        │           └─ 功能：目标检测 → 坐标变换
        │
        ├── 📂 launch/
        │   ├── 📄 hand_eye_calibration.launch.py
        │   │   ├─ 参数：arm_pose_topic, camera_image_topic
        │   │   └─ 启动：hand_eye_calibration_node
        │   │
        │   └── 📄 visual_servo_demo.launch.py
        │       ├─ 参数：calibration_file, target_color
        │       └─ 启动：visual_servo_controller_node
        │
        ├── 📂 config/
        │   └── 📄 hand_eye_calibration.yaml   [参数配置]
        │       ├─ 棋盘参数
        │       ├─ 相机参数 (D435i)
        │       └─ 标定参数
        │
        └── 📂 docs/  (可选，用于额外文档)
            ├── api_examples.md
            └── troubleshooting.md
```

## 📑 文件详细说明

### 🔷 CMakeLists.txt - 构建配置

```cmake
# 定义了 3 个编译目标

1️⃣ Library: calibration_utils
   - 包含源文件：calibration_utils.cpp
   - 导出头文件：include/hand_eye_calibration/
   - 用途：被节点或外部代码链接

2️⃣ Executable: hand_eye_calibration_node
   - 源文件：hand_eye_calibration_node.cpp
   - 链接库：calibration_utils
   - 安装位置：lib/hand_eye_calibration/

3️⃣ Executable: visual_servo_controller_node
   - 源文件：visual_servo_controller_node.cpp
   - 链接库：calibration_utils
   - 安装位置：lib/hand_eye_calibration/
```

### 🔷 package.xml - 包元数据

```xml
<package format="3">
  <name>hand_eye_calibration</name>
  <version>1.0.0</version>
  
  <!-- 构建依赖 -->
  <buildtool_depend>ament_cmake</buildtool_depend>
  
  <!-- 运行时依赖 -->
  <depend>rclcpp</depend>          ← ROS2 C++ 库
  <depend>geometry_msgs</depend>   ← 位姿消息
  <depend>sensor_msgs</depend>     ← 图像消息
  <depend>cv_bridge</depend>       ← OpenCV 与 ROS 转换
  <depend>OpenCV</depend>          ← 图像处理
  <depend>Eigen3</depend>          ← 线性代数
  ...
</package>
```

### 🔷 calibration_utils.hpp - 库接口

```cpp
namespace hand_eye_calibration {

// 1. 数据结构
struct CalibrationSample {
    Eigen::Isometry3d T_base_tool;   // 机械臂位姿
    Eigen::Isometry3d T_cam_board;   // 相机观测
    cv::Mat image;                   // 原始图像
};

struct CalibrationResult {
    Eigen::Isometry3d T_tool_cam;    // 标定结果（末端→相机）
    double reprojection_error;       // 误差
    size_t num_samples;              // 样本数
    bool success;                    // 是否成功
};

// 2. 检测类
class CheckerboardDetector {
    bool detectAndEstimatePose(...);  // 检测棋盘位姿
};

// 3. 标定求解类
class TsaiLenzSolver {
    CalibrationResult solve(...);    // 求解标定
    double validateCalibration(...); // 验证结果
};

// 4. 文件 I/O
namespace io {
    bool saveCalibrationResultYAML(...);
    bool loadCalibrationResultYAML(...);
}

}
```

### 🔷 hand_eye_calibration_node.cpp - 标定节点

```cpp
class HandEyeCalibrationNode : public rclcpp::Node {
public:
    HandEyeCalibrationNode() {
        // 订阅机械臂末端位姿
        arm_pose_sub_ = create_subscription<PoseStamped>(
            "arm_pose", ...);
        
        // 订阅相机图像
        camera_image_sub_ = create_subscription<Image>(
            "camera_image", ...);
        
        // 发布标定结果
        calibration_pub_ = create_publisher<TransformStamped>(
            "calibration_result", ...);
    }
    
private:
    void syncTimerCallback() {
        // 1. 检测棋盘
        detector_->detectAndEstimatePose(image, ...);
        
        // 2. 创建样本
        samples_.push_back({T_base_tool, T_cam_board, image});
        
        // 3. 达到最少数量时计算
        if (samples_.size() >= min_samples_) {
            computeCalibration();
        }
    }
    
    void computeCalibration() {
        // 求解标定
        result = solver_->solve(samples_);
        
        // 保存和发布
        io::saveCalibrationResultYAML(output_file_, result);
        publishCalibrationResult(result);
    }
};
```

### 🔷 visual_servo_controller_node.cpp - 伺服节点

```cpp
class VisualServoControllerNode : public rclcpp::Node {
private:
    Eigen::Isometry3d T_tool_cam;  // 从标定结果加载
    
    void imageCallback(...) {
        // 1. 检测目标
        points = detectTarget(image);
        
        // 2. 估计相机坐标
        T_cam_target = estimateTargetPose(points);
        
        // 3. 坐标变换：相机 → 基座
        // T_base_target = T_base_tool * T_tool_cam * T_cam_target
        T_base_target = T_base_tool_ * T_tool_cam_ * T_cam_target;
        
        // 4. 发布目标位姿
        target_pose_pub_->publish(T_base_target);
    }
};
```

### 🔷 hand_eye_calibration.yaml - 参数配置

```yaml
# 棋盘参数
checkerboard_rows: 6
checkerboard_cols: 9
square_size_mm: 30.0

# 相机参数（D435i）
camera_fx: 906.948
camera_fy: 905.906
camera_cx: 648.379
camera_cy: 383.873

# 标定参数
min_samples: 10
output_file: /tmp/hand_eye_calibration_result.yaml
```

## 🔄 代码流程图

### 标定流程

```
hand_eye_calibration_node
├─ 接收 /tool_pose (来自 pose_mover)
├─ 接收 /camera/color/image_raw (来自相机)
│
└─ 定时同步 (100ms)
   ├─ 调用 CheckerboardDetector::detectAndEstimatePose()
   │  ├─ cv::findChessboardCorners()
   │  ├─ cv::solvePnP()
   │  └─ 返回 T_cam_board
   │
   ├─ 创建 CalibrationSample
   │  └─ 存储 (T_base_tool, T_cam_board, image)
   │
   └─ 当样本数 >= min_samples
      ├─ 调用 TsaiLenzSolver::solve()
      │  ├─ 求解旋转 (SVD)
      │  ├─ 求解平移 (LSQ)
      │  └─ 返回 T_tool_cam
      │
      ├─ 验证结果 (计算重投影误差)
      ├─ 保存到 YAML
      └─ 发布结果
```

### 视觉伺服流程

```
visual_servo_controller_node
├─ 加载标定结果 T_tool_cam
├─ 接收 /camera/color/image_raw
│
└─ 处理每一帧
   ├─ detectTarget() → 找到像素坐标
   ├─ estimateTargetPose() → 获得 T_cam_target
   │
   ├─ 坐标变换
   │  T_base_target = T_base_tool * T_tool_cam * T_cam_target
   │
   ├─ 发布 /target_pose
   └─ 输出 /debug_image
```

## 📊 数据流关系

```
┌─────────────────────────────────┐
│         相机驱动                 │
│  (RealSense D435i)             │
└──────────┬──────────────────────┘
           │
           ├─→ /camera/color/image_raw
           │
           └─→ /camera/camera_info

           ↓

┌──────────────────────────────────┐
│    pose_mover 节点               │
│    (control 包)                  │
└──────────┬───────────────────────┘
           │
           └─→ /tool_pose (末端位姿)

           ↓

    ┌─────────────┴──────────────┐
    ↓                            ↓
    
┌────────────────────┐   ┌────────────────────────┐
│ hand_eye_          │   │ visual_servo_          │
│ calibration_node   │   │ controller_node        │
└────────┬───────────┘   └────────┬───────────────┘
         │                        │
         └─→ 标定结果             └─→ /target_pose_visual
             YAML
```

## 🏗️ 编译依赖关系

```
hand_eye_calibration
├── calibration_utils 库
│   ├── depends: Eigen3, OpenCV
│   └── provides: 标定算法
│
├── hand_eye_calibration_node
│   ├── links: calibration_utils
│   ├── depends: rclcpp, geometry_msgs, sensor_msgs, cv_bridge, yaml-cpp
│   └── provides: ROS2 标定服务
│
└── visual_servo_controller_node
    ├── links: calibration_utils
    ├── depends: rclcpp, geometry_msgs, sensor_msgs, cv_bridge
    └── provides: ROS2 伺服服务
```

## 🎯 关键文件修改清单

### 原有包修改

```
control/src/pose_mover.cpp
├─ 新增：tool_pose_pub_ (发布末端位姿)
├─ 修改：publishJointStates() 中添加位姿计算
└─ 新增：订阅 /target_pose_visual (接收伺服目标)
```

### 新增包

```
hand_eye_calibration/  (全部新建)
├── CMakeLists.txt (60 行)
├── package.xml (45 行)
├── include/hand_eye_calibration/calibration_utils.hpp (140 行)
├── src/calibration_utils.cpp (340 行)
├── src/hand_eye_calibration_node.cpp (240 行)
├── src/visual_servo_controller_node.cpp (260 行)
├── launch/*.launch.py (各 50 行)
├── config/*.yaml (30 行)
└── README.md (600 行)
```

---

## 📚 导航

- **快速开始**：→ README.md
- **系统集成**：→ INTEGRATION_GUIDE.md  
- **快速参考**：→ QUICK_REFERENCE.md
- **API 文档**：→ include/hand_eye_calibration/calibration_utils.hpp
- **相机参数**：→ D435i_quick_reference.md
