# Hand-Eye Calibration Package

用于机械臂与 Intel RealSense D435i 相机的手眼标定，基于 OpenCV hand-eye 标定实现。

## 📦 包结构

```
hand_eye_calibration/
├── CMakeLists.txt                          # 编译配置
├── package.xml                             # 包元数据
├── README.md                               # 本文件
│
├── include/hand_eye_calibration/
│   └── calibration_utils.hpp               # 标定工具库头文件
│
├── src/
│   ├── calibration_utils.cpp               # 标定工具库实现
│   └── hand_eye_calibration_node.cpp       # 标定节点
│
├── launch/
│   └── hand_eye_calibration.launch.py      # 标定 launch 文件
│
└── config/
    └── hand_eye_calibration.yaml           # 标定参数配置
```

## 🔧 核心组件

### 1. `calibration_utils.hpp/cpp` - 标定工具库

**主要类**：

#### `CheckerboardDetector`
- 检测棋盘角点
- 使用 PnP 方法估计棋盘位姿
- 支持可配置的棋盘参数

```cpp
CheckerboardDetector detector(fx, fy, cx, cy);
detector.setCheckerboardSize(6, 9, 0.03);  // 6x9, 30mm
Eigen::Isometry3d T_cam_board;
std::vector<cv::Point2f> corners;
detector.detectAndEstimatePose(image, T_cam_board, corners);
```

#### `TsaiLenzSolver`
- 求解手眼标定
- 计算末端工具 → 相机的变换 X
- 内部使用 OpenCV `calibrateHandEye` 进行求解

```cpp
TsaiLenzSolver solver;
CalibrationResult result = solver.solve(samples);
double error = result.reprojection_error;
```

#### `CalibrationSample` / `CalibrationResult`
- 数据结构：存储每个采样点的信息
- 存储标定结果（位置、旋转、误差等）

### 2. `hand_eye_calibration_node` - 标定节点

**功能**：
- 订阅机械臂末端位姿：`/tool_pose`
- 订阅相机图像：`/camera/color/image_raw`
- 同步采集标定样本
- 手动触发计算并发布标定结果

**参数**：
```yaml
checkerboard_rows: 8
checkerboard_cols: 8
square_size_mm: 33.0
camera_fx: 906.948
camera_fy: 905.906
camera_cx: 648.379
camera_cy: 383.873
min_samples: 20
output_file: /home/arnoyin/grad_proj/other_hands/implementation/arm_ws/hand_eye_calibration_result.yaml
max_pose_age_sec: 2.0
max_image_age_sec: 3.0
board_margin_px: 20
stability_window_sec: 2.0
stable_pos_thresh_m: 0.0012
stable_rot_thresh_deg: 0.2
```

**输入话题**：
- `/tool_pose` (geometry_msgs/PoseStamped)：机械臂末端位姿
- `/camera/camera/color/image_raw` (sensor_msgs/Image)：相机 RGB 图像

**输出话题**：
- `calibration_result` (geometry_msgs/TransformStamped)：标定结果

## 🚀 快速开始

### 1. 编译

```bash
cd ~/arm_ws
colcon build --packages-select hand_eye_calibration
source install/setup.bash
```

### 2. 准备工作

#### 相机准备
```bash
# 确保相机驱动已安装
ros2 launch realsense2_camera rs_launch.py rgb_camera.color_profile:=1280x720x30

# 验证话题
ros2 topic list | grep camera
```

#### 棋盘准备
- 打印 9×6 的棋盘（每格 30mm）
- 粘贴在平面上
- 放在机械臂末端工具能观察到的位置

### 3. 启动标定

**终端 1：启动 ROS 节点（pose_mover 等）**
```bash
ros2 run control pose_mover_node
```

**终端 2：启动相机驱动**
```bash
ros2 launch realsense2_camera rs_launch.py rgb_camera.color_profile:=1280x720x30
```

**终端 3：启动标定节点**
```bash
ros2 launch hand_eye_calibration hand_eye_calibration.launch.py \
  arm_pose_topic:=/tool_pose \
  camera_image_topic:=/camera/color/image_raw
```

### 4. 采集数据

使用手柄或其他方式控制机械臂，使其**移动到不同位置**。
要求：
- 相机始终能清晰看到棋盘
- 采集位置应分散在工作空间中（≥10个位置）

**采集位置建议**：
- 位置 1-3：棋盘正前方，不同距离
- 位置 4-6：棋盘倾斜 30-60°
- 位置 7-9：棋盘在左/右侧
- 位置 10+：棋盘在上/下方

### 4.1 使用自动脚本（推荐）

工作区根目录脚本 [calib_pose_auto.sh](../../calib_pose_auto.sh) 已重构为精简流程：

- 预设一组基于当前 URDF 的保守可达姿态（约 50cm、适合完整观察 8x8 角点 / 33mm 棋盘）
- 自动执行：采样 -> 计算 -> 结果快照保存 -> 定点测试 -> 静态漂移统计
- 自动生成运行目录：`archive/calib_runs/<timestamp>/`

运行：

```bash
cd ~/arm_ws
./calib_pose_auto.sh
```

可选环境变量：

```bash
REQUIRED_SUCCESS=20 STATIC_TEST_SAMPLES=20 ./calib_pose_auto.sh
```

输出文件（每次运行）：
- `archive/calib_runs/<timestamp>/hand_eye_calibration_result.yaml`
- `archive/calib_runs/<timestamp>/summary.txt`
- `archive/calib_runs/<timestamp>/capture_log.txt`
- `archive/calib_runs/<timestamp>/test_board_raw.txt`
- 根目录快照：`hand_eye_calibration_result_<timestamp>.yaml`

### 5. 计算标定结果

采样完成后手动调用：
```bash
ros2 service call /calibration/compute std_srvs/srv/Trigger
```

输出类似：
```
[hand_eye_calibration_node] Computing calibration with 12 samples...
[hand_eye_calibration_node] Calibration successful!
  Translation: [0.0523, -0.0045, 0.1234] m
  Rotation (quat): [0.0012, 0.7071, -0.0015, 0.7070]
  Reprojection error: 0.002345 m
[hand_eye_calibration_node] Calibration saved to: /tmp/hand_eye_calibration_result.yaml
```

## 📊 标定结果验证（直观版）

当前节点已提供两种直接可读的验证输出：

1) `/calibration/compute` 输出一致性统计（`base<-board`）
- mean xyz
- std xyz
- max radius
- rot mean/std/max (deg)

2) `/calibration/test_board_pose` 返回结构化字符串
- `cam_t/cam_q/base_t/base_q`
- 可被脚本直接解析做漂移统计

### 推荐验收标准
- 连续 3 次独立标定，两两比较：
  - 平移差 `dT < 0.02~0.03 m`
  - 旋转差 `dR < 5~8 deg`
- 静态棋盘漂移（固定板不动）：
  - `base<-board` 的 `std`、`max` 明显小于历史结果

### 一键统计脚本
新增脚本：`scripts/repeatability_check.py`

```bash
# 1) 比较三次标定结果
python3 src/hand_eye_calibration/scripts/repeatability_check.py compare \
  --files hand_eye_calibration_result_1.yaml hand_eye_calibration_result_2.yaml hand_eye_calibration_result_3.yaml \
  --trans-thr 0.03 --rot-thr 8

# 2) 静态棋盘漂移统计（节点运行中）
python3 src/hand_eye_calibration/scripts/repeatability_check.py board --samples 30 --interval 0.5
```

输出会给出：每次测量值 + 均值 + std + max + PASS/FAIL。

## 📝 API 使用示例

### 使用标定库

```cpp
#include "hand_eye_calibration/calibration_utils.hpp"
using namespace hand_eye_calibration;

// 1. 检测棋盘
CheckerboardDetector detector(906.948, 905.906, 648.379, 383.873);
detector.setCheckerboardSize(6, 9, 0.03);

cv::Mat image = cv::imread("image.jpg");
Eigen::Isometry3d T_cam_board;
std::vector<cv::Point2f> corners;
if (detector.detectAndEstimatePose(image, T_cam_board, corners)) {
    std::cout << "棋盘位置: " << T_cam_board.translation().transpose() << std::endl;
}

// 2. 从 YAML 加载标定结果
CalibrationResult result;
io::loadCalibrationResultYAML("/path/to/result.yaml", result);
Eigen::Isometry3d T_tool_cam = result.T_tool_cam;

// 3. 进行坐标变换
Eigen::Isometry3d T_base_tool = ...;  // 从机械臂获得
Eigen::Isometry3d T_cam_target = ...;  // 从视觉检测获得

// 计算目标在基座坐标系中的位置
Eigen::Isometry3d T_base_target = T_base_tool * T_tool_cam * T_cam_target;
```

## 🔍 故障排除

| 问题 | 原因 | 解决方案 |
|-----|-----|--------|
| 棋盘检测失败 | 光线不足/角度不佳 | 改善光线、调整相机角度 |
| 重投影误差很大 | 采集位置分布不均 | 增加采集数量、覆盖更多方向 |
| 相机参数错误 | 使用了错误的标定 | 重新标定相机内参 |
| 坐标变换错误 | 话题映射不对 | 检查 remappings 和话题名 |

## 🎯 输出文件格式

标定结果保存为 YAML 格式：

```yaml
tool_to_camera:
  translation:
    x: 0.0523
    y: -0.0045
    z: 0.1234
  rotation:
    x: 0.0012
    y: 0.7071
    z: -0.0015
    w: 0.7070
  matrix_4x4: [r11, r12, ..., 1.0]
metadata:
  reprojection_error: 0.002345
  num_samples: 12
  timestamp: 1711270000
  units_translation: meter
  rotation_format: quaternion_xyzw
  transform_semantics: tool_to_camera
```

> 当前约定已统一为 `tool_to_camera`，历史旧命名已移除。

## 🧾 变更记录核对（按当前代码验证）

已核对并确认仍生效的历史改动：
- 采样姿态来源为 `/tool_pose`（非 `/target_pose`）
- 采样新鲜度拒绝：stale pose / stale image
- 棋盘边缘安全区拒绝：`board_margin_px`
- 稳定判定采用位姿窗口，且支持“目标后先检测到运动再允许判稳”
- 手动触发 `capture` / `compute`，不再依赖固定采样点数硬触发

## ✍️ 2026-03-30 本次简化改动

- 删除了节点内复杂在线筛选与离群重算参数路径（减少黑盒判定）
- `compute` 改为单次全量求解 + 明确一致性统计输出
- `test_board_pose` 返回结构化 message，便于自动统计
- YAML 输出统一为 `tool_to_camera` + `matrix_4x4` + 语义元数据
- 新增 `scripts/repeatability_check.py` 用于 3 次重复标定与静态漂移统计
- 重构根目录 `calib_pose_auto.sh`：移除复杂候选/随机逻辑，改为固定可复现实验流并自动归档结果

## 🔗 依赖

- **ROS2 Humble** (or compatible)
- **OpenCV** 4.x
- **Eigen3**
- **yaml-cpp**
- **MoveIt2**（可选，用于运动规划）

## 📚 参考资源

- [Tsai-Lenz 手眼标定](https://en.wikipedia.org/wiki/Hand%E2%80%93eye_calibration)
- [Intel RealSense D435i](https://www.intelrealsense.com/)
- [OpenCV PnP](https://docs.opencv.org/4.x/d5/d1f/calib3d_solvePnP.html)
- [ROS2 入门](https://docs.ros.org/en/humble/)

## 📄 许可

Apache-2.0

## 👨‍💻 维护

Arno <arno@example.com>

---

## 常见问题

**Q：可以使用其他棋盘尺寸吗？**  
A：可以。在 `config/hand_eye_calibration.yaml` 中修改 `checkerboard_rows`, `checkerboard_cols`, `square_size_mm`

**Q：能否使用其他相机？**  
A：可以。修改配置中的 `camera_fx`, `camera_fy`, `camera_cx`, `camera_cy` 参数

**Q：需要多少个标定样本？**  
A：至少 10 个，建议 15-20 个以获得更好的准确度

**Q：标定结果保存在哪里？**  
A：默认保存在 `/tmp/hand_eye_calibration_result.yaml`，可在配置中修改
