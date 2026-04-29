# 工作区集成指南

## 📋 当前工作区结构

```
arm_ws/
├── src/
│   ├── control/                          # 原有机械臂控制包
│   │   ├── src/pose_mover.cpp
│   │   ├── launch/
│   │   └── config/
│   │
│   └── hand_eye_calibration/            # 新建手眼标定包（独立）
│       ├── src/
│       │   ├── calibration_utils.cpp
│       │   ├── hand_eye_calibration_node.cpp
│       │   └── visual_servo_controller_node.cpp
│       ├── include/
│       │   └── calibration_utils.hpp
│       ├── launch/
│       │   ├── hand_eye_calibration.launch.py
│       │   └── visual_servo_demo.launch.py
│       ├── config/
│       │   └── hand_eye_calibration.yaml
│       ├── CMakeLists.txt
│       ├── package.xml
│       └── README.md
│
├── build/
└── install/
```

## 🔧 编译步骤

### 1. 编译新包

```bash
cd ~/arm_ws

# 只编译手眼标定包
colcon build --packages-select hand_eye_calibration

# 或编译所有包
colcon build
```

### 2. 刷新环境

```bash
source install/setup.bash
```

## 🚀 完整工作流程

### 工作流程图

```
┌─────────────────────────────────────────────────────────┐
│               完整的手眼标定与视觉伺服流程                 │
└─────────────────────────────────────────────────────────┘

阶段 1: 标定
─────────────────────────────────────
[相机驱动]
    ↓
[pose_mover_node] → 发布 /tool_pose
    ↓
[hand_eye_calibration_node] ← 订阅 /tool_pose 和相机图像
    │
    ├─ 检测棋盘角点
    ├─ PnP 求位姿
    ├─ 采集样本
    └─ 计算标定 (Tsai-Lenz)
    ↓
[标定结果] → /tmp/hand_eye_calibration_result.yaml


阶段 2: 视觉伺服
─────────────────────────────────────
[相机驱动]
    ↓
[pose_mover_node] → 发布 /tool_pose
    ↓
[visual_servo_controller_node] ← 订阅相机图像和 /tool_pose
    │
    ├─ 检测目标物体
    ├─ 相机 → 基座坐标变换
    └─ 发布 /target_pose_visual
    ↓
[pose_mover_node] ← 订阅 /target_pose_visual
    ↓
[机械臂移动到目标位置]
```

## 📝 实际使用步骤

### 步骤 1: 启动基础系统（终端 1）
```bash
# 启动相机驱动
ros2 launch realsense2_camera rs_launch.py rgb_camera.color_profile:=1280x720x30

# 或使用具体设备
ros2 launch realsense2_camera rs_launch.py serial_no:=YOUR_SERIAL_NUMBER
```

### 步骤 2: 启动机械臂控制（终端 2）
```bash
# 从 control 包启动机械臂控制
ros2 run control pose_mover_node
```

### 步骤 3: 启动标定（终端 3）
```bash
# 从 hand_eye_calibration 包启动标定
ros2 launch hand_eye_calibration hand_eye_calibration.launch.py
```

### 步骤 4: 采集标定数据（手柄或其他方式）
- 移动机械臂到不同位置（≥10个）
- 保持相机能清晰看到棋盘
- 节点自动计算并保存结果

### 步骤 5: 验证标定（终端 4）
```bash
# 启动视觉伺服演示
ros2 launch hand_eye_calibration visual_servo_demo.launch.py target_color:=red
```

## 🎯 设计优势

### 1. **模块化设计**
- ✅ 手眼标定与机械臂控制分离
- ✅ 易于独立测试和维护
- ✅ 可复用到其他机械臂系统

### 2. **清晰的架构**
```
hand_eye_calibration 包
├── 公共库      (calibration_utils)     → 可被其他包使用
├── 标定节点    (hand_eye_calibration_node)
└── 应用节点    (visual_servo_controller_node)
```

### 3. **易于扩展**
- 添加新的标定算法（不仅 Tsai-Lenz）
- 添加新的目标检测方法
- 支持其他相机和棋盘参数

### 4. **版本管理**
```
package.xml: version: 1.0.0
config/*.yaml: 参数版本控制
README.md: 文档跟踪
```

## 📊 话题映射

### 标定阶段

| 话题名 | 类型 | 方向 | 来源 |
|-------|------|------|------|
| `/tool_pose` | PoseStamped | 输入 | pose_mover |
| `/camera/color/image_raw` | Image | 输入 | 相机驱动 |
| `/calibration_result` | TransformStamped | 输出 | hand_eye_calib |

### 视觉伺服阶段

| 话题名 | 类型 | 方向 | 来源/目标 |
|-------|------|------|---------|
| `/tool_pose` | PoseStamped | 输入 | pose_mover |
| `/camera/color/image_raw` | Image | 输入 | 相机驱动 |
| `/target_pose_visual` | PoseStamped | 输出 | 发送到 pose_mover |
| `/debug_image` | Image | 输出 | 用于可视化 |

## 🔌 与现有系统集成

### 连接到 pose_mover

在 `pose_mover.cpp` 中已添加：
```cpp
// 发布末端位姿
tool_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
    "/tool_pose", 10);

// 订阅视觉伺服目标
subscription_ = this->create_subscription<geometry_msgs::msg::Pose>(
    "/target_pose_visual", 10, ...);
```

### 话题重映射

如果话题名不同，使用 `remappings` 调整：

```bash
# 启动时指定话题映射
ros2 run hand_eye_calibration hand_eye_calibration_node \
  --ros-args -r arm_pose:=/your_arm_pose_topic \
            -r camera_image:=/your_camera_topic
```

或在 launch 文件中：
```python
remappings=[
    ('arm_pose', '/custom_pose_topic'),
    ('camera_image', '/custom_image_topic'),
]
```

## 📦 包依赖关系

```
hand_eye_calibration
├── depends on: rclcpp, geometry_msgs, sensor_msgs, cv_bridge, OpenCV, Eigen3
└── used by: 机械臂控制系统（可选）

control (pose_mover)
├── depends on: moveit, DDS, Unitree SDK
└── provides: /tool_pose 话题
```

## 🧪 测试

### 单元测试（可选扩展）
```bash
# 编译测试
colcon build --packages-select hand_eye_calibration --cmake-args -DBUILD_TESTING=ON

# 运行测试
colcon test --packages-select hand_eye_calibration
```

### 集成测试
```bash
# 运行完整工作流程
ros2 launch hand_eye_calibration hand_eye_calibration.launch.py

# 验证标定结果
cat /tmp/hand_eye_calibration_result.yaml
```

## 📚 文件清单

### 新增文件
```
hand_eye_calibration/
├── CMakeLists.txt                      ← 编译配置
├── package.xml                         ← 包元数据
├── README.md                           ← 包文档
│
├── include/hand_eye_calibration/
│   └── calibration_utils.hpp           ← 库接口（可复用）
│
├── src/
│   ├── calibration_utils.cpp           ← 库实现
│   ├── hand_eye_calibration_node.cpp   ← 标定节点
│   └── visual_servo_controller_node.cpp ← 伺服节点
│
├── launch/
│   ├── hand_eye_calibration.launch.py
│   └── visual_servo_demo.launch.py
│
└── config/
    └── hand_eye_calibration.yaml
```

### 修改的文件
```
control/src/pose_mover.cpp
  - 添加了 tool_pose_pub_ 发布末端位姿
  - 在 publishJointStates() 中添加位姿计算
  - 添加了对视觉伺服目标的订阅支持
```

## 🎓 学习路径

1. **理解标定原理**
   - 阅读 README.md 中的原理说明
   - 查看 `calibration_utils.hpp` 的接口注释

2. **理解代码结构**
   - 研究 `CheckerboardDetector` 类
   - 研究 `TsaiLenzSolver` 类

3. **学习 ROS2 集成**
   - 查看 `hand_eye_calibration_node.cpp`
   - 查看话题映射和参数传递

4. **实践操作**
   - 运行标定
   - 调整参数观察效果
   - 验证视觉伺服

## 🔄 维护建议

### 定期更新检查
```bash
# 更新后重新编译
colcon build --packages-select hand_eye_calibration --cmake-clean-first
```

### 配置版本管理
```bash
# 备份标定结果
cp /tmp/hand_eye_calibration_result.yaml \
   ~/calibration_results/result_$(date +%Y%m%d_%H%M%S).yaml
```

### 日志记录
```bash
# 记录标定过程
ros2 launch hand_eye_calibration hand_eye_calibration.launch.py \
  > calibration_log_$(date +%Y%m%d_%H%M%S).log 2>&1
```

## 📞 故障排除

### 包找不到
```bash
# 重新 source 环境
source install/setup.bash

# 或重新编译
colcon build
source install/setup.bash
```

### 依赖缺失
```bash
# 安装依赖
rosdep install --from-paths src --ignore-src -y
```

### 话题连接失败
```bash
# 检查发布者/订阅者
ros2 topic list
ros2 topic echo /tool_pose
```

---

**最后**：如有问题或改进建议，欢迎反馈！✨
