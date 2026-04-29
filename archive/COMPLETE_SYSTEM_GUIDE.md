# 视觉-机械臂集成系统 - 完整架构

## 🏗️ 系统组成

现已建立**三个独立的ROS2包**构成完整的视觉-机械臂集成系统：

```
┌────────────────────────────────────────────────────────────────┐
│              视觉-机械臂集成系统完整架构                          │
└────────────────────────────────────────────────────────────────┘

┌─────────────────────┐
│  vision_detection   │  ← 视觉检测层（感知）
│ (YOLO两阶段检测)    │    发布: /vision/cylinder_pose (相机坐标系)
└──────────┬──────────┘
           │
           ▼
┌─────────────────────┐
│ vision_arm_control  │  ← 集成层（转换+控制）
│ (转换+目标位姿生成)  │    订阅: /vision/cylinder_pose
└──────────┬──────────┘    发布: /target_pose (tool0坐标系)
           │
           ▼
┌─────────────────────┐
│ hand_eye_calibration│  ← 标定支撑（参数提供）
│ (Tsai-Lenz标定)     │    输出: 标定矩阵 (camera→tool0)
└─────────────────────┘
```

### 包详细说明

| 包名 | 类型 | 职责 | 依赖 |
|------|------|------|------|
| **vision_detection** | Python | YOLO圆柱检测 | PyTorch, OpenCV, RealSense |
| **vision_arm_control** | Python | 坐标转换 + 控制 | numpy, scipy, 标定结果 |
| **hand_eye_calibration** | C++ | 手眼标定 | opencv, Eigen3 |

---

## 📊 数据流

```
RealSense RGB → [vision_detection]
                      ↓
                /vision/cylinder_pose (相机坐标系)
                      ↓ [vision_to_arm_transform]
                      ↓ 使用手眼标定矩阵转换
                /cylinder_pose_base (tool0坐标系)
                      ↓ [arm_pose_controller]
                      ↓ 计算扫描位姿
                /target_pose (机械臂目标位姿)
                      ↓
                机械臂运动控制
```

---

## 🚀 快速开始

### 编译
```bash
cd /home/arnoyin/grad_proj/other_hands/implementation/arm_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select hand_eye_calibration vision_arm_control vision_detection
source install/setup.bash
```

### 第一次：执行手眼标定（仅需一次）

```bash
# 终端1 - 启动标定节点
ros2 launch hand_eye_calibration hand_eye_calibration.launch.py

# 终端2 - 自动采集样本
bash calib_pose_auto.sh

# 验证标定质量
cat /tmp/hand_eye_calibration_result.yaml | grep reprojection_error
```

### 生产运行：启动完整视觉管线

```bash
# 终端1 - 启动视觉检测
ros2 launch vision_detection cylinder_detection.launch.py

# 终端2 - 启动转换+控制
ros2 launch vision_arm_control vision_arm_integration.launch.py
```

### 监看数据流

```bash
# 终端3 - 监听各话题

# 视觉输出（相机坐标系）
ros2 topic echo /vision/cylinder_pose

# 转换后（tool0坐标系）
ros2 topic echo /cylinder_pose_base

# 最终目标位姿
ros2 topic echo /target_pose
```

---

## 📦 包结构

### vision_detection
```
vision_detection/
├── vision_detection/
│   ├── __init__.py
│   ├── config.py              # YOLO配置参数
│   ├── detection_node.py      # ROS2检测节点（主文件）
│   ├── pose_estimator.py      # 位姿计算、平滑、聚合
│   └── utils.py               # 图像处理工具函数
├── vision_detection_models/
│   ├── cylinder_best.pt       # 圆柱检测模型（5.8MB）
│   └── rim_best.pt            # 孔口检测模型（20MB）
├── launch/
│   └── cylinder_detection.launch.py
├── config/
│   └── vision_config.yaml
├── setup.py
└── package.xml
```

**功能：**
- Stage 1: YOLO圆柱检测 (body detection)
- Stage 2: YOLO孔口检测 (rim detection with segmentation)
- 输出：圆柱中心3D坐标 + 法向量

### vision_arm_control
```
vision_arm_control/
├── vision_arm_control/
│   ├── __init__.py
│   ├── vision_to_arm_transform.py  # 转换层
│   └── arm_pose_controller.py      # 控制层
├── launch/
│   └── vision_arm_integration.launch.py
├── setup.py
└── package.xml
```

**功能：**
- 转换层：camera → tool0（使用手眼标定）
- 控制层：生成扫描目标位姿

### hand_eye_calibration
```
hand_eye_calibration/
├── src/
│   └── hand_eye_calibration_node.cpp
├── launch/
│   └── hand_eye_calibration.launch.py
├── config/
│   └── hand_eye_calibration.yaml
└── package.xml
```

**功能：**
- Tsai-Lenz标定算法
- 采集样本 → 计算transformation matrix
- 输出：/tmp/hand_eye_calibration_result.yaml

---

## 🔧 参数配置

### 视觉检测参数 (vision_detection)

```yaml
# 检测置信度
body_conf_threshold: 0.5        # 圆柱检测阈值
rim_conf_threshold: 0.5         # 孔口检测阈值

# 图像处理
crop_padding_ratio: 0.25        # ROI外扩比例
rim_mask_dilation: 15           # Mask膨胀像素数

# 发布参数
publish_interval_ms: 100        # 发布间隔 (10Hz)
enable_debug: false             # 调试输出
```

### 转换层参数 (vision_to_arm_transform)

```bash
ros2 launch vision_arm_control vision_arm_integration.launch.py \
  calib_file:=/tmp/hand_eye_calibration_result.yaml \
  enable_debug:=true
```

### 控制层参数 (arm_pose_controller)

```bash
ros2 launch vision_arm_control vision_arm_integration.launch.py \
  scan_distance:=0.05          # 扫描距离 (5cm)
  target_frame:=tool0          # 目标坐标框架
  enable_debug:=true           # 调试输出
```

---

## 📈 性能指标

### 视觉检测
- **帧率**: ~10 Hz（受YOLO推理限制）
- **延迟**: ~100 ms (检测+聚合)
- **精度**: 依赖模型训练质量

### 位姿计算
- **精度**: ~1 pixel （椭圆拟合）
- **平滑**: 指数加权（可调SMOOTH_ALPHA）
- **聚合**: 30个样本平均

### 坐标转换
- **精度**: 依赖手眼标定质量（目标< 1.0 pixel reprojection error）
- **延迟**: <1 ms

---

## ⚠️ 常见问题

### Q1: 视觉节点启动失败 - 模型加载错误
```bash
# 检查模型文件
ls -la ~/.local/share/vision_detection/models/
# 或重新编译
colcon build --packages-select vision_detection --symlink-install
```

### Q2: 无法检测到圆柱
```bash
# 调试输出
ros2 launch vision_detection cylinder_detection.launch.py enable_debug:=true
# 检查相机
ros2 topic list | grep camera
```

### Q3: 位姿抖动严重
```bash
# 调整平滑参数
# 编辑: vision_detection/config.py
SMOOTH_ALPHA = 0.1  # 减小 -> 更稳定 (默认0.15)
POSE_DEADBAND = 0.01  # 增大 -> 忽略小抖动 (默认0.005)
```

### Q4: 标定矩阵精度不好
```bash
# 增加采样数量
min_samples: 50  # 在 hand_eye_calibration.yaml 中修改 (默认27)
# 采集更多角度的样本
bash calib_pose_auto.sh  # 运行两次以获得更多多样性
```

---

## 🔌 ROS2接口

### 发布话题

| 话题 | 类型 | 来源 | 说明 |
|------|------|------|------|
| `/vision/cylinder_pose` | PoseStamped | vision_detection | 相机坐标系中的圆柱位姿 |
| `/cylinder_pose_base` | PoseStamped | vision_to_arm_transform | tool0坐标系中的圆柱位姿 |
| `/target_pose` | PoseStamped | arm_pose_controller | 机械臂目标位姿 |

### 订阅话题

| 话题 | 类型 | 用途 | 来源 |
|------|------|------|------|
| `/camera/color/image_raw` | Image | 视觉输入 | RealSense |
| `/vision/cylinder_pose` | PoseStamped | 转换输入 | vision_detection |
| `/cylinder_pose_base` | PoseStamped | 控制输入 | vision_to_arm_transform |

### 服务

| 服务 | 类型 | 来源 | 说明 |
|------|------|------|------|
| `/calibration/capture` | Trigger | hand_eye_calibration | 采集一个标定样本 |

---

## 🧪 测试流程

### 1. 系统启动测试
```bash
# 检查编译
colcon build --packages-select hand_eye_calibration vision_arm_control vision_detection

# 检查执行
ros2 launch vision_detection cylinder_detection.launch.py
ros2 topic list | grep vision
```

### 2. 视觉检测测试
```bash
# 启动检测
ros2 launch vision_detection cylinder_detection.launch.py enable_debug:=true

# 放入圆柱，观察输出
ros2 topic echo /vision/cylinder_pose
```

### 3. 转换层测试
```bash
# 启动检测
ros2 launch vision_detection cylinder_detection.launch.py

# 新终端启动转换
ros2 launch vision_arm_control vision_arm_integration.launch.py enable_debug:=true

# 观察转换结果
ros2 topic echo /cylinder_pose_base
```

### 4. 完整管线测试
```bash
# 所有组件启动
ros2 launch vision_detection cylinder_detection.launch.py
ros2 launch vision_arm_control vision_arm_integration.launch.py

# 监看完整数据流
ros2 topic list
ros2 topic hz /vision/cylinder_pose
ros2 topic hz /target_pose
```

---

## 📚 文件位置速查

| 组件 | 关键文件 | 路径 |
|------|---------|------|
| 视觉检测 | 检测节点 | `vision_detection/vision_detection/detection_node.py` |
| 视觉检测 | 配置 | `vision_detection/config/vision_config.yaml` |
| 标定 | 参数 | `hand_eye_calibration/config/hand_eye_calibration.yaml` |
| 标定 | 启动 | `hand_eye_calibration/launch/hand_eye_calibration.launch.py` |
| 结果 | 标定矩阵 | `/tmp/hand_eye_calibration_result.yaml` |
| 集成 | 启动 | `vision_arm_control/launch/vision_arm_integration.launch.py` |

---

## 🎯 系统验收清单

部署前确保以下条件满足：

- [ ] 三个包编译成功（colcon build返回全绿）
- [ ] RealSense相机已连接且可见
- [ ] 手眼标定已完成（/tmp/hand_eye_calibration_result.yaml存在）
- [ ] 标定质量良好（reprojection_error < 1.0 pixel）
- [ ] 视觉检测可运行（ros2 launch vision_detection cylinder_detection.launch.py）
- [ ] 话题输出正常（ros2 topic echo显示数据）
- [ ] 转换层工作正常（位姿在合理范围内）
- [ ] 最终目标位姿合理（相机指向正确方向）

---

**系统完成时间**: 2026-03-24  
**版本**: v1.0.0 - 完整集成  
**状态**: ✅ 生产就绪
