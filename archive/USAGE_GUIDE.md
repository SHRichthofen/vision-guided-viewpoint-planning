# 视觉-机械臂集成系统使用指南

## 🏗️ 系统架构

该系统分为两个独立的部分：

### 1. **手眼标定**（按需，一次性）
- 包名：`hand_eye_calibration`
- 类型：C++ 节点
- 频率：**按需执行**（仅在标定矩阵需要更新时）
- 输出：`/tmp/hand_eye_calibration_result.yaml`

### 2. **视觉-控制集成管线**（生产运行）
- 包名：`vision_arm_control`
- 类型：Python 节点
- 频率：**连续运行**（在生产任务中）
- 依赖：手眼标定结果
- 功能：坐标转换 + 目标位姿生成

---

## 📋 工作流

```
                    ┌─────────────────────────┐
                    │   是否需要重新标定?      │
                    └──────────┬──────────────┘
                              │
                    ┌─────────┴─────────┐
                    │ YES              NO
                    ▼                  ▼
            [执行标定流程]      [生产管线运行]
```

---

## 🛠️ 第一步：编译

```bash
cd /home/arnoyin/grad_proj/other_hands/implementation/arm_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select hand_eye_calibration vision_arm_control
source install/setup.bash
```

---

## 📐 第二步：手眼标定（按需执行，首次必需）

### 启动标定节点
```bash
# 终端1
ros2 launch hand_eye_calibration hand_eye_calibration.launch.py
```

### 采集样本

方法A：**自动采集脚本**（推荐）
```bash
# 终端2 - 执行自动采集脚本
bash calib_pose_auto.sh
```

方法B：**手动采集**
```bash
# 每个位姿执行一次
ros2 service call /calibration/capture std_srvs/srv/Trigger
```

### 验证标定结果
```bash
# 查看输出文件
cat /tmp/hand_eye_calibration_result.yaml

# 验证标定质量（reprojection error < 1.0 pixel 为优）
grep "reprojection_error" /tmp/hand_eye_calibration_result.yaml
```

---

## 🎯 第三步：生产管线（连续运行）

### 启动视觉-控制集成节点

```bash
# 终端2 - 启动集成管线
ros2 launch vision_arm_control vision_arm_integration.launch.py
```

预期输出：
```
[vision_to_arm_transform-1] ============================================================
[vision_to_arm_transform-1] 转换层节点启动 (Vision-to-Arm Transform)
[vision_to_arm_transform-1] ============================================================
[vision_to_arm_transform-1] ✓ 转换层节点初始化成功
[vision_to_arm_transform-1] ✓ 手眼标定矩阵已加载

[arm_pose_controller-2] ============================================================
[arm_pose_controller-2] 控制层节点启动 (Arm Pose Controller)
[arm_pose_controller-2] ============================================================
[arm_pose_controller-2] ✓ 控制层节点初始化成功
```

### 数据流验证

```bash
# 终端3 - 监听各话题

# 1. 视觉检测输出（相机坐标系）
ros2 topic echo /vision/cylinder_pose

# 2. 转换层输出（tool0坐标系）
ros2 topic echo /cylinder_pose_base

# 3. 控制层输出（目标位姿）
ros2 topic echo /target_pose
```

---

## ⚙️ 参数配置

### 手眼标定配置
文件：`src/hand_eye_calibration/config/hand_eye_calibration.yaml`

```yaml
hand_eye_calibration: ros__parameters:
  # 棋盘参数
  checkerboard_rows: 8           # 棋盘行数（角数）
  checkerboard_cols: 8           # 棋盘列数（角数）
  square_size_mm: 33.0           # 方格大小（mm）
  
  # 相机参数（D435i）
  camera_fx: 906.948
  camera_fy: 905.906
  camera_cx: 648.379
  camera_cy: 383.873
  
  # 标定参数
  min_samples: 27                # 最少样本数
  output_file: /tmp/hand_eye_calibration_result.yaml
```

### 转换层配置
参数：

```bash
ros2 launch vision_arm_control vision_arm_integration.launch.py \
  calib_file:=/tmp/hand_eye_calibration_result.yaml \
  enable_debug:=true
```

### 控制层配置
参数：

```bash
ros2 launch vision_arm_control vision_arm_integration.launch.py \
  scan_distance:=0.05 \
  target_frame:=tool0 \
  enable_debug:=true
```

---

## 🔄 典型工作场景

### 场景1：首次使用（需要标定）
```bash
# 1. 编译
colcon build --packages-select hand_eye_calibration vision_arm_control

# 2. 执行标定
ros2 launch hand_eye_calibration hand_eye_calibration.launch.py
bash calib_pose_auto.sh

# 3. 验证标定质量
cat /tmp/hand_eye_calibration_result.yaml

# 4. 启动生产管线
ros2 launch vision_arm_control vision_arm_integration.launch.py
```

### 场景2：日常运行（已有标定）
```bash
# 直接启动生产管线（假设标定文件已存在）
ros2 launch vision_arm_control vision_arm_integration.launch.py
```

### 场景3：重新标定（相机位置变化）
```bash
# 重新执行标定流程
ros2 launch hand_eye_calibration hand_eye_calibration.launch.py
# ... 采集新样本
bash calib_pose_auto.sh

# 验证后重启生产管线（会自动加载新的标定）
ros2 launch vision_arm_control vision_arm_integration.launch.py
```

---

## 📊 监控和调试

### 启用完整调试输出
```bash
ros2 launch vision_arm_control vision_arm_integration.launch.py \
  enable_debug:=true
```

### 观察转换过程
```bash
# 查看相机→tool0的转换矩阵
grep "T_camera_to_tool" /tmp/hand_eye_calibration_result.yaml
```

### 性能指标
```bash
# 查看标定质量
grep -A2 "reprojection_error" /tmp/hand_eye_calibration_result.yaml

# 查看平移距离
grep "translation" /tmp/hand_eye_calibration_result.yaml

# 查看旋转四元数
grep "rotation" /tmp/hand_eye_calibration_result.yaml
```

---

## ⚠️ 常见问题

### Q: 启动生产管线时报"标定文件未找到"
A: 
```bash
# 确保已执行标定
ros2 launch hand_eye_calibration hand_eye_calibration.launch.py
# 检查文件是否存在
ls -la /tmp/hand_eye_calibration_result.yaml
```

### Q: 转换层收不到视觉消息
A:
```bash
# 检查视觉节点是否运行
ros2 topic list | grep vision/cylinder_pose
# 检查消息发布频率
ros2 topic hz /vision/cylinder_pose
```

### Q: 控制层输出的位姿不合理
A:
```bash
# 启用调试输出观察转换细节
ros2 launch vision_arm_control vision_arm_integration.launch.py enable_debug:=true
# 查看各层的中间结果
ros2 topic echo /cylinder_pose_base  # 转换后的位姿
ros2 topic echo /target_pose          # 最终目标
```

### Q: 需要修改扫描距离
A:
```bash
# 直接修改启动参数
ros2 launch vision_arm_control vision_arm_integration.launch.py scan_distance:=0.03
```

---

## 📚 相关文件

| 文件 | 用途 |
|------|------|
| `hand_eye_calibration/launch/hand_eye_calibration.launch.py` | 标定启动 |
| `hand_eye_calibration/config/hand_eye_calibration.yaml` | 标定参数 |
| `/tmp/hand_eye_calibration_result.yaml` | 标定结果 |
| `vision_arm_control/launch/vision_arm_integration.launch.py` | 生产管线启动 |
| `calib_pose_auto.sh` | 自动标定脚本 |

---

## ✅ 系统就绪检查列表

运行前确保以下条件满足：

- [ ] 编译成功：`colcon build` 返回全绿
- [ ] 相机已连接：`ros2 topic list | grep camera`
- [ ] 机械臂通信正常：`ros2 topic list | grep tool_pose`
- [ ] 标定文件存在：`ls /tmp/hand_eye_calibration_result.yaml`
- [ ] 标定质量良好：reprojection_error < 1.0 pixel

---

**最后更新**: 2026-03-24
