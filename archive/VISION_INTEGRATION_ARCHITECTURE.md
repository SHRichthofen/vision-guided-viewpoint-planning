# 视觉-机械臂集成架构

## 📦 包结构

现已完全分离为两个独立的ROS2包：

### 1️⃣ `hand_eye_calibration` 包 - 纯标定

**职责：**
- ✅ 采集手眼标定样本
- ✅ 执行Tsai-Lenz标定算法
- ✅ 输出标定结果（YAML格式）

**关键节点：**
- `hand_eye_calibration_node` - 标定服务节点

**输出：**
- `/tmp/hand_eye_calibration_result.yaml` - 标定矩阵

**依赖：** 无Python依赖（纯C++）

---

### 2️⃣ `vision_arm_control` 包 - 视觉-机械臂集成

**职责：**
- 🔄 **转换层** - 坐标系转换（camera → tool0）
- 🎯 **控制层** - 目标位姿生成

**关键节点：**
1. `vision_to_arm_transform` - 转换层
   - 订阅：`/vision/cylinder_pose` (geometry_msgs/PoseStamped)
   - 发布：`/cylinder_pose_base` (geometry_msgs/PoseStamped)
   - 功能：使用手眼标定结果进行坐标转换

2. `arm_pose_controller` - 控制层
   - 订阅：`/cylinder_pose_base` (geometry_msgs/PoseStamped)
   - 发布：`/target_pose` (geometry_msgs/PoseStamped)
   - 功能：生成机械臂扫描目标位姿

**依赖：** Python (numpy, scipy, pyyaml)

---

## 🏗️ 数据流

```
视觉检测
    ↓
/vision/cylinder_pose (相机坐标系)
    ↓
[转换层] ← 加载手眼标定矩阵
    ↓
/cylinder_pose_base (tool0坐标系)
    ↓
[控制层] ← 计算扫描距离
    ↓
/target_pose (目标位姿)
    ↓
机械臂控制
```

---

## 🚀 使用方法

### 编译
```bash
cd /home/arnoyin/grad_proj/other_hands/implementation/arm_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select hand_eye_calibration vision_arm_control
```

### 启动
```bash
# 先运行手眼标定（一次）
ros2 run hand_eye_calibration hand_eye_calibration_node

# 然后运行集成节点（在另一个终端）
source install/setup.bash
ros2 launch vision_arm_control vision_arm_integration.launch.py
```

### 参数配置

**转换层参数** (`vision_to_arm_transform`):
```yaml
calib_file: '/tmp/hand_eye_calibration_result.yaml'  # 标定文件路径
enable_debug: true                                   # 调试输出
```

**控制层参数** (`arm_pose_controller`):
```yaml
scan_distance: 0.05           # 扫描距离 (m)
enable_debug: true            # 调试输出
target_frame: 'tool0'         # 目标坐标框架
```

---

## 📝 设计特点

### 分离性
- ✅ **手眼标定**完全独立，无关视觉控制
- ✅ **视觉控制**完全独立，可与任何标定系统集成
- ✅ 易于测试、维护和扩展

### 清晰性
- ✅ **单一职责** - 每个节点只做一件事
- ✅ **明确接口** - 标准ROS消息，易于集成
- ✅ **可调试性** - 每层都可独立测试

### 可扩展性
- ✅ 转换层可轻松扩展（添加更多坐标变换）
- ✅ 控制层可添加更复杂的运动规划
- ✅ 易于添加新的视觉算法

---

## 🔧 故障排查

### 转换层未收到视觉消息
```bash
# 检查视觉节点是否发布
ros2 topic echo /vision/cylinder_pose
```

### 标定文件未找到
```bash
# 确保手眼标定已完成
ros2 run hand_eye_calibration hand_eye_calibration_node
# 检查输出文件
cat /tmp/hand_eye_calibration_result.yaml
```

### 目标位姿不合理
```bash
# 启用调试输出，观察转换过程
ros2 launch vision_arm_control vision_arm_integration.launch.py enable_debug:=true
```

---

## 📚 后续工作

**下一阶段** (当前已完成转换层 + 控制层):
- [ ] QR码扫描模块（单独包）
- [ ] 多角度扫描策略
- [ ] 与实际硬件集成测试
- [ ] 性能优化和安全验证

---

**架构确认时间**: 2026-03-24  
**最后更新**: $(date)
