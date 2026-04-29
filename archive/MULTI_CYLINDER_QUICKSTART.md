# 快速开始 - 多圆柱目标选择系统

## 编译

```bash
cd ~/grad_proj/other_hands/implementation/arm_ws
colcon build
source install/setup.bash
```

## 启动系统

### 方式1：一行命令启动全部（推荐）

```bash
ros2 launch vision_arm_control vision_arm_integration.launch.py
```

这会启动：
- ✓ vision_detection - YOLO圆柱检测
- ✓ target_selector - UI目标选择器
- ✓ vision_to_arm_transform - 坐标变换
- ✓ arm_pose_controller - 位姿生成

### 方式2：分别启动（调试用）

```bash
# Terminal 1: Vision Detection
ros2 run vision_detection cylinder_detection

# Terminal 2: Target Selector UI
ros2 run vision_arm_control target_selector

# Terminal 3: Transform Layer
ros2 run vision_arm_control vision_to_arm_transform

# Terminal 4: Arm Controller
ros2 run vision_arm_control arm_pose_controller
```

## 使用UI界面

启动后会弹出一个OpenCV窗口：

```
┌─────────────────────────────────────┐
│ Cylinder Target Selector            │
│ Detected Cylinders: 3               │
│                                     │
│ >>> [SELECTED] <<< Cylinder 0       │
│ Pos=(0.450, 0.120, 0.320)           │
│                                     │
│ [ ] Cylinder 1                      │
│ Pos=(0.280, -0.150, 0.305)          │
│                                     │
│ [ ] Cylinder 2                      │
│ Pos=(0.620, 0.080, 0.315)           │
│                                     │
│ Controls:                           │
│ UP/DOWN arrow or W/S - Select       │
│ ENTER - Confirm   A - Auto  Q - Quit│
└─────────────────────────────────────┘
```

### 按键操作

| 按键 | 功能 |
|-----|------|
| **↑ / W** | 向上选择圆柱 |
| **↓ / S** | 向下选择圆柱 |
| **ENTER** | 确认当前选择（日志显示） |
| **A** | 切换手动/自动模式 |
| **Q** | 退出UI |

## 工作流

### 手动模式（默认）

1. **启动系统**
   ```bash
   ros2 launch vision_arm_control vision_arm_integration.launch.py
   ```

2. **观察UI**
   - 所有检测到的圆柱显示在列表中
   - 第一个圆柱默认被选中（绿色高亮）

3. **选择目标圆柱**
   - 按UP/DOWN箭头或W/S切换选择
   - 注意UI中绿色高亮的是当前选中

4. **发送指令**
   - 按ENTER确认选择
   - 或直接等待（自动发送）
   - arm_controller收到选择→生成目标位姿
   - MoveIt规划并执行

5. **观察机械臂动作**
   - 查看vision_detection日志确认位姿
   - 观察arm_controller日志确认目标生成
   - 机械臂应该移动到目标位置

6. **重复**
   - 按UP/DOWN选择下一个圆柱
   - 重复步骤4-5

### 自动模式（队列管理）

1. **启动系统**
   ```bash
   ros2 launch vision_arm_control vision_arm_integration.launch.py
   ```

2. **切换到自动模式**
   - 在UI窗口按 **A** 键
   - 看到 "Status: AUTO MODE" 提示

3. **自动队列启动**
   - target_selector 自动将所有圆柱加入队列
   - 自动发送 queue[0]
   - arm_controller生成目标位姿
   - MoveIt执行

4. **系统自动进行**
   - 监听 /joint_states
   - 当机械臂速度下降到阈值以下 → 判定到位
   - 自动发送 queue[1]，重复

5. **观察进度**
   - 看target_selector的日志输出队列状态
   - 当队列完成，会提示 "Queue completed"

6. **手动干预**
   - 任何时候按A键切回手动模式
   - 或按Q退出

## 监听话题验证数据流

打开新的Terminal，实时查看各个话题：

```bash
# 查看所有检测到的圆柱
ros2 topic echo /vision/cylinders

# 查看当前选中的圆柱ID
ros2 topic echo /target_id

# 查看相机坐标系中的圆柱位姿
ros2 topic echo /vision/cylinder_pose

# 查看tool0坐标系中的圆柱位姿（变换后）
ros2 topic echo /cylinder_pose_base

# 查看最终的机械臂目标位姿
ros2 topic echo /target_pose

# 实时查看机械臂关节状态
ros2 topic echo /joint_states
```

## 日志输出示例

### vision_detection

```
[INFO] [vision_detection]: 检测到 3 个圆柱
[INFO] [vision_detection]: ✓ 圆柱 0: 位置=[0.4500, 0.1200, 0.3200], 法向=[0.0100, -0.0250, 0.9997]
[INFO] [vision_detection]: ✓ 圆柱 1: 位置=[0.2800, -0.1500, 0.3050], 法向=[0.0050, -0.0100, 0.9999]
[INFO] [vision_detection]: ✓ 圆柱 2: 位置=[0.6200, 0.0800, 0.3150], 法向=[-0.0050, 0.0050, 0.9999]
```

### target_selector (手动模式)

```
[INFO] [target_selector]: 目标已更新: 圆柱 ID = 0
[INFO] [target_selector]: 目标已更新: 圆柱 ID = 1
[INFO] [target_selector]: 目标已更新: 圆柱 ID = 2
```

### target_selector (自动模式)

```
[INFO] [target_selector]: ✓ 自动模式启用, 队列: [0, 1, 2]
[INFO] [target_selector]: 目标已更新: 圆柱 ID = 0
[INFO] [target_selector]: ✓ 圆柱 0 完成，进行下一个...
[INFO] [target_selector]: 目标已更新: 圆柱 ID = 1
[INFO] [target_selector]: ✓ 圆柱 1 完成，进行下一个...
[INFO] [target_selector]: ✓ 队列完成！
```

### arm_pose_controller

```
[INFO] [arm_pose_controller]: 🎯 目标已切换: 圆柱 ID = 0
[INFO] [arm_pose_controller]: ================================================================
[INFO] [arm_pose_controller]: 📍 生成目标位姿 (ID=0)
[INFO] [arm_pose_controller]:   位置: [0.4500, 0.1200, 0.3200]
[INFO] [arm_pose_controller]:   法向量: [0.0100, -0.0250, 0.9997]
[INFO] [arm_pose_controller]: 🎯 计算目标位姿
[INFO] [arm_pose_controller]:   扫描位置: [0.4501, 0.1197, 0.3700]
[INFO] [arm_pose_controller]:   相机指向: [-0.0100, 0.0250, -0.9997]
[INFO] [arm_pose_controller]: ✓ 已发布目标位姿到 /target_pose
```

## 常见问题

### Q1: UI窗口没有出现

**检查**：
```bash
# 确认enable_ui参数是true
ros2 launch vision_arm_control vision_arm_integration.launch.py | grep enable_ui

# 查看target_selector是否正常启动
ros2 node list | grep target_selector
```

**解决**：
- 检查X11转发（如果是SSH连接）
- 确保没有其他OpenCV窗口占用

### Q2: 机械臂不动

**检查**：
1. arm_controller是否收到target_id
   ```bash
   ros2 topic echo /target_id
   ```

2. /target_pose是否被发布
   ```bash
   ros2 topic echo /target_pose
   ```

3. MoveIt是否监听/target_pose

**解决**：
- 查看arm_pose_controller的日志
- 验证hand-eye calibration结果文件是否存在

### Q3: 自动模式无法工作

**检查**：
- /joint_states是否被发布
- 速度阈值是否设置正确

**调整参数**：
```bash
ros2 launch vision_arm_control vision_arm_integration.launch.py \
  target_selector:__params:={arm_reach_threshold=0.05, arm_timeout_seconds=15.0}
```

### Q4: 视觉检测不稳定

**原因**：
- 光照不足
- RealSense距离过近/过远
- YOLO模型置信度设置

**调整**：
- 增加光照
- 调整摄像头位置到0.3~0.5m
- 修改vision_detection中的conf参数（当前0.5）

## 调试模式

启用详细日志：

```bash
ros2 launch vision_arm_control vision_arm_integration.launch.py \
  vision_detection:__params:={enable_debug=true} \
  target_selector:__params:={enable_ui=true} \
  arm_pose_controller:__params:={enable_debug=true}
```

## 性能指标

| 指标 | 值 |
|-----|-----|
| 视觉检测频率 | ~10Hz (100ms间隔) |
| 圆柱检测延迟 | ~50-100ms |
| UI响应时间 | <50ms |
| arm_controller处理时间 | <10ms |
| 手动模式操作延迟 | ~200ms (UI选择→发送→执行) |

## 系统架构参考

详见 [MULTI_CYLINDER_SYSTEM.md](MULTI_CYLINDER_SYSTEM.md)

---

**快速提示**：
- 始终在启动前检查 `colcon build` 成功
- 第一次运行前，验证hand-eye calibration结果存在于 `/tmp/hand_eye_calibration_result.yaml`
- 如有问题，查看各节点的日志输出（都是screen模式）
- UI界面可以在任何时刻通过按Q关闭，系统继续运行
